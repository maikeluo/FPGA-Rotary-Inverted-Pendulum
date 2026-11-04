`timescale 1ns/1ps
// ACG720 r54r13: independent pendulum polarity and release/settle calibration.
module acg720_top #(
 // Optional measured boot calibration. Use scripts/configure_calibration.py.
 // r15a: only BALANCE slow damping tuned; 4 restores uploaded r15.
 parameter integer BALANCE_SLOW_DAMP_GAIN=5,
 parameter integer RETURN_LIMIT_V=1500,        // v53r15h: 0 = 关闭慢回位积分(bias)，只保留 P+D 与慢速阻尼
 parameter integer TRIM_VELOCITY_V=-64,
 parameter integer TRIM_POSITION_V=-12,
 parameter integer OUTER_SLEW_MDEG_V=8,        // v54r2: 外环参考限速 8 mdeg/ms
 parameter integer OUTER_FAST_SLEW_MDEG_V=32,  // v54r2: 摆杆偏离≥3度时 32 mdeg/ms
 parameter integer POS_DEADBAND_V=8, // Position feedback dead band, encoder counts.
 // ---- v53r15i 拓展功能参数 ----
 parameter integer TRAVEL_LIMIT_COUNTS=780, AUTO_RETRY_LIMIT=3,
 parameter integer TRAJ_ENABLE=1,              // 0 = 完全关闭拓展，恢复纯平衡
 parameter integer TRAJ_TARGET_COUNTS=400, // 400 counts ~= 138.46 degrees at 1040 counts/rev.
 parameter integer TRAJ_VMAX=4096, // Q16 reference velocity: 62.5 counts/s.
 parameter integer TRAJ_ACCEL=4, // Q16 acceleration: approximately 61.035 counts/s^2.
 parameter integer TRAJ_THETA_HOLD=4000, // 4-degree reference hold threshold.
 parameter integer TRAJ_THETA_ABORT=8000, // 8-degree recovery-wait threshold.
 parameter integer FIXED_CAL_ENABLE=0, FIXED_UP=0, FIXED_DOWN=0, // Measure this assembly first; configure_calibration.py can embed its verified pair.
 // r54r6: A405 log has opposite encoder acceleration in 402/436 qualified
 // reversals. Flip ENCODER only to restore the motor/position feedback sign.
 // Confirm on this wiring with the diagnostic image: S0 positive -> count rises.
 parameter integer DIAGNOSTIC=0, ENCODER_REVERSE=0, MOTOR_REVERSE=0,
 // log6 initial positive motor pulse: encoder increases but sensor angle falls.
 // Normalize the pendulum coordinate for BOTH swing and balance; do not invert
 // the motor/encoder feedback pair or merely reverse the swing output alone.
 parameter integer PENDULUM_REVERSE=1,
 parameter integer DOWN_CAL_STABLE_TICKS=1000, START_STABLE_TICKS=200,
 parameter integer ADC_AVERAGE_LOG2=9,
 // Diagnostic-only limits, expressed in 1 ms control ticks / encoder counts.
 parameter integer JOG_START_DUTY=80, JOG_MAX_DUTY=200, JOG_RAMP_STEP=3,
 parameter integer JOG_MAX_TICKS=500, JOG_NO_MOTION_TICKS=200,
 parameter integer JOG_TRAVEL_COUNTS=64, JOG_MIN_MOTION_COUNTS=4,
 parameter integer JOG_SPEED_Q8=512, JOG_REARM_TICKS=20,
 parameter integer CLOCK_HZ=50000000, CONTROL_DIV=50000, ADC_CHANNEL=0,
 parameter integer DEBOUNCE_TICKS=10, PWM_PERIOD=1250, PWM_DEAD_CYCLES=250, // 40kHz PWM; 250 clocks of neutral count on direction reversal.
 parameter integer RECOVERY_HOLD_TICKS=2000, RECOVERY_STABLE_TICKS=200, TELEMETRY_HZ=100
)(input clk_50m,input reset_n,input [3:0] key_n,input [1:0] dip_n,
 input adc_dout,input enc1_a,input enc1_b,
 output adc_sclk,output adc_din,output adc_cs_n,
 output pwma,output ain1,output ain2,output [3:0] led,output uart_tx_o);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe=2'b00;
 always @(posedge clk_50m or negedge reset_n)
   if(!reset_n)reset_pipe<=0;else reset_pipe<={reset_pipe[0],1'b1};
 wire rst_n=reset_pipe[1];wire tick;
 tick_gen #(.DIV(CONTROL_DIV)) timer(clk_50m,rst_n,tick);
 wire [5:0] keys,pressed;
 input_filter #(.WIDTH(6),.STABLE_TICKS(DEBOUNCE_TICKS)) inputs(
   clk_50m,rst_n,tick,{~dip_n,~key_n},keys,pressed);
 wire [9:0] raw;wire adc_valid,adc_ready,adc_bad,adc_range_bad;
 wire clear_fault,recovery_lock;
 adc128s102_acg720 #(.CHANNEL(ADC_CHANNEL),.AVERAGE_LOG2(ADC_AVERAGE_LOG2),.AUTOCLEAR_FAULT(1)) adc(
   .clk(clk_50m),.rst_n(rst_n),.dout(adc_dout),.sclk(adc_sclk),.din(adc_din),.cs_n(adc_cs_n),
   .raw(raw),.valid(adc_valid),.ready(adc_ready),.fault(adc_bad),
   .clear_fault(clear_fault),.range_bad(adc_range_bad));
 wire signed [31:0] position;wire [31:0] enc_errors;wire zero_encoder;
 wire [2:0] state;wire [7:0] fault;wire active;wire signed [15:0] control_command;
 wire signed [31:0] target,reference_q16;
 wire signed [31:0] travel_origin;wire [15:0] capture_ticks;wire [7:0] limit_mask;
 quadrature #(.REVERSE(ENCODER_REVERSE)) enc(clk_50m,rst_n,
   enc1_a,enc1_b,zero_encoder,position,enc_errors);
 wire signed [31:0] last_position,velocity_q8;
 wire [63:0] control_debug;
 encoder_velocity_filter velocity_filter(clk_50m,rst_n,tick,zero_encoder,clear_fault,
   position,last_position,velocity_q8);
 reg [31:0] last_errors,adc_age;reg encoder_bad,stall_bad;
 reg [9:0] stall_ticks;
 reg [31:0] encoder_still_ticks;
 always @(posedge clk_50m or negedge rst_n)begin
   if(!rst_n)begin last_errors<=0;encoder_bad<=0;adc_age<=0;stall_bad<=0;stall_ticks<=0;encoder_still_ticks<=0;end
   else begin
     if(adc_valid)adc_age<=0;else if(adc_age<CLOCK_HZ)adc_age<=adc_age+1;
     if(zero_encoder || clear_fault)begin
       // Recovery keeps the encoder count; re-prime against that same count.
       
       last_errors<=zero_encoder ? 32'd0:enc_errors;encoder_bad<=0;stall_bad<=0;stall_ticks<=0;encoder_still_ticks<=0;end
     else if(tick)begin
       if(position!=last_position || enc_errors!=last_errors)encoder_still_ticks<=0;
       else if(encoder_still_ticks<RECOVERY_STABLE_TICKS)encoder_still_ticks<=encoder_still_ticks+1;
       last_errors<=enc_errors;
       if(enc_errors-last_errors>4)encoder_bad<=1;
       if(active && (control_command>150 || control_command< -150) && position==last_position)begin
         if(stall_ticks<1000)stall_ticks<=stall_ticks+1;else stall_bad<=1;
       end else stall_ticks<=0;
     end
   end
 end
 wire adc_timeout=adc_age>CLOCK_HZ/500;
 wire [1:0] key_short, key_long;
 wire calibrated,calibrating,sensor_sample;wire [9:0] upright_raw,down_raw;
 wire [7:0] sensor_status;wire signed [31:0] sensor_angle,sensor_delta;
 reg signed [31:0] angle,angle_delta;reg sample;
 // Keep normalization off the controller's arithmetic path. The angle, rate
 // and valid strobe receive the same one-clock (20 ns) additional latency.
 always @(posedge clk_50m or negedge rst_n)begin
   if(!rst_n)begin angle<=0;angle_delta<=0;sample<=0;end
   else begin
     sample<=sensor_sample;
     if(sensor_sample)begin
       angle<=PENDULUM_REVERSE ? -sensor_angle:sensor_angle;
       angle_delta<=PENDULUM_REVERSE ? -sensor_delta:sensor_delta;
     end
   end
 end
 wire down_cal_pulse,down_cal_pending,down_wait_release,start_allowed,start_blocked,down_ready;
 angle_sensor #(.MAX_CAL_SPREAD(6),.FIXED_CAL_ENABLE(FIXED_CAL_ENABLE),
   .FIXED_UP(FIXED_UP),.FIXED_DOWN(FIXED_DOWN)) sensor(clk_50m,rst_n,tick,raw,adc_ready,adc_bad|adc_timeout,
   key_long[0],down_cal_pulse,state==0 && !DIAGNOSTIC,calibrated,calibrating,zero_encoder,
   upright_raw,down_raw,sensor_status,sensor_angle,sensor_delta,sensor_sample,clear_fault);
 generate if(!DIAGNOSTIC)begin: normal_recovery
 fault_recovery #(.HOLD_TICKS(RECOVERY_HOLD_TICKS),.STABLE_TICKS(RECOVERY_STABLE_TICKS)) recovery(
   clk_50m,rst_n,tick,keys[1:0],!calibrating && !keys[2] && !keys[3] &&
     !encoder_bad && position==last_position && enc_errors==last_errors &&
     encoder_still_ticks>=RECOVERY_STABLE_TICKS,
   adc_valid,adc_ready,adc_timeout,adc_range_bad,raw,clear_fault,recovery_lock);
 end else begin: jog_no_recovery
   // In the diagnostic image S1 means reverse jog, not a recovery gesture.
   assign clear_fault=1'b0;assign recovery_lock=1'b0;
 end endgenerate
 // v40: was "SW1 and SW2 both held". Now SW2 alone is the recovery / stop button, so a
 // single long press of SW2 clears the fault latch and re-references the travel origin.
 // v53r15j: 拨码难拨 -> 改用 SW3/SW4 按键控制拓展目标
 key_len #(.LONG_TICKS(1200)) klen(
   clk_50m,rst_n,tick,keys[3:2],pressed[3:2],key_short,key_long);
 wire stop_held=!key_n[1];
 // v53r15b: sensor_status.bit0 是"本次采样无效"的瞬时标志。摆杆在下垂位摆动时
 // ADC 会撞轨(raw=0/1023)，该标志恒为 1。旧写法只豁免 SWING/CAPTURE/BALANCE，
 // 漏了 IDLE/FAULT -> 摔倒后 healthy=0 -> 连手动 SW1 都无法启动。
 // 改为只认锁存故障(adc_bad=OTR、adc_timeout)。
 wire adc_health_ok=adc_ready && !adc_bad && !adc_timeout && raw>0 && raw<1023;
 wire healthy=calibrated && !calibrating && !down_cal_pending && !down_cal_pulse && !encoder_bad && !stall_bad &&
               ((state==3'd1) || adc_health_ok);
 wire manual_start_request=pressed[0] && !recovery_lock && !stop_held && !keys[2] && !keys[3];
 wire manual_start_evt=manual_start_request && start_allowed && healthy;
 acg720_cal_guard #(.DOWN_STABLE_TICKS(DOWN_CAL_STABLE_TICKS),.START_STABLE_TICKS(START_STABLE_TICKS)) cal_guard(
   .clk(clk_50m),.rst_n(rst_n),.tick(tick),.idle(state==0 && !DIAGNOSTIC),
   .stop_request(stop_held || keys[1] || recovery_lock),
   .up_request(key_long[0]),.down_request(key_long[1]),.down_key_held(keys[3]),
   .adc_ok(adc_health_ok && !adc_range_bad),.calibrated(calibrated),.calibrating(calibrating),
   .position_still(position==last_position && enc_errors==last_errors),.raw(raw),
   .angle(angle),.rate(angle_delta),.start_request(manual_start_request),
   .down_cal_pulse(down_cal_pulse),.down_pending(down_cal_pending),.wait_release(down_wait_release),
   .start_allowed(start_allowed),.start_blocked(start_blocked),.down_ready(down_ready));
 // Remember the cause at the actual health-trip edge. Encoder/stall or
 // calibration faults must never be relabelled as recoverable ADC trips.
 reg adc_only_trip;
 always @(posedge clk_50m or negedge rst_n)begin
   if(!rst_n)adc_only_trip<=0;
   else if(stop_held || recovery_lock || manual_start_evt)adc_only_trip<=0;
   else if((state==1 || state==2 || state==3) && !healthy)
     adc_only_trip<=calibrated && !calibrating && !encoder_bad && !stall_bad && !adc_health_ok;
 end
 wire ar_auto_stop,ar_auto_start,ar_gave_up,ar_armed;
 wire [3:0] ar_retry;
 auto_recover #(.MAX_RETRY(AUTO_RETRY_LIMIT)) autorec(
   .clk(clk_50m),.rst_n(rst_n),.tick(tick),.enable(DIAGNOSTIC==0),
   .manual_stop(stop_held || keys[1] || recovery_lock || key_long[0] || key_long[1] || down_cal_pending),
   .manual_start(manual_start_evt && state==0 && healthy),
   .healthy(healthy && adc_health_ok),.state(state),.fault(fault),
   .angle(angle),.angle_delta(angle_delta),.auto_stop(ar_auto_stop),.auto_start(ar_auto_start),
   .retry_count(ar_retry),.gave_up(ar_gave_up),.armed(ar_armed),.adc_only_fault(adc_only_trip));
 wire start_evt=manual_start_evt || ar_auto_start;
 wire stop_evt=keys[1] || recovery_lock || stop_held || ar_auto_stop;
 reg recovery_basic_hold;
 reg [1:0] previous_mode;
 always @(posedge clk_50m or negedge rst_n)begin
   if(!rst_n)begin recovery_basic_hold<=0;previous_mode<=0;end
   else begin
     previous_mode<=keys[5:4];
     if(ar_auto_start)recovery_basic_hold<=1;
     else if(manual_start_evt || keys[5:4]!=previous_mode)recovery_basic_hold<=0;
   end
 end
  //=========== 拓展：拨码定点 + 梯形轨迹 + 摆杆门控 ===========
  wire [1:0] mode_sel = recovery_basic_hold ? 2'b00 : keys[5:4];
  wire signed [31:0] traj_ref, traj_final;
  wire signed [15:0] traj_speed;
  wire traj_busy, traj_arrived, traj_gated, traj_error;
  wire traj_enable = (state==3'd3) && !stop_evt && (TRAJ_ENABLE!=0);
  wire traj_active = (state==3'd3) && (TRAJ_ENABLE!=0) && !stop_evt;
  // v53r15n: 去掉串口命令（L20 悬空收到的噪声字节会被当成 '-' 命令，
  //          导致目标一直被减、底座永远追不上）。目标只由拨码决定。
  wire step_plus_i  = 1'b0;
  wire step_minus_i = 1'b0;
  wire go_home_i    = 1'b0;
  traj_gen #(.VMAX(TRAJ_VMAX),.ACCEL(TRAJ_ACCEL),
             .TARGET_COUNTS(TRAJ_TARGET_COUNTS),
             .THETA_HOLD(TRAJ_THETA_HOLD),.THETA_ABORT(TRAJ_THETA_ABORT)) traj(
    clk_50m,rst_n,tick,traj_enable,mode_sel,position,travel_origin,
    angle,angle_delta,
    step_plus_i, step_minus_i, go_home_i,
    traj_ref,traj_final,traj_speed,
    traj_busy,traj_arrived,traj_gated,traj_error,velocity_q8);
 wire jog_enable;wire signed [15:0] jog_command;
 wire signed [31:0] jog_origin;wire [15:0] jog_elapsed;
 wire [7:0] jog_fault,jog_status;wire jog_launch;
 wire [63:0] io_debug;wire [31:0] io_key_counts;
 wire [15:0] io_pwm_pulses;wire [7:0] io_seen;
 wire pwm,in1,in2;
 generate if(DIAGNOSTIC) begin: io_only
   // The interface image has no swing/capture/balance controller.
   diagnostic_jog #(.START_DUTY(JOG_START_DUTY),.MAX_DUTY(JOG_MAX_DUTY),.RAMP_STEP(JOG_RAMP_STEP),
     .MAX_TICKS(JOG_MAX_TICKS),.NO_MOTION_TICKS(JOG_NO_MOTION_TICKS),
     .TRAVEL_COUNTS(JOG_TRAVEL_COUNTS),.MIN_MOTION_COUNTS(JOG_MIN_MOTION_COUNTS),
     .SPEED_Q8(JOG_SPEED_Q8),.REARM_TICKS(JOG_REARM_TICKS)) jog(
     .clk(clk_50m),.rst_n(rst_n),.tick(tick),.keys(keys[1:0]),.raw_keys(~key_n[1:0]),
     .healthy(adc_ready && !adc_bad && !adc_timeout && !encoder_bad),
     .position(position),.velocity_q8(velocity_q8),.enable(jog_enable),.command(jog_command),
     .origin(jog_origin),.elapsed(jog_elapsed),.fault(jog_fault),.status(jog_status),.launch_pulse(jog_launch));
   io_observer observer(.clk(clk_50m),.rst_n(rst_n),.key_n(key_n),.keys(keys[3:0]),
     .encoder_a(enc1_a),.encoder_b(enc1_b),.launch(jog_launch),.drive_enable(jog_enable),
     .pwm(pwm),.in1(in1),.in2(in2),.debug(io_debug),.key_counts(io_key_counts),
     .pwm_pulses(io_pwm_pulses),.key_seen(io_seen));
   assign state=3'd0;assign fault=jog_fault;assign active=1'b0;
   assign control_command=16'sd0;assign target=jog_origin;assign reference_q16=jog_origin<<<16;
   assign travel_origin=jog_origin;assign capture_ticks=jog_elapsed;assign limit_mask=jog_status;assign control_debug=64'd0;
 end else begin: closed_loop
  assign jog_enable=1'b0;assign jog_command=16'sd0;assign jog_origin=32'sd0;
  assign jog_elapsed=16'd0;assign jog_fault=8'd0;assign jog_status=8'd0;assign jog_launch=1'b0;
  assign io_debug=64'd0;assign io_key_counts=32'd0;assign io_pwm_pulses=16'd0;assign io_seen=8'd0;
  balance_control #(.SWING_FILTER_ENABLE(1),.SWING_COHERENT_ENABLE(1),
                     .TRAVEL_LIMIT(TRAVEL_LIMIT_COUNTS),.SWING_TRAVEL_LIMIT(TRAVEL_LIMIT_COUNTS),
                     .RETURN_DAMP_GAIN(BALANCE_SLOW_DAMP_GAIN),
                     .RETURN_LIMIT(RETURN_LIMIT_V),.RETURN_DEADBAND(12),.RETURN_SETTLE_TICKS(500),
                     .TRIM_VELOCITY(TRIM_VELOCITY_V),
                     .BASE_VELOCITY_SPLIT_ENABLE(0),   // Preserve uploaded unsplit TRIM_VELOCITY=-64.
                     .TRIM_POSITION(TRIM_POSITION_V),
                     .POSITION_DEADBAND(POS_DEADBAND_V),
                     .OUTER_SLEW_MDEG(OUTER_SLEW_MDEG_V),
                     .OUTER_FAST_SLEW_MDEG(OUTER_FAST_SLEW_MDEG_V),
                     .VEL_FILTER_SHIFT(5),.MOTION_FEEDBACK_ENABLE(1)) control(clk_50m,rst_n,sample,start_evt,
      stop_evt,healthy,angle,angle_delta,position,velocity_q8,2'b00,
      state,fault,control_command,active,target,reference_q16,
      zero_encoder,clear_fault,travel_origin,capture_ticks,limit_mask,control_debug,
      traj_ref,traj_active,manual_start_evt,{{16{traj_speed[15]}},traj_speed},adc_health_ok && !adc_range_bad);
 end endgenerate
 wire signed [15:0] command=DIAGNOSTIC ? jog_command:control_command;
 wire drive_enable=DIAGNOSTIC ? jog_enable : (!recovery_lock && !stop_held && active && key_n[1]);
 motor_pwm #(.PERIOD(PWM_PERIOD),.DEAD_CYCLES(PWM_DEAD_CYCLES),.REVERSE(MOTOR_REVERSE)) motor(
   clk_50m,rst_n,drive_enable,command,pwm,in1,in2);
 // ACG720 wiring v4: A channel only; STBY is tied to 3.3V externally.
 assign pwma=pwm;
 assign ain1=in1;
 assign ain2=in2;
  assign led=DIAGNOSTIC ? {jog_fault!=0,jog_status[4],jog_enable,adc_ready}:
    {(state==4) || traj_error,traj_arrived,active,adc_ready};
 wire [7:0] flags={recovery_lock,stall_bad,(DIAGNOSTIC!=0),adc_timeout,encoder_bad,adc_bad,(calibrating || down_cal_pending),calibrated};
 // A413 normal image only: high bits are calibration UI status, not faults.
 // bit4=pendulum reversed; bit5=DOWN pending; bit6=release S3; bit7=start refused.
 wire [7:0] sensor_report=DIAGNOSTIC ? sensor_status :
   sensor_status | {start_blocked,down_wait_release,down_cal_pending,(PENDULUM_REVERSE!=0),4'd0};
 telemetry #(.INTERVAL(CLOCK_HZ/TELEMETRY_HZ),.UART_DIV(CLOCK_HZ/115200),.EXTENDED(1),.DEBUG(2)) logger(
   clk_50m,rst_n,state,fault,sensor_report,flags,raw,DIAGNOSTIC?io_key_counts:angle,position,target,
   drive_enable?command:16'sd0,upright_raw,down_raw,uart_tx_o,
   DIAGNOSTIC?enc_errors:angle_delta,velocity_q8,travel_origin,capture_ticks,
   DIAGNOSTIC ? jog_status : {1'b0,ar_gave_up,traj_error,traj_arrived,traj_gated,limit_mask[2:0]},{2'b00,keys},
   DIAGNOSTIC?io_debug:(drive_enable?control_debug:64'd0),
   {(DIAGNOSTIC ? JOG_TRAVEL_COUNTS[15:0]:TRAJ_TARGET_COUNTS[15:0]),
    (DIAGNOSTIC ? io_pwm_pulses:traj_speed),(DIAGNOSTIC ? jog_origin:traj_final),
    (DIAGNOSTIC ? io_seen:{1'b0,recovery_basic_hold,adc_only_trip,adc_health_ok,healthy,ar_armed,ar_gave_up,stop_evt}),
    {4'd0,ar_retry},16'hA413});
endmodule

// Same physical ports as the normal image. Hold S0/S1 for positive/negative
// 8 -> 20% ramp over 40ms while held. Release stops immediately. Each press
// is limited to 0.5s / 64 counts; <4 counts after 200ms stops with fault 0x20.
// Both keys must be released for 20ms before any new jog (including at boot).
module acg720_io_test(input clk_50m,input reset_n,input [3:0] key_n,input [1:0] dip_n,
 input adc_dout,input enc1_a,input enc1_b,
 output adc_sclk,output adc_din,output adc_cs_n,
 output pwma,output ain1,output ain2,output [3:0] led,output uart_tx_o);
 acg720_top #(.DIAGNOSTIC(1)) impl(
 .clk_50m(clk_50m),.reset_n(reset_n),.key_n(key_n),.dip_n(dip_n),
 .adc_dout(adc_dout),.enc1_a(enc1_a),.enc1_b(enc1_b),
 .adc_sclk(adc_sclk),.adc_din(adc_din),.adc_cs_n(adc_cs_n),
 .pwma(pwma),.ain1(ain1),.ain2(ain2),.led(led),.uart_tx_o(uart_tx_o));
endmodule
