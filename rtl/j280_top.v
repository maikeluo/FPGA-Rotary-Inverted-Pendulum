`timescale 1ns/1ps
// v54r4: motion-relative outer feedback, measured arrival windows, qualified ADC-only recovery, identifiable telemetry.
module j280_top #(
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
 parameter integer DIAGNOSTIC=0, USE_XH1=1, ENCODER_REVERSE=1, MOTOR_REVERSE=0,
 parameter integer CLOCK_HZ=50000000, CONTROL_DIV=50000, ADC_HALF_DIV=10,
 parameter integer DEBOUNCE_TICKS=10, PWM_PERIOD=1250, PWM_DEAD_CYCLES=250, // 40kHz PWM; 250 clocks of neutral count on direction reversal.
 parameter integer RECOVERY_HOLD_TICKS=2000, RECOVERY_STABLE_TICKS=200, TELEMETRY_HZ=100
)(input clk_50m,input reset_n,input [3:0] key_n,input [1:0] dip_n,
 input [9:0] adc_data,input adc_otr,input enc1_a,input enc1_b,input enc2_a,input enc2_b,
 output adc_clk,output adc_oe_n,output pwma,output ain1,output ain2,
 output pwmb,output bin1,output bin2,output [3:0] led,output uart_tx_o,
 input uart_rx_o);
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
 adc_3pa1030 #(.HALF_DIV(ADC_HALF_DIV),.AUTOCLEAR_FAULT(1)) adc(
   clk_50m,rst_n,adc_data,adc_otr,adc_clk,adc_oe_n,raw,adc_valid,adc_ready,adc_bad,
   clear_fault,adc_range_bad);
 wire signed [31:0] position;wire [31:0] enc_errors;wire zero_encoder;
 wire [2:0] state;wire [7:0] fault;wire active;wire signed [15:0] control_command;
 wire signed [31:0] target,reference_q16;
 wire signed [31:0] travel_origin;wire [15:0] capture_ticks;wire [7:0] limit_mask;
 quadrature #(.REVERSE(ENCODER_REVERSE)) enc(clk_50m,rst_n,
   USE_XH1?enc1_a:enc2_a,USE_XH1?enc1_b:enc2_b,zero_encoder,position,enc_errors);
 wire signed [31:0] last_position,velocity_q8;
 wire [63:0] control_debug;
 encoder_velocity_filter velocity_filter(clk_50m,rst_n,tick,zero_encoder,clear_fault,
   position,last_position,velocity_q8);
 reg [31:0] last_errors,adc_age;reg encoder_bad,stall_bad;
 reg [9:0] stall_ticks,jog_ticks;
 reg [31:0] encoder_still_ticks;
 always @(posedge clk_50m or negedge rst_n)begin
   if(!rst_n)begin last_errors<=0;encoder_bad<=0;adc_age<=0;stall_bad<=0;stall_ticks<=0;jog_ticks<=0;encoder_still_ticks<=0;end
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
     if(tick)begin
       if(!(keys[0]^keys[1]))jog_ticks<=0;
       else if(jog_ticks<500)jog_ticks<=jog_ticks+1;
     end
   end
 end
 wire adc_timeout=adc_age>CLOCK_HZ/500;
 wire [1:0] key_short, key_long;
 wire calibrated,calibrating,sample;wire [9:0] upright_raw,down_raw;
 wire [7:0] sensor_status;wire signed [31:0] angle,angle_delta;
 angle_sensor #(.FIXED_CAL_ENABLE(FIXED_CAL_ENABLE),
   .FIXED_UP(FIXED_UP),.FIXED_DOWN(FIXED_DOWN)) sensor(clk_50m,rst_n,tick,raw,adc_ready,adc_bad|adc_timeout,
   key_long[0],key_long[1],state==0,calibrated,calibrating,zero_encoder,
   upright_raw,down_raw,sensor_status,angle,angle_delta,sample,clear_fault);
 fault_recovery #(.HOLD_TICKS(RECOVERY_HOLD_TICKS),.STABLE_TICKS(RECOVERY_STABLE_TICKS)) recovery(
   clk_50m,rst_n,tick,keys[1:0],!calibrating && !keys[2] && !keys[3] &&
     !encoder_bad && position==last_position && enc_errors==last_errors &&
     encoder_still_ticks>=RECOVERY_STABLE_TICKS,
   adc_valid,adc_ready,adc_timeout,adc_range_bad,raw,clear_fault,recovery_lock);
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
 wire healthy=calibrated && !calibrating && !encoder_bad && !stall_bad &&
               ((state==3'd1) || adc_health_ok);
 wire manual_start_evt=pressed[0] && !recovery_lock && !stop_held;
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
   .manual_stop(stop_held || keys[1] || recovery_lock),
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
 generate if(DIAGNOSTIC) begin: io_only
   // The interface image has no swing/capture/balance controller.
   assign state=3'd0;assign fault=8'd0;assign active=1'b0;
   assign control_command=16'sd0;assign target=32'sd0;assign reference_q16=32'sd0;
   assign travel_origin=32'sd0;assign capture_ticks=16'd0;assign limit_mask=8'd0;assign control_debug=64'd0;
 end else begin: closed_loop
  balance_control #(.TRAVEL_LIMIT(TRAVEL_LIMIT_COUNTS),.SWING_TRAVEL_LIMIT(TRAVEL_LIMIT_COUNTS),
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
      traj_ref,traj_active,manual_start_evt,{{16{traj_speed[15]}},traj_speed},1'b1);
 end endgenerate
 wire jog_enable=adc_ready && !adc_bad && !adc_timeout && !encoder_bad && (jog_ticks<500) && (keys[0]^keys[1]);
 wire signed [15:0] command=DIAGNOSTIC ? (keys[0]?16'sd80:-16'sd80):control_command;
 wire drive_enable=DIAGNOSTIC ? jog_enable : (!recovery_lock && !stop_held && active && key_n[1]);
 wire pwm,in1,in2;
 motor_pwm #(.PERIOD(PWM_PERIOD),.DEAD_CYCLES(PWM_DEAD_CYCLES),.REVERSE(MOTOR_REVERSE)) motor(
   clk_50m,rst_n,drive_enable,command,pwm,in1,in2);
 // Schematic 20260618: XH1=BO1/BO2=channel B; XH2=AO1/AO2=channel A.
 assign pwma=USE_XH1 ? 1'b1:pwm;
 assign ain1=USE_XH1 ? 1'b0:in1;assign ain2=USE_XH1 ? 1'b0:in2;
 assign pwmb=USE_XH1 ? pwm:1'b1;
 assign bin1=USE_XH1 ? in1:1'b0;assign bin2=USE_XH1 ? in2:1'b0;
  assign led={ (state==4) || traj_error, traj_arrived, active, adc_ready };
 wire [7:0] flags={recovery_lock,stall_bad,(DIAGNOSTIC!=0),adc_timeout,encoder_bad,adc_bad,calibrating,calibrated};
 telemetry #(.INTERVAL(CLOCK_HZ/TELEMETRY_HZ),.UART_DIV(CLOCK_HZ/115200),.EXTENDED(1),.DEBUG(2)) logger(
   clk_50m,rst_n,state,fault,sensor_status,flags,raw,angle,position,target,
   drive_enable?command:16'sd0,upright_raw,down_raw,uart_tx_o,
   angle_delta,velocity_q8,travel_origin,capture_ticks,{1'b0,ar_gave_up,traj_error,traj_arrived,traj_gated,limit_mask[2:0]},{2'b00,keys},
   drive_enable?control_debug:64'd0,
   {TRAJ_TARGET_COUNTS[15:0],traj_speed,traj_final,
    {1'b0,recovery_basic_hold,adc_only_trip,adc_health_ok,healthy,ar_armed,ar_gave_up,stop_evt},
    {4'd0,ar_retry},16'h5404});
endmodule

module j280_io_test(input clk_50m,input reset_n,input [3:0] key_n,input [1:0] dip_n,
 input [9:0] adc_data,input adc_otr,input enc1_a,input enc1_b,input enc2_a,input enc2_b,
 output adc_clk,output adc_oe_n,output pwma,output ain1,output ain2,
 output pwmb,output bin1,output bin2,output [3:0] led,output uart_tx_o,input uart_rx_o);
 j280_top #(.DIAGNOSTIC(1)) impl(clk_50m,reset_n,key_n,dip_n,adc_data,adc_otr,
 enc1_a,enc1_b,enc2_a,enc2_b,adc_clk,adc_oe_n,pwma,ain1,ain2,pwmb,bin1,bin2,led,uart_tx_o,uart_rx_o);
endmodule

