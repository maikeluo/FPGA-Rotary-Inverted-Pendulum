// Uploaded r54r2 reference: module names and declaration ordering normalized for standalone simulator compatibility; datapath unchanged.
`timescale 1ns/1ps
// v53r15: BALANCE horizontal velocity feedback split between fast/slow paths.
// Small-signal DC gain retained at defaults; reduced fast ripple coupling.
// No calibration rewriting or increase in total reference/PWM limits.
// r12 integral law retained; CAPTURE and SWING retain original D gain.
// The output coefficient and r11 leakage remain unchanged (no mode-step in I).
// Encoder velocity rounding is corrected separately in j280_top/quadrature.
// P and stored-I output coefficients are retained; BALANCE D is reduced.
// BALANCE I input retains r12 gain; rate curve, capture and swing are retained.
// Leakage is an empirical closed-loop tuning candidate, not a stability proof.
// This is an empirical tuning candidate, not verified mechanical stability.
// 1kHz control sample; command is PWM permille, rod angle is millidegrees.
// These are bench-tuned coefficients, not a validated physical plant model.
// New device timing and hardware acceptance are required for this release.
module balance_control_uploaded #(
 // Swing, P and outer settings are inherited; BALANCE D is scheduled below.
 // TRAVEL_LIMIT=200000 is inherited software behavior, NOT mechanical permission.
 // RETURN_* controls bounded slow bias and low-frequency reference damping.
 parameter integer KP_THETA=6554, KD_THETA=196608, KP_POSITION=16384, KD_VELOCITY=76800,
 parameter integer KD_VEL=20,
 parameter integer MAX_DUTY=400, SWING_DUTY=320, BOTTOM_SPEED=1200,
 parameter integer CAPTURE_ANGLE=8000, CAPTURE_RATE=200, FALL_ANGLE=30000,
 parameter integer ENERGY_MARGIN=20000,
 parameter integer SWING_TIMEOUT=15000, TRAVEL_LIMIT=200000, SWING_TRAVEL_LIMIT=780, SPEED_LIMIT=16,
 parameter integer SETTLE_TICKS=1000,
 parameter integer BASE_MAX_DUTY=300, CAPTURE_POSITION=5000, CAPTURE_VELOCITY_Q8=1000,
 parameter integer KI_THETA=6144, INT_LIMIT=8000000,
 // BALANCE only: subtract ceil(abs(I)/2048) toward zero per 1ms sample.
 // r10 comparison requires disabled leak AND both BALANCE_*_GAIN_Q8=256.
 // Valid leak shift range 1..30.
 parameter integer I_LEAK_ENABLE=1, I_LEAK_SHIFT=11,
 // BALANCE input gain Q8: 192/256. CAPTURE always uses 256/256.
 // 256 reproduces r11 I behavior. Out-of-range settings fall back to 256.
 parameter integer BALANCE_I_GAIN_Q8=192,
 // 208/256 = 81.25% of KD_THETA after 256 BALANCE ticks.
 // 256 reproduces r12 D gain. Invalid settings fall back to 256.
 parameter integer BALANCE_D_GAIN_Q8=208,
 parameter integer D_RESCUE_ENABLE=1,
 parameter integer BASE_VELOCITY_SPLIT_ENABLE=1,
 parameter integer TRIM_POSITION=-2, TRIM_VELOCITY=-8, TRIM_MAX=4000,
 parameter integer RATE_DEADBAND=8, TRIM_FILTER_SHIFT=5,
 // AFTER the existing rate dead band: slope 1 to 64 mdeg/ms, then 1/2.
 // CAPTURE/BALANCE only. Zero disables shaping for an r6 numerical comparison.
 parameter integer RATE_SOFT_ENABLE=1, RATE_SOFT_KNEE=64,
 parameter integer RETURN_ENABLE=1, RETURN_LIMIT=1000, RETURN_DEADBAND=16,
 parameter integer RETURN_PERSIST_ENABLE=1,
 parameter integer RETURN_SETTLE_TICKS=250,
 // Added only in BALANCE. Limits are rod-reference millidegrees.
 parameter integer RETURN_DAMP_ENABLE=1, RETURN_DAMP_GAIN=4, RETURN_DAMP_LIMIT=500,
 parameter integer RETURN_UNWIND_ENABLE=1,
 parameter integer POSITION_DEADBAND=0,  // v53r15q: 位置死区（计数）
 parameter integer VEL_FILTER_SHIFT=4,     // v53r15w: 速度反馈低通（1/2^N）
 parameter integer OUTER_SLEW_MDEG=0,         // v54r2: 外环参考变化限速（mdeg/ms，0=关闭）
 parameter integer OUTER_FAST_SLEW_MDEG=32    // v54r2: 摆杆偏离时放宽的限速
)(input clk,input rst_n,input tick,input start,input stop,input healthy,
 input signed [31:0] angle,input signed [31:0] angle_delta,
 input signed [31:0] position,input signed [31:0] velocity_q8,
 input [1:0] mode,output reg [2:0] state,output reg [7:0] fault,
 output reg signed [15:0] command,output active,
 output reg signed [31:0] target,output signed [31:0] reference_q16,
 input reset_reference,input recover_reference,
 output reg signed [31:0] travel_origin,output [15:0] capture_ticks,
 output [7:0] limit_mask,output reg [63:0] control_debug,
 // v53r15i: 拓展功能接口——外部（traj_gen）提供平滑后的目标位置
 input signed [31:0] target_ext,input target_ext_en);
 localparam IDLE=0,SWING=1,CAPTURE=2,BALANCE=3,FAULT=4;
 wire [31:0] abs_angle=(angle<0)?-angle:angle;
 wire [31:0] abs_rate=(angle_delta<0)?-angle_delta:angle_delta;
 wire speed_bad=velocity_q8>SPEED_LIMIT*256 || velocity_q8< -SPEED_LIMIT*256;
 wire rate_bad=abs_rate>5000;
 reg [31:0] age,settle;
 localparam integer BALANCE_D_SAFE=(BALANCE_D_GAIN_Q8>=0 && BALANCE_D_GAIN_Q8<=256)?
   BALANCE_D_GAIN_Q8:256;
 localparam signed [31:0] BALANCE_KD=(64'sd1*KD_THETA*BALANCE_D_SAFE)/256;
 localparam integer KD_RAMP_STEP=(KD_THETA-BALANCE_KD+255)/256;
 reg signed [31:0] kd_live,kd_sample;
 localparam integer KD_RECOVER_STEP=(KD_THETA-BALANCE_KD+7)/8;
 reg rescue_active;
 reg [7:0] rescue_quiet;
 wire rescue_alarm=D_RESCUE_ENABLE && (abs_angle>=3000 || abs_rate>=120);
 wire rescue_quiet_now=abs_angle<=1500 && abs_rate<=60;
 wire rescue_gain=D_RESCUE_ENABLE && (rescue_active || rescue_alarm);
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin rescue_active<=0;rescue_quiet<=0;end
   else if(!D_RESCUE_ENABLE || state!=BALANCE || stop || !healthy)begin
     rescue_active<=0;rescue_quiet<=0;
   end else if(tick)begin
     if(rescue_alarm)begin rescue_active<=1;rescue_quiet<=0;end
     else if(rescue_active && rescue_quiet_now)begin
       if(rescue_quiet>=249)begin rescue_active<=0;rescue_quiet<=0;end
       else rescue_quiet<=rescue_quiet+1'b1;
     end else rescue_quiet<=0;
   end
 end
 // Gain scheduling adds no temporal filter to the rate signal.
 // Snapshot BEFORE this tick's gain update, including the CAPTURE boundary.
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)kd_live<=KD_THETA;
   else if(state!=BALANCE || stop || !healthy)kd_live<=KD_THETA;
   else if(tick)begin
     if(rescue_gain)begin
       if(kd_live<KD_THETA-KD_RECOVER_STEP)kd_live<=kd_live+KD_RECOVER_STEP;
       else kd_live<=KD_THETA;
     end else if(kd_live>BALANCE_KD+KD_RAMP_STEP)kd_live<=kd_live-KD_RAMP_STEP;
     else kd_live<=BALANCE_KD;
   end
 end
 // Q8 blend: bring base feedback in over 256 samples (256 ms at 1 kHz).
 // This is independent of the 1000-sample BALANCE qualification counter.
 reg [8:0] hold_blend;
 reg [11:0] trim_ramp; // 0..2048 at +2/ms: 1.024s entry ramp
 reg signed [31:0] theta_int; // mdeg * ticks, committed once per completed hold sample
 // v53r5: integral authority cannot exceed the physical output budget.
 // This division is elaboration-time only; no run-time divider is inferred.
 localparam signed [63:0] INT_OUTPUT_LIMIT=(KI_THETA>0) ?
   (64'sd1*MAX_DUTY*16777216/KI_THETA):64'sd0;
 localparam integer EFFECTIVE_INT_LIMIT=(KI_THETA<=0 || INT_LIMIT<=0) ? 0:
   ((INT_OUTPUT_LIMIT<INT_LIMIT)?INT_OUTPUT_LIMIT:INT_LIMIT);
 reg signed [32:0] integral_sum,integral_adjusted;
 reg signed [31:0] integral_snapshot,integral_leak_step,integral_decayed;
 reg leak_sample,balance_i_sample;
 localparam integer BALANCE_I_GAIN_SAFE=(BALANCE_I_GAIN_Q8>=0 && BALANCE_I_GAIN_Q8<=256)?
   BALANCE_I_GAIN_Q8:256;
 reg signed [40:0] integral_error_product;
 // Nearest integer, ties away from zero, using only one rounding addition.
 // Negative (x+127)>>>8 is symmetric with positive (x+128)>>>8.
 wire signed [40:0] integral_error_rounded=(integral_error_product+
   (integral_error_product<0?41'sd127:41'sd128))>>>8;
 wire signed [32:0] integral_wide=$signed({integral_snapshot[31],integral_snapshot});
 localparam signed [32:0] LEAK_ROUND=(33'sd1<<I_LEAK_SHIFT)-1;
 // For negative I, arithmetic shift already equals -ceil(abs(I)/2^N).
 // Avoid an abs -> rounding-add -> negate chain in one clock stage.
 wire signed [32:0] positive_leak=(integral_wide+LEAK_ROUND)>>>I_LEAK_SHIFT;
 reg signed [31:0] integral_candidate;
 reg integral_block_s;
 // v53r15u: 启动时重设原点的触发条件（同时保留原 travel_bad 语义用于 SWING 保护）
 wire start_reference = start && (state==IDLE) && healthy && !(speed_bad || rate_bad);
 wire signed [32:0] travel_offset=$signed({position[31],position})-$signed({travel_origin[31],travel_origin});
 // Signed 33-bit differences avoid treating negative travel as unsigned.
 // Swing/hold limits are inherited; actual mechanics and cable travel need
 // independent confirmation. A state label is not a hardware acceptance test.
 wire signed [32:0] travel_lim=(state==SWING)?$signed(SWING_TRAVEL_LIMIT):$signed(TRAVEL_LIMIT);
 wire travel_bad=travel_offset>travel_lim || travel_offset< -travel_lim;
 // v12: base-centering bias, used ONLY while SWING commands the pump.
 // v11 hardware data: the pump's oscillation centre drifted ~40 encoder counts/s
 // (pos-origin 20 -> 570 over 14 s) until it ate the +-780 travel budget and the
 // swing collapsed (run 2 tripped fault=2 after 1.5 s). This applies a small DC
 // offset opposing that drift; it is inactive outside SWING because pump_s gates it.
 wire signed [32:0] swing_center_err=$signed({position[31],position})-$signed({travel_origin[31],travel_origin});
 reg signed [15:0] swing_bias;
 always @* begin
   if(swing_center_err>33'sd480)swing_bias=16'sd60;
   else if(swing_center_err< -33'sd480)swing_bias=-16'sd60;
 // v13 FIX: swing_center_err[15:0] is a PART-SELECT, which is UNSIGNED in Verilog,
 // so >>>3 became a logical shift and a small NEGATIVE error produced +8191
 // (pump_result = +-290 - 8191 = -8481, far beyond MAX_DUTY). Shift the signed
 // 33-bit value as a whole instead; the result stays within +-60 here.
   else swing_bias=swing_center_err>>>3;
 end
 wire signed [32:0] hold_offset=$signed({position[31],position})-$signed({target[31],target});
 // Position/velocity return loop: retain gains, +/-4deg cap and the
 // existing entry ramp, but reject its measured ~19Hz reference ripple.
 // 1/32 IIR at 1kHz gives ~32ms response time. Rod angle/rate are NOT filtered here.
 wire signed [31:0] base_trim,base_trim_pd,return_bias,return_damping,return_velocity;
 // Only split when the complementary slow branch exists. CAPTURE keeps TV.
 localparam integer SPLIT_ACTIVE=BASE_VELOCITY_SPLIT_ENABLE && RETURN_ENABLE && RETURN_DAMP_ENABLE && TRIM_MAX>0;
 localparam integer BALANCE_TV=SPLIT_ACTIVE ? TRIM_VELOCITY/2 : TRIM_VELOCITY;
 localparam integer DAMP_GAIN_LIVE=SPLIT_ACTIVE ? RETURN_DAMP_GAIN*2 : RETURN_DAMP_GAIN;
 localparam integer DAMP_LIMIT_LIVE=SPLIT_ACTIVE ? RETURN_DAMP_LIMIT*2 : RETURN_DAMP_LIMIT;
 generate if(RETURN_ENABLE && TRIM_MAX>0 && (RETURN_DAMP_ENABLE || RETURN_UNWIND_ENABLE))begin: return_motion
   base_return_damping_uploaded #(.GAIN(DAMP_GAIN_LIVE),.LIMIT(DAMP_LIMIT_LIVE)) damping(
     .clk(clk),.rst_n(rst_n),.enable(state==BALANCE && healthy && !stop),.tick(tick),
     .velocity_q8(velocity_q8),.filtered_velocity_q8(return_velocity),.trim(return_damping));
 end else begin: no_return_motion
   assign return_velocity=0;assign return_damping=0;
 end endgenerate
 wire hold_enabled=(state==CAPTURE || state==BALANCE) && healthy && !stop;
 base_trim_filter_uploaded #(.TP(TRIM_POSITION),.TV(TRIM_VELOCITY),.LIMIT(TRIM_MAX),
   .FILTER_SHIFT(TRIM_FILTER_SHIFT),.TV_BAL(BALANCE_TV),.POSITION_DEADBAND(POSITION_DEADBAND),.VEL_FILTER_SHIFT(VEL_FILTER_SHIFT)) outer_filter(
   clk,rst_n,hold_enabled,tick,position,target,velocity_q8,trim_ramp,base_trim_pd,state==BALANCE);
 generate if(RETURN_ENABLE && TRIM_MAX>0)begin: slow_return
   // Persistent mode keeps the center offset and uses qualified opposite
   // error to unwind it. Legacy mode retains r9's explicit zero release.
   // Both modes retain quiet/headroom checks and return-motion growth gating.
   reg signed [32:0] combined,pd_and_damping;
   reg signed [31:0] limited;
   wire signed [32:0] headroom_trim=RETURN_DAMP_ENABLE?pd_and_damping:
     $signed({base_trim_pd[31],base_trim_pd});
   wire ready_to_return=abs_angle<=3000 && abs_rate<=120 &&
     velocity_q8>=-128 && velocity_q8<=128 &&
     command>=-395 && command<=395 &&   // v53r15d: 原 300 太保守(实测 command 常摆到 ±344)，回位积分被永久挡住
     headroom_trim>=-(TRIM_MAX-RETURN_LIMIT) && headroom_trim<=(TRIM_MAX-RETURN_LIMIT);
   base_return_bias_uploaded #(.LIMIT_MDEG(RETURN_LIMIT),.DEADBAND(RETURN_DEADBAND),
     .SETTLE_TICKS(RETURN_SETTLE_TICKS),.UNWIND_ENABLE(RETURN_UNWIND_ENABLE),
     .PERSIST_ENABLE(RETURN_PERSIST_ENABLE)) bias(
     .clk(clk),.rst_n(rst_n),.enable(state==BALANCE && healthy && !stop),.tick(tick),
     .ready(ready_to_return),.position(position),.target(target),.bias(return_bias),
     .velocity_q8(return_velocity));
   // Separate sum and clamp registers keep the new reference path short.
   // At 1kHz, this settles long before the next inner-loop sample.
   always @(posedge clk or negedge rst_n)begin
     if(!rst_n)begin combined<=0;limited<=0;pd_and_damping<=0;end
     else if(!hold_enabled)begin combined<=0;limited<=0;pd_and_damping<=0;end
     else begin
       pd_and_damping<=$signed({base_trim_pd[31],base_trim_pd})+
         (RETURN_DAMP_ENABLE?$signed({return_damping[31],return_damping}):33'sd0);
       // The bypass keeps the exact r8 reference latency for regression.
       combined<=((RETURN_DAMP_ENABLE && state==BALANCE)?pd_and_damping:$signed({base_trim_pd[31],base_trim_pd}))+
         $signed({return_bias[31],return_bias});
       if(combined>TRIM_MAX)limited<=TRIM_MAX;
       else if(combined< -TRIM_MAX)limited<=-TRIM_MAX;
       else limited<=combined;
     end
   end
   // v54r2: 外环参考变化限速（移植自 v54r1 的 outer_reference_slew_uploaded）
   // 只限制水平外环产生的目标倾角，不限原始摆杆角/角速度、不限停止/故障处理。
   // 目的：位置误差突变时不让 θ_ref 瞬间跳到限幅，给内环留响应时间。
   outer_reference_slew_uploaded #(.NORMAL_STEP(OUTER_SLEW_MDEG),.FAST_STEP(OUTER_FAST_SLEW_MDEG)) reference_slew(
     clk,rst_n,tick,hold_enabled,state==BALANCE,angle,limited,base_trim);
 end else begin: no_slow_return
   assign base_trim=base_trim_pd;assign return_bias=32'sd0;
 end endgenerate
 wire base_position_settled=hold_offset<=CAPTURE_POSITION && hold_offset>=-CAPTURE_POSITION;
 wire base_velocity_settled=velocity_q8<=CAPTURE_VELOCITY_Q8 && velocity_q8>=-CAPTURE_VELOCITY_Q8;
 wire capture_qualified=abs_angle<CAPTURE_ANGLE && abs_rate<CAPTURE_RATE &&
                         base_position_settled && base_velocity_settled;
 // v14 DIAGNOSTIC: rate_bad is derived from the rod angle, which is NOT
 // trustworthy while the angle sensor is railed (it rails on every swing around
 // the bottom). During SWING it must not abort the pump or stall the pipeline.
 // Encoder-based travel_bad / speed_bad stay fully active, and limit_mask still
 // reports the raw rate_bad for telemetry.
 wire rate_bad_live=rate_bad && !(state==SWING);
 assign limit_mask={5'b0,rate_bad,speed_bad,travel_bad};
 assign capture_ticks=settle[15:0];
 // The soft travel budget belongs to the last deliberate manual recovery.
 // Ordinary STOP/START never moves this reference. Absolute encoder stays intact.
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)travel_origin<=0;
   else if(reset_reference)travel_origin<=0;
 else if(start_reference)travel_origin<=position;   // v53r15u: 每次启动把原点设到当前位置
   else if(recover_reference && stop)travel_origin<=position;
 end

 // Eight registered calculation stages after the sample snapshot.
 // No multicycle/false-path exception: each register path still has 20 ns.
 // One sample per 1 ms. A new sample while busy faults instead of using stale data.
 localparam integer BASE_BUDGET=(BASE_MAX_DUTY<MAX_DUTY)?BASE_MAX_DUTY:MAX_DUTY;
 localparam integer BASE_WIDTH=$clog2(BASE_BUDGET+1)+17;
 reg [3:0] calc_phase;
 reg signed [31:0] a_s,r_s,p_s,v_s,deadband_rate_s,feedback_rate_s;
 // Widen before the magnitude/offset arithmetic, including -2^31 inputs.
 // This combinational result is registered at phase 1; the D multiplier
 // consumes it at phase 2. No shaping+multiply chain is added to one clock.
 wire signed [32:0] deadband_rate_wide=$signed({deadband_rate_s[31],deadband_rate_s});
 wire signed [32:0] rate_positive_excess=deadband_rate_wide-RATE_SOFT_KNEE;
 wire signed [32:0] rate_negative_excess=-deadband_rate_wide-RATE_SOFT_KNEE;
 wire signed [32:0] rate_positive_soft=RATE_SOFT_KNEE+(rate_positive_excess>>>1);
 wire signed [32:0] rate_negative_soft=-33'sd1*RATE_SOFT_KNEE-(rate_negative_excess>>>1);
 reg signed [15:0] rate16_s;
 reg signed [31:0] cos_s;
 reg [8:0] blend_s;
 reg base_s,pump_s,seed_s,slow_s;
 reg signed [63:0] theta_product,rate_product,position_product,velocity_product;
 reg signed [63:0] kinetic,potential,base_sum,angle_sum,energy_s;
 reg signed [BASE_WIDTH-1:0] base_limited;
 reg signed [63:0] blend_product,total_q16,total_duty;
 reg signed [15:0] effort_limited,pump_result;
 reg signed [15:0] debug_trim_s,debug_bias_s,debug_damping_s,debug_i_s;
 wire signed [63:0] integral_product=64'sd1*KI_THETA*theta_int;
 reg signed [31:0] cos_q10;
 wire capture_entry=abs_angle<CAPTURE_ANGLE && abs_rate<CAPTURE_RATE;
 wire sample_fault=travel_bad || speed_bad || rate_bad_live ||
   ((state==CAPTURE || state==BALANCE) && abs_angle>FALL_ANGLE) ||
   (state==SWING && age>=SWING_TIMEOUT) || calc_phase!=0;
 wire cancel_calc=stop || !healthy || (state!=SWING && state!=CAPTURE && state!=BALANCE) ||
                   (tick && sample_fault);
 assign active=(state==SWING || state==CAPTURE || state==BALANCE) && healthy && !stop;
 assign reference_q16=target<<<16;
 always @* begin
   if(abs_angle<7500)cos_q10=1024;
   else if(abs_angle<22500)cos_q10=989;
   else if(abs_angle<37500)cos_q10=887;
   else if(abs_angle<52500)cos_q10=724;
   else if(abs_angle<67500)cos_q10=512;
   else if(abs_angle<82500)cos_q10=265;
   else if(abs_angle<97500)cos_q10=0;
   else if(abs_angle<112500)cos_q10=-265;
   else if(abs_angle<127500)cos_q10=-512;
   else if(abs_angle<142500)cos_q10=-724;
   else if(abs_angle<157500)cos_q10=-887;
   else if(abs_angle<172500)cos_q10=-989;
   else cos_q10=-1024;
 end
 // Valid/phase owns initialization. Datapath registers have no asynchronous reset,
 // allowing multiplier registers to map to device resources where supported.
 always @(posedge clk)begin
   if(rst_n && !cancel_calc)case(calc_phase)
     0:if(tick)begin
       a_s<=angle-base_trim;r_s<=angle_delta;p_s<=position-target;v_s<=velocity_q8;
       integral_snapshot<=theta_int;kd_sample<=kd_live;
       balance_i_sample<=state==BALANCE;
       leak_sample<=I_LEAK_ENABLE && I_LEAK_SHIFT>=1 && I_LEAK_SHIFT<=30 && state==BALANCE;
       debug_trim_s<=base_trim;debug_bias_s<=return_bias;
       debug_damping_s<=RETURN_DAMP_ENABLE?return_damping:32'sd0;
       // Snapshot the original dead-band result. No temporal rate filter is
       // added; preserve the eight-clock command latency.
       // Subtract the dead zone to avoid a step at either boundary.
       if(angle_delta>RATE_DEADBAND)deadband_rate_s<=angle_delta-RATE_DEADBAND;
       else if(angle_delta< -RATE_DEADBAND)deadband_rate_s<=angle_delta+RATE_DEADBAND;
       else deadband_rate_s<=0;
       rate16_s<=(angle_delta>32767)?16'sd32767:
         ((angle_delta< -32768)?-16'sd32768:angle_delta[15:0]);
       cos_s<=cos_q10;blend_s<=hold_blend;
       base_s<=state==CAPTURE || state==BALANCE;
       pump_s<=state==SWING && !capture_entry;seed_s<=age[7];slow_s<=abs_rate<8;
     end
     1:begin
       // Use the CURRENT snapshotted angle error, never last tick's a_s.
       // Keep old I in the same units through CAPTURE -> BALANCE.
       // Scale only new error, never the stored I or its output coefficient.
       if(balance_i_sample)integral_error_product<=41'sd1*BALANCE_I_GAIN_SAFE*a_s;
       else integral_error_product<=$signed({{9{a_s[31]}},a_s})<<<8;
       if(!leak_sample)integral_leak_step<=0;
       else integral_leak_step<=integral_snapshot<0?(integral_wide>>>I_LEAK_SHIFT):positive_leak;
       theta_product<=64'sd1*KP_THETA*a_s;
       // Continuous, sign-symmetric high-rate compression. Small signals
       // retain r6 gain. SWING always keeps the original rate handling.
       if(RATE_SOFT_ENABLE && RATE_SOFT_KNEE>0 && base_s)begin
         if(deadband_rate_wide>RATE_SOFT_KNEE)feedback_rate_s<=rate_positive_soft;
         else if(deadband_rate_wide< -RATE_SOFT_KNEE)feedback_rate_s<=rate_negative_soft;
         else feedback_rate_s<=deadband_rate_s;
       end else feedback_rate_s<=deadband_rate_s;
       position_product<=64'sd1*KP_POSITION*p_s;
       velocity_product<=64'sd1*KD_VELOCITY*v_s;
       kinetic<=$signed(rate16_s)*$signed(rate16_s);
     end
     2:begin
       rate_product<=64'sd1*kd_sample*feedback_rate_s;
       integral_sum<=$signed({integral_snapshot[31],integral_snapshot})+
         $signed(integral_error_rounded[32:0]);
       integral_decayed<=integral_snapshot-integral_leak_step;
       base_sum<=position_product+velocity_product;
       potential<=(64'sd1*BOTTOM_SPEED*BOTTOM_SPEED*(1024-cos_s))>>>11;
     end
     3:begin
       integral_adjusted<=integral_sum-$signed({integral_leak_step[31],integral_leak_step});
       angle_sum<=theta_product+rate_product;
       if(base_sum>64'sd1*BASE_BUDGET*65536)base_limited<=BASE_BUDGET*65536;
       else if(base_sum< -64'sd1*BASE_BUDGET*65536)base_limited<=-BASE_BUDGET*65536;
       else base_limited<=base_sum;
       energy_s<=kinetic-potential;
     end
     4:begin
       if(integral_adjusted>EFFECTIVE_INT_LIMIT)integral_candidate<=EFFECTIVE_INT_LIMIT;
       else if(integral_adjusted< -EFFECTIVE_INT_LIMIT)integral_candidate<=-EFFECTIVE_INT_LIMIT;
       else integral_candidate<=integral_adjusted;
       blend_product<=$signed(base_limited)*$signed({1'b0,blend_s});
       if(slow_s)pump_result<=(seed_s ? -SWING_DUTY:SWING_DUTY)-swing_bias;
       else if((r_s<0) ^ (cos_s<0))pump_result<=((energy_s<ENERGY_MARGIN)?SWING_DUTY:-SWING_DUTY)-swing_bias;
       else pump_result<=((energy_s<ENERGY_MARGIN)?-SWING_DUTY:SWING_DUTY)-swing_bias;
     end
     5:begin
       total_q16<=angle_sum+(integral_product>>>8)-(64'sd1*KD_VEL*v_s);
       debug_i_s<=integral_product>>>24; // arithmetic floor, PWM permille
     end
     6:total_duty<=total_q16>>>16;
     7:begin
       // Same sampled result as effort_limited. Freeze only if integration
       // would push an already limited output farther into saturation.
       // Opposite-sign error can unwind immediately, even while saturated.
       integral_block_s<=(total_duty>=MAX_DUTY && a_s>0) ||
                         (total_duty<=-MAX_DUTY && a_s<0);
       if(total_duty>MAX_DUTY)effort_limited<=MAX_DUTY;
       else if(total_duty< -MAX_DUTY)effort_limited<=-MAX_DUTY;
       else effort_limited<=total_duty;
     end
     default:begin end
   endcase
 end
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin calc_phase<=0;command<=0;control_debug<=0;end
   else if(cancel_calc)begin calc_phase<=0;command<=0;control_debug<=0;end
   else if(calc_phase==0)begin if(tick)calc_phase<=1;end
   else if(calc_phase==8)begin
     command<=pump_s?pump_result:effort_limited;calc_phase<=0;
     // Four signed int16 fields refer to THIS published command, not a later
     // integral commit or a newly updated outer-loop reference.
     control_debug<=base_s?{debug_damping_s,debug_bias_s,debug_trim_s,debug_i_s}:64'd0;
   end
   else calc_phase<=calc_phase+1'b1;
 end
 // Commit after the eight-clock computation, so error and saturation status
 // refer to one sample. Cancellation cannot publish a stale integral candidate.
 // Command uses the previous integral state; this update serves the next tick.
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)theta_int<=0;
   else if(cancel_calc || (state!=CAPTURE && state!=BALANCE))theta_int<=0;
   else if(calc_phase==8 && base_s)begin
     // Existing anti-windup blocks the error increment that would worsen
     // output saturation. Gradual removal of old history still proceeds.
     theta_int<=integral_block_s?integral_decayed:integral_candidate;
   end
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin state<=IDLE;fault<=0;age<=0;settle<=0;target<=0;hold_blend<=0;trim_ramp<=0;end
   else if(stop) begin state<=IDLE;fault<=0;age<=0;settle<=0;target<=position;hold_blend<=0;trim_ramp<=0;end
   else if(!healthy && state!=IDLE && state!=FAULT) begin state<=FAULT;fault<=8'h01;end
   else if(start && state==IDLE && healthy) begin
    // v53r15u: 每次启动把原点重设为当前位置 —— 起摆窗口永远居中，
    // 否则底座会像今天这样被拖到 3072 计数（远超 ±780 窗口）后再也起不来。
    // 原点已重设，travel_bad 必然为 0，预检只保留速度/角速度两项。
    if(speed_bad || rate_bad)begin state<=FAULT;fault<=8'h02;end
    else begin
        state<=(abs_angle<CAPTURE_ANGLE && abs_rate<CAPTURE_RATE)?CAPTURE:SWING;
        age<=0;settle<=0;fault<=0;target<=position;hold_blend<=0;trim_ramp<=0;
    end
   end else if(tick) begin
     if(state==SWING || state==CAPTURE || state==BALANCE) begin
       age<=age+1;
       // v18: 2 s slow ramp of the base position trim, only once the rod is held
       // v22: 1 s ramp instead of 2 s -- v17 showed the base covers ~550 counts of the
       // +-780 budget within the first 2 s, so a 2 s ramp intervenes too late.
       if((state==CAPTURE || state==BALANCE) && trim_ramp<2048)trim_ramp<=trim_ramp+2;
       if(travel_bad || speed_bad || rate_bad_live) begin
         state<=FAULT;fault<=8'h02;
       end else if(calc_phase!=0)begin state<=FAULT;fault<=8'h10;end
       else case(state)
         SWING:begin
           
           if(age>=SWING_TIMEOUT) begin state<=FAULT;fault<=8'h04;end
           else if(abs_angle<CAPTURE_ANGLE && abs_rate<CAPTURE_RATE) begin state<=CAPTURE;age<=0;target<=position;hold_blend<=0;end
         end
         CAPTURE:begin
           if(hold_blend<256)hold_blend<=hold_blend+1'b1;
           
           if(abs_angle>FALL_ANGLE) begin state<=FAULT;fault<=8'h08;end
           else if(capture_qualified) begin
             if(settle>=SETTLE_TICKS-1) begin state<=BALANCE;settle<=0;end
             else settle<=settle+1;
           end else settle<=0;
         end
         BALANCE:begin
           if(hold_blend<256)hold_blend<=hold_blend+1'b1;
           
           if(abs_angle>FALL_ANGLE) begin state<=FAULT;fault<=8'h08;end
           // v53r15i: 拓展——BALANCE 期间允许外部把【平滑后的目标】写入 target。
           // target 就是 base_trim_filter_uploaded 的参考，位置误差 = position - 平滑参考，
           // 天然很小，摆杆不会被目标跳变拽倒。
           else if(target_ext_en) target<=target_ext;
         end
       endcase
     end
   end
 end
endmodule

// Outer reference arithmetic is sampled, then split across registers. There
// is no position->multiply->clamp->ramp->angle path in one 50MHz clock anymore.
// FILTER_SHIFT=5 is 1/32 per CONTROL SAMPLE, so samples must remain 1kHz.
module base_trim_filter_uploaded #(
 parameter integer TP=-2,TV=-8,LIMIT=4000,FILTER_SHIFT=5,TV_BAL=TV,POSITION_DEADBAND=0,VEL_FILTER_SHIFT=4
)(input clk,input rst_n,input enable,input tick,
 input signed [31:0] position,target,velocity_q8,input [11:0] ramp,
 output signed [31:0] trim,input balance_mode);
 reg [3:0] phase;
 reg signed [32:0] offset_s;
 reg signed [31:0] velocity_s,limited_s,goal_q8,error_q8,accum_q8;
 reg [11:0] ramp_s;
 reg balance_s;
 reg signed [63:0] p_product,v_product,sum_s,ramp_product;
 // v53r4: Gowin mapped the old 64-bit negative constant multipliers
 // into a 61-level path (47.930 ns at the requested 20 ns clock).
 // For the shipped -2/-8 gains, express the exact arithmetic as wiring
 // plus a bounded-width two's-complement negate. Widen BEFORE shifting:
 // offset is signed 33 bits; -2*offset needs 35 bits, -8*velocity 36.
 // No narrowing of physical inputs, no saturation or gain change here.
  // v53r15q: 位置死区。|误差| <= POSITION_DEADBAND 时不产生位置反馈，
  // 用来切断"trim 慢慢积累 -> 突破静摩擦 -> 冲过去"的极限环。
  // 连续死区：减掉死区宽度，边界不产生台阶。
  // v53r15w: 速度反馈低通。TV 放大到 -64 后，编码器差分量化的噪声被放大 5.3 倍
  // 直接变成力矩抖动（用户反馈"来回幅度小了但有点抖"）。
  // 1/16 IIR，时间常数 16 ms —— 对 4 秒的摆动无影响，只滤高频。
  reg  signed [31:0] velocity_f;
  // r15E1: fractional Q8 filter state prevents an integer signed-shift
  // residue. Reset/STOP clears the complete filter. The extra arithmetic
  // stages complete within the existing eight-clock outer-loop pipeline.
  localparam integer VEL_SHIFT_SAFE=(VEL_FILTER_SHIFT<0)?0:
    ((VEL_FILTER_SHIFT>16)?16:VEL_FILTER_SHIFT);
  reg signed [40:0] velocity_accum_q8,velocity_error_q8,velocity_step_q8;
  wire signed [40:0] velocity_input_q8=$signed({velocity_q8[31],velocity_q8,8'd0});
  wire signed [40:0] velocity_step_next;
  generate if(VEL_SHIFT_SAFE==0)begin: velocity_no_shift
    assign velocity_step_next=velocity_error_q8;
  end else begin: velocity_rounded_shift
    localparam signed [40:0] HALF=41'sd1<<(VEL_SHIFT_SAFE-1);
    // Nearest, ties away from zero; symmetric for both directions.
    assign velocity_step_next=(velocity_error_q8+
      (velocity_error_q8<0?HALF-41'sd1:HALF))>>>VEL_SHIFT_SAFE;
  end endgenerate
  wire signed [40:0] velocity_output_wide=(velocity_accum_q8+
    (velocity_accum_q8<0?41'sd127:41'sd128))>>>8;
  always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
      velocity_accum_q8<=0;velocity_error_q8<=0;velocity_step_q8<=0;velocity_f<=0;
    end else if(!enable)begin
      velocity_accum_q8<=0;velocity_error_q8<=0;velocity_step_q8<=0;velocity_f<=0;
    end else case(phase)
      0:if(tick)velocity_error_q8<=velocity_input_q8-velocity_accum_q8;
      1:velocity_step_q8<=velocity_step_next;
      2:velocity_accum_q8<=velocity_accum_q8+velocity_step_q8;
      3:velocity_f<=velocity_output_wide[31:0];
      default:begin end
    endcase
  end

  reg  signed [32:0] offset_eff;   // 死区后的位置误差（在第 0 拍寄存，切断比较器长链）
  wire signed [63:0] position_scaled,velocity_scaled,velocity_normal,velocity_balance;
 generate
   if(TP == -2)begin: position_shift
     wire signed [34:0] shifted=$signed({offset_eff[32],offset_eff,1'b0});
     wire signed [34:0] negated=-shifted;
     assign position_scaled={{29{negated[34]}},negated};
   end else if(TP == -12)begin: position_twelve
     // v53r15x: TP=-12 = -(8+4)，移位相加。比 -8 推力大 1.5 倍，静差更小
     wire signed [35:0] s8=$signed({offset_eff[32],offset_eff,3'b000});
     wire signed [35:0] s4=$signed({offset_eff[32],offset_eff,2'b00});
     wire signed [36:0] sum=$signed({s8[35],s8})+$signed({s4[35],s4});
     wire signed [36:0] neg=-sum;
     assign position_scaled={{27{neg[36]}},neg};
   end else if(TP == -8)begin: position_eight
     // v53r15m: TP=-8 用移位实现（外环增益 x4，减小跟踪滞后）
     wire signed [35:0] shifted=$signed({offset_eff[32],offset_eff,3'b000});
     wire signed [35:0] negated=-shifted;
     assign position_scaled={{28{negated[35]}},negated};
   end else if(TP == -16)begin: position_sixteen
     // v53r15r: TP=-16（移位 4 位）。晃动幅度约等于 静摩擦所需trim/TP，
     // 提高 TP 才能缩小极限环（TP=-8 实测峰峰 192 计数）
     wire signed [36:0] shifted=$signed({offset_eff[32],offset_eff,4'b0000});
     wire signed [36:0] negated=-shifted;
     assign position_scaled={{27{negated[36]}},negated};
   end else begin: position_general
     // Compatibility for parameter overrides; recheck device timing if tuned.
     assign position_scaled=64'sd1*TP*offset_eff;
   end
   if(TV == -8)begin: velocity_shift
     wire signed [35:0] shifted=$signed({velocity_s[31],velocity_s,3'b000});
     wire signed [35:0] negated=-shifted;
     assign velocity_normal={{28{negated[35]}},negated};
   end else if(TV == -40)begin: velocity_forty
     // v53r15F: TV=-40 = -(32+8)。-64 对跳动刹车过猛造成粘滑，降到 -40
     wire signed [36:0] s32=$signed({velocity_s[31],velocity_s,5'b00000});
     wire signed [36:0] s8 =$signed({velocity_s[31],velocity_s,3'b000});
     wire signed [37:0] sum=$signed({s32[36],s32})+$signed({s8[36],s8});
     wire signed [37:0] neg=-sum;
     assign velocity_normal={{26{neg[37]}},neg};
   end else if(TV == -64)begin: velocity_sixtyfour
     // v53r15v: TV=-64（<<6 纯移位）。实测阻尼/比例只有 0.18，
     // 严重欠阻尼，这是 ±83 计数往复的真正原因。
     wire signed [37:0] shifted=$signed({velocity_s[31],velocity_s,6'b000000});
     wire signed [37:0] negated=-shifted;
     assign velocity_normal={{26{negated[37]}},negated};
    end else if(TV == -12)begin: velocity_twelve
      // v53r15e: -12 = -(8+4)，移位相加再取负
      wire signed [34:0] s8=$signed({velocity_s[31],velocity_s,3'b000});
      wire signed [34:0] s4=$signed({velocity_s[31],velocity_s,2'b00});
      wire signed [35:0] sum=$signed({s8[34],s8})+$signed({s4[34],s4});
      wire signed [35:0] neg=-sum;
      assign velocity_normal={{28{neg[35]}},neg};
    end else begin: velocity_general
     assign velocity_normal=64'sd1*TV*velocity_s;
   end
   if(TV_BAL == -4)begin: balance_velocity_shift
     wire signed [34:0] shifted=$signed({velocity_s[31],velocity_s,2'b00});
     wire signed [34:0] negated=-shifted;
     assign velocity_balance={{29{negated[34]}},negated};
   end else if(TV_BAL == -6)begin: balance_velocity_six
     // v53r15e: -6 = -(4+2)，移位相加再取负，避免通用 64 位常数乘法
     wire signed [34:0] s2=$signed({velocity_s[31],velocity_s,1'b0});
     wire signed [34:0] s4=$signed({velocity_s[31],velocity_s,2'b00});
     wire signed [35:0] sum=$signed({s2[34],s2})+$signed({s4[34],s4});
     wire signed [35:0] neg=-sum;
     assign velocity_balance={{28{neg[35]}},neg};
   end else if(TV_BAL == -64)begin: balance_velocity_sixtyfour
     wire signed [37:0] sh=$signed({velocity_s[31],velocity_s,6'b000000});
     wire signed [37:0] ng=-sh;
     assign velocity_balance={{26{ng[37]}},ng};
   end else if(TV_BAL == -12)begin: balance_velocity_twelve
     // v53r15p: 关掉速度分配后 TV_BAL=-12，用 -(8+4) 移位实现
     wire signed [34:0] s8=$signed({velocity_s[31],velocity_s,3'b000});
     wire signed [34:0] s4=$signed({velocity_s[31],velocity_s,2'b00});
     wire signed [35:0] sum=$signed({s8[34],s8})+$signed({s4[34],s4});
     wire signed [35:0] neg=-sum;
     assign velocity_balance={{28{neg[35]}},neg};
   end else begin: balance_velocity_general
     assign velocity_balance=64'sd1*TV_BAL*velocity_s;
   end
 endgenerate
 assign velocity_scaled=balance_s ? velocity_balance : velocity_normal;

 wire signed [31:0] step_q8=(FILTER_SHIFT==0)?error_q8:
   (error_q8>=0 ? ((error_q8+(1<<(FILTER_SHIFT-1)))>>>FILTER_SHIFT):
                    -(((-error_q8+(1<<(FILTER_SHIFT-1)))>>>FILTER_SHIFT)));
 // Signed rounding avoids a persistent negative one-mdeg offset at rest.
 assign trim=accum_q8>=0 ? ((accum_q8+128)>>>8):-(((-accum_q8+128)>>>8));
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin phase<=0;accum_q8<=0;end
   else if(!enable)begin phase<=0;accum_q8<=0;end
   else if(phase==0)begin if(tick)phase<=1;end
   else if(phase==7)begin accum_q8<=accum_q8+step_q8;phase<=0;end
   else phase<=phase+1'b1;
 end
 // Only phase/accumulator require reset; phase owns validity of the datapath.
 always @(posedge clk)begin
   if(rst_n && enable)case(phase)
     0:if(tick)begin
       offset_s<=$signed({position[31],position})-$signed({target[31],target});
        // v53r15q: 位置死区在第 0 拍就寄存，避免比较器+移位+乘法串成一条长链
        if(POSITION_DEADBAND <= 0)
            offset_eff<=$signed({position[31],position})-$signed({target[31],target});
        else if(($signed({position[31],position})-$signed({target[31],target})) > 33'sd1*POSITION_DEADBAND)
            offset_eff<=($signed({position[31],position})-$signed({target[31],target})) - 33'sd1*POSITION_DEADBAND;
        else if(($signed({position[31],position})-$signed({target[31],target})) < -33'sd1*POSITION_DEADBAND)
            offset_eff<=($signed({position[31],position})-$signed({target[31],target})) + 33'sd1*POSITION_DEADBAND;
        else offset_eff<=33'sd0;
       // velocity_f is updated by the resettable fractional filter above.
       balance_s<=balance_mode;velocity_s<=velocity_f;ramp_s<=(ramp>2048)?12'd2048:ramp;
     end
     1:begin p_product<=position_scaled;v_product<=velocity_scaled;end
     2:sum_s<=p_product+v_product;
     3:begin
       if(sum_s>LIMIT)limited_s<=LIMIT;
       else if(sum_s< -LIMIT)limited_s<=-LIMIT;
       else limited_s<=sum_s;
     end
     4:ramp_product<=64'sd1*limited_s*$signed({1'b0,ramp_s});
     5:goal_q8<=(ramp_product>>>11)<<<8;
     6:error_q8<=goal_q8-accum_q8;
     default:begin end
   endcase
 end
endmodule





// Slow position integral, Q13 mdeg, at one tick per 1ms. This is a bounded
// rod-reference correction; it never changes the absolute base target.
module base_return_bias_uploaded #(
 parameter integer LIMIT_MDEG=500,DEADBAND=16,SETTLE_TICKS=250,MAX_ERROR=400,
 parameter integer UNWIND_ENABLE=0,RETURN_SPEED_Q8=8,RELEASE_STEP_Q13=2048,
 parameter integer PERSIST_ENABLE=0
)(input clk,input rst_n,input enable,input tick,input ready,
 input signed [31:0] position,target,output signed [31:0] bias,
 input signed [31:0] velocity_q8);
 localparam signed [32:0] LIMIT_Q13=33'sd1*LIMIT_MDEG*8192;
 reg [31:0] quiet_ticks;
 reg [1:0] phase;
 reg signed [32:0] offset_s,candidate;
 wire signed [32:0] live_offset=$signed({position[31],position})-$signed({target[31],target});
 reg signed [31:0] increment,accum_q13;
 wire near_target=live_offset<=DEADBAND && live_offset>=-DEADBAND;
 wire wrong_bias=(live_offset>DEADBAND && accum_q13>0) ||
                 (live_offset< -DEADBAND && accum_q13<0);
 wire returning=(live_offset>DEADBAND && velocity_q8< -RETURN_SPEED_Q8) ||
                (live_offset< -DEADBAND && velocity_q8>RETURN_SPEED_Q8);
 // Persistent mode retains DC correction inside the target dead band.
 // Opposite position error unwinds it gradually instead of forcing zero.
 wire release_now=UNWIND_ENABLE && !PERSIST_ENABLE && (near_target || wrong_bias);
 wire growth_ready=ready && !(UNWIND_ENABLE && returning &&
   !(PERSIST_ENABLE && wrong_bias)) && !release_now;
 reg release_s;
 assign bias=(accum_q13>=0)?((accum_q13+4096)>>>13):-(((-accum_q13+4096)>>>13));
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin quiet_ticks<=0;phase<=0;accum_q13<=0;end
   else if(!enable)begin quiet_ticks<=0;phase<=0;accum_q13<=0;end
   else begin
     if(tick)begin
       if(!growth_ready)quiet_ticks<=0;
       else if(quiet_ticks<SETTLE_TICKS)quiet_ticks<=quiet_ticks+1'b1;
       // Reject overlapping samples before deciding to start new work.
       if(phase!=0)phase<=0;
       else if(release_now || (growth_ready && quiet_ticks>=SETTLE_TICKS))phase<=1;
     end else if(phase==3)begin
       if(candidate>LIMIT_Q13)accum_q13<=LIMIT_Q13;
       else if(candidate< -LIMIT_Q13)accum_q13<=-LIMIT_Q13;
       else accum_q13<=candidate;
       phase<=0;
     end else if(phase!=0)phase<=phase+1'b1;
   end
 end
 always @(posedge clk)begin
   if(rst_n && enable)case(phase)
     0:if(tick && (release_now || (growth_ready && quiet_ticks>=SETTLE_TICKS)))begin
       offset_s<=live_offset;release_s<=release_now;
     end
     1:begin
       // Clip before narrowing. Continuous dead band avoids a step at its edge.
       if(release_s)begin
         // Slew toward ZERO only, never jump across it or reverse the bias.
         if(accum_q13>RELEASE_STEP_Q13)increment<=-RELEASE_STEP_Q13;
         else if(accum_q13< -RELEASE_STEP_Q13)increment<=RELEASE_STEP_Q13;
         else increment<=-accum_q13;
       end
       else if(offset_s>DEADBAND+MAX_ERROR)increment<=-MAX_ERROR;
       else if(offset_s< -(DEADBAND+MAX_ERROR))increment<=MAX_ERROR;
       else if(offset_s>DEADBAND)increment<=DEADBAND-offset_s;
       else if(offset_s< -DEADBAND)increment<=-DEADBAND-offset_s;
       else increment<=0;
     end
     2:candidate<=$signed({accum_q13[31],accum_q13})+$signed({increment[31],increment});
     default:begin end
   endcase
 end
endmodule


// BALANCE-only low-frequency damping. Input is encoder counts/ms in Q8.
// 1/128 IIR rejects the fast velocity ripple before adding at most +/-0.5deg.
// At 1kHz the time constant is ~127.5ms; DC velocity gain adds -4 mdeg/Q8.
// Fixed-point fractions and symmetric rounding avoid directional DC bias.
module base_return_damping_uploaded #(
 parameter integer GAIN=4,LIMIT=500,FILTER_SHIFT=7,DEADBAND_Q8=4
)(input clk,input rst_n,input enable,input tick,
 input signed [31:0] velocity_q8,output reg signed [31:0] filtered_velocity_q8,
 output reg signed [31:0] trim);
 reg [2:0] phase;
 reg signed [31:0] input_q16,error_q16,step_q16,accum_q16;
 reg signed [63:0] product;
 wire signed [31:0] velocity_error=(filtered_velocity_q8>DEADBAND_Q8)?
   filtered_velocity_q8-DEADBAND_Q8:((filtered_velocity_q8< -DEADBAND_Q8)?
   filtered_velocity_q8+DEADBAND_Q8:32'sd0);
 wire signed [63:0] damping_scaled;
 generate if(GAIN==4)begin: gain_shift
   wire signed [34:0] shifted=$signed({velocity_error[31],velocity_error,2'b00});
   wire signed [34:0] negated=-shifted;
   assign damping_scaled={{29{negated[34]}},negated};
 end else if(GAIN==8)begin: gain_eight_shift
   wire signed [35:0] shifted=$signed({velocity_error[31],velocity_error,3'b000});
   wire signed [35:0] negated=-shifted;
   assign damping_scaled={{28{negated[35]}},negated};
 end else if(GAIN==10)begin: gain_ten_shift
   wire signed [36:0] times8=$signed({{2{velocity_error[31]}},velocity_error,3'b000});
   wire signed [36:0] times2=$signed({{4{velocity_error[31]}},velocity_error,1'b0});
   wire signed [36:0] sum10=times8+times2;
   wire signed [36:0] negated=-sum10;
   assign damping_scaled={{27{negated[36]}},negated};
 end else begin: gain_general
   assign damping_scaled=-64'sd1*GAIN*velocity_error;
 end endgenerate
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin phase<=0;accum_q16<=0;filtered_velocity_q8<=0;trim<=0;end
   else if(!enable)begin phase<=0;accum_q16<=0;filtered_velocity_q8<=0;trim<=0;end
   else if(tick)begin
     if(phase==0)phase<=1;else phase<=0; // cancel overlaps, no stale publish
   end else if(phase!=0)begin
     if(phase==3)accum_q16<=accum_q16+step_q16;
     if(phase==4)filtered_velocity_q8<=accum_q16>=0?
       ((accum_q16+128)>>>8):-(((-accum_q16+128)>>>8));
     if(phase==6)begin
       if(product>LIMIT)trim<=LIMIT;
       else if(product< -LIMIT)trim<=-LIMIT;
       else trim<=product;
       phase<=0;
     end else phase<=phase+1'b1;
   end
 end
 always @(posedge clk)begin
   if(rst_n && enable)case(phase)
     0:if(tick)begin
       // Match the inherited speed guard, with wide input protection even in
       // a standalone test. Bound before shifting, never truncate an input.
       if(velocity_q8>4096)input_q16<=1048576;
       else if(velocity_q8< -4096)input_q16<=-1048576;
       else input_q16<=velocity_q8<<<8;
     end
     1:error_q16<=input_q16-accum_q16;
     2:step_q16<=FILTER_SHIFT==0?error_q16:
       (error_q16>=0?((error_q16+(1<<(FILTER_SHIFT-1)))>>>FILTER_SHIFT):
       -(((-error_q16+(1<<(FILTER_SHIFT-1)))>>>FILTER_SHIFT)));
     5:product<=damping_scaled;
     default:begin end
   endcase
 end
endmodule


// v54r2: 外环参考变化限速（移植自 v54r1 的 outer_reference_slew_uploaded）
// 水平外环产生的目标倾角每 1 ms 最多变化 NORMAL_STEP mdeg；
// 摆杆偏离 |angle| >= FAST_ANGLE(3度) 时放宽到 FAST_STEP mdeg/ms。
// CAPTURE 阶段透明跟随原参考，BALANCE 从同一值开始，不会突然归零。
module outer_reference_slew_uploaded #(
 parameter integer NORMAL_STEP=8,FAST_STEP=32,FAST_ANGLE=3000
)(input clk,rst_n,tick,enable,balance_mode,
 input signed [31:0] angle,request,output signed [31:0] reference);
 generate if(NORMAL_STEP<=0)begin: bypass
  assign reference=request;
 end else begin: bounded
  reg signed [31:0] held;
  wire signed [32:0] difference=$signed({request[31],request})-$signed({held[31],held});
  wire [31:0] abs_angle=angle<0?-angle:angle;
  localparam integer FAST_SAFE=(FAST_STEP>NORMAL_STEP)?FAST_STEP:NORMAL_STEP;
  wire signed [32:0] step_limit=(abs_angle>=FAST_ANGLE)?FAST_SAFE:NORMAL_STEP;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)held<=0;
   else if(!enable)held<=0;
   else if(!balance_mode)held<=request;
   else if(tick)begin
    if(difference>step_limit)held<=$signed({held[31],held})+step_limit;
    else if(difference< -step_limit)held<=$signed({held[31],held})-step_limit;
    else held<=request;
   end
  end
  assign reference=!enable?32'sd0:(balance_mode?held:request);
 end endgenerate
endmodule
