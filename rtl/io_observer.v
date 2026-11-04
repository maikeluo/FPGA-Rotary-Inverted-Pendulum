`timescale 1ns/1ps
// Passive diagnostic evidence. Nothing in this module can command the motor
// or change the encoder position. Counts survive key release until reset;
// pwm_pulses is for the most recently accepted jog and resets on launch.
module io_observer(
 input clk,input rst_n,input [3:0] key_n,input [3:0] keys,
 input encoder_a,input encoder_b,input launch,input drive_enable,
 input pwm,input in1,input in2,
 output [63:0] debug,output [31:0] key_counts,
 output reg [15:0] pwm_pulses,output [7:0] key_seen);
 (* ASYNC_REG="TRUE" *) reg [5:0] meta,sync;
 reg [2:0] warmup;
 reg [1:0] previous_ab,last_direction;
 reg [3:0] previous_keys,raw_seen,filtered_seen;
 reg [15:0] a_edges,b_edges,s0_presses,s1_presses,launches;
 reg previous_pulse;
 // Disabled motor_pwm deliberately drives PWM high with both direction pins
 // low (coast). Do NOT report that idle level as a delivered PWM pulse.
 wire effective_pulse=drive_enable && pwm && (in1^in2);
 wire pulse_edge=effective_pulse && !previous_pulse;
 wire [15:0] io_bits={last_direction,drive_enable,pwm,in2,in1,sync[5:4],keys,sync[3:0]};
 assign debug={b_edges,a_edges,launches,io_bits};
 assign key_counts={s1_presses,s0_presses};
 assign key_seen={filtered_seen,raw_seen};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   meta<=0;sync<=0;warmup<=0;previous_ab<=0;previous_keys<=0;last_direction<=0;
   raw_seen<=0;filtered_seen<=0;a_edges<=0;b_edges<=0;
   s0_presses<=0;s1_presses<=0;launches<=0;pwm_pulses<=0;previous_pulse<=0;
  end else begin
   // sync[4]=A, sync[5]=B. These synchronizers are for observation only.
   meta<={encoder_b,encoder_a,~key_n};sync<=meta;
   previous_pulse<=effective_pulse;
   previous_ab<=sync[5:4];previous_keys<=keys;
   if(warmup<7)warmup<=warmup+1'b1;
   else begin
    raw_seen<=raw_seen | sync[3:0];filtered_seen<=filtered_seen | keys;
    if(sync[4]!=previous_ab[0] && a_edges!=16'hffff)a_edges<=a_edges+1'b1;
    if(sync[5]!=previous_ab[1] && b_edges!=16'hffff)b_edges<=b_edges+1'b1;
    if(keys[0] && !previous_keys[0] && s0_presses!=16'hffff)s0_presses<=s0_presses+1'b1;
    if(keys[1] && !previous_keys[1] && s1_presses!=16'hffff)s1_presses<=s1_presses+1'b1;
   end
   if(launch)begin
    if(launches!=16'hffff)launches<=launches+1'b1;
    last_direction<=keys[1:0];
    pwm_pulses<=pulse_edge?16'd1:16'd0;
   end else if(pulse_edge && pwm_pulses!=16'hffff)pwm_pulses<=pwm_pulses+1'b1;
  end
 end
endmodule
