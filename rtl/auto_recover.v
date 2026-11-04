`timescale 1ns/1ps
// r15d: count actual launches. Only fall/timeout may retry.
// Manual stop/disable has priority; hardware faults require manual handling.
module auto_recover #(
 parameter integer ENABLE=1,HANG_ANGLE=160000,HANG_RATE=60,HANG_TICKS=1500,
 parameter integer IDLE_TICKS=300,BASE_WAIT=1000,WAIT_STEP=2000,MAX_WAIT=20000,
 parameter integer MAX_RETRY=10,GOOD_TICKS=30000
)(input clk,rst_n,tick,enable,manual_stop,manual_start,healthy,
 input [2:0] state,input [7:0] fault,
 input signed [31:0] angle,angle_delta,
 output reg auto_stop,auto_start,output reg [3:0] retry_count,
 output reg gave_up,armed,input adc_only_fault);
 localparam IDLE=0,SWING=1,CAPTURE=2,BALANCE=3,FAULT=4;
 localparam integer RETRY_LIMIT=(MAX_RETRY>15)?15:((MAX_RETRY<1)?1:MAX_RETRY);
 localparam integer INITIAL_WAIT=(BASE_WAIT>MAX_WAIT)?MAX_WAIT:BASE_WAIT;
 wire [31:0] abs_angle=angle<0?-angle:angle;
 wire [31:0] abs_rate=angle_delta<0?-angle_delta:angle_delta;
 wire hanging=abs_angle>HANG_ANGLE && abs_angle<=180000 && abs_rate<HANG_RATE;
 wire retryable=(fault==8'h08 || fault==8'h04 || (fault==8'h01 && adc_only_fault));
 reg session_enabled;
 reg [31:0] hang_cnt,idle_cnt,good_cnt,wait_need;
 wire [32:0] next_wait={1'b0,wait_need}+WAIT_STEP;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   auto_stop<=0;auto_start<=0;retry_count<=0;gave_up<=0;armed<=0;
   session_enabled<=0;hang_cnt<=0;idle_cnt<=0;good_cnt<=0;wait_need<=INITIAL_WAIT;
  end else begin
   auto_stop<=0;auto_start<=0;
   if(manual_stop || !ENABLE || !enable)begin
    session_enabled<=0;armed<=0;hang_cnt<=0;idle_cnt<=0;good_cnt<=0;
   end else if(manual_start && state==IDLE && healthy)begin
    session_enabled<=1;armed<=0;gave_up<=0;retry_count<=0;
    hang_cnt<=0;idle_cnt<=0;good_cnt<=0;wait_need<=INITIAL_WAIT;
   end else if(tick)begin
    if(state==BALANCE && healthy && abs_angle<8000 && abs_rate<200)begin
     if(good_cnt<GOOD_TICKS)good_cnt<=good_cnt+1'b1;
     if(good_cnt>=GOOD_TICKS-1)begin
      retry_count<=0;gave_up<=0;wait_need<=INITIAL_WAIT;
     end
    end else good_cnt<=0;
    if(!session_enabled)begin armed<=0;hang_cnt<=0;idle_cnt<=0;end
    else if(state==FAULT)begin
     idle_cnt<=0;armed<=0;
     if(!retryable || retry_count>=RETRY_LIMIT)begin
      gave_up<=1;hang_cnt<=0;
     end else if(!gave_up && healthy && hanging)begin
      if(hang_cnt>=HANG_TICKS+wait_need-1)begin
       auto_stop<=1;armed<=1;hang_cnt<=0;
      end else hang_cnt<=hang_cnt+1'b1;
     end else hang_cnt<=0;
    end else if(state==IDLE && armed)begin
     hang_cnt<=0;
     if(healthy && hanging && !gave_up)begin
      if(idle_cnt>=IDLE_TICKS-1)begin
       auto_start<=1;armed<=0;idle_cnt<=0;
       retry_count<=retry_count+1'b1;
       if(next_wait>MAX_WAIT)wait_need<=MAX_WAIT;else wait_need<=next_wait[31:0];
      end else idle_cnt<=idle_cnt+1'b1;
     end else idle_cnt<=0;
    end else begin armed<=0;hang_cnt<=0;idle_cnt<=0;end
   end
  end
 end
endmodule
