`timescale 1ns/1ps
// v54 shared reference for basic home hold, left/right targets and shuttle.
// Arrival means BOTH reference completion AND measured settling. A timeout
// pauses the task; it never forces a direction reversal before actual arrival.
module traj_gen #(
 parameter integer VMAX=4096,ACCEL=4,TARGET_COUNTS=400,DWELL_TICKS=500,
 parameter integer ARRIVE_BAND=1,THETA_HOLD=4000,THETA_ABORT=8000,
 parameter integer STEP_COUNTS=400,MAX_STEP=400,
 parameter integer ACT_BAND=16,ACT_VEL_Q8=8,ACT_SETTLE=300,
 parameter integer REV_TIMEOUT=15000,CALM_TICKS=100,STILL_SPAN=3
)(input clk,rst_n,tick,enable,input [1:0] mode,
 input signed [31:0] actual,origin,angle,angle_delta,
 input step_plus,step_minus,go_home,
 output reg signed [31:0] ref_pos,final_target,
 output reg signed [15:0] speed_q8,
 output reg busy,arrived,gated,error,
 input signed [31:0] velocity_q8);
 reg pp_positive,recovering,timed_out;
 reg [1:0] mode_d;
 reg signed [31:0] tgt_lat;
 reg signed [63:0] p_q16,v_q16,dv_r,v_next;
 reg [2:0] ph;
 reg [31:0] dwell,wait_count,calm,inband;
 reg [63:0] velocity_sum;reg window_settled;
 reg signed [31:0] still_lo,still_hi;
 reg signed [31:0] tgt_cnt;
 always @* case(mode)
  0:tgt_cnt=origin;
  1:tgt_cnt=origin+TARGET_COUNTS;
  2:tgt_cnt=origin-TARGET_COUNTS;
  3:tgt_cnt=pp_positive?origin+TARGET_COUNTS:origin-TARGET_COUNTS;
 endcase
 wire signed [63:0] goal_q16=$signed({{32{tgt_lat[31]}},tgt_lat})<<<16;
 wire signed [63:0] actual_q16=$signed({{32{actual[31]}},actual})<<<16;
 wire signed [63:0] distance=goal_q16-p_q16;
 wire [63:0] abs_distance=distance<0?-distance:distance;
 wire [31:0] abs_v=v_q16<0?-v_q16[31:0]:v_q16[31:0];
 wire [63:0] product=abs_v*(abs_v+ACCEL);
 localparam integer DIV=2*ACCEL,SHIFT=$clog2(DIV);
 wire [63:0] stopping;
 generate if((1<<SHIFT)==DIV)begin: stop_shift
  assign stopping=product>>SHIFT;
 end else begin: stop_divide
  assign stopping=product/DIV;
 end endgenerate
 reg [63:0] stopping_r;
 reg signed [63:0] dv;
 always @* begin
  if(distance==0)dv=0;
  else if((distance>0 && v_q16<0)||(distance<0 && v_q16>0))dv=distance>0?VMAX:-VMAX;
  else if(abs_distance<=stopping_r)dv=0;
  else dv=distance>0?VMAX:-VMAX;
 end
 wire [31:0] abs_theta=angle<0?-angle:angle;
 wire [31:0] abs_rate=angle_delta<0?-angle_delta:angle_delta;
 wire [31:0] abs_velocity=velocity_q8<0?-velocity_q8:velocity_q8;
 wire signed [32:0] actual_error=$signed({actual[31],actual})-$signed({tgt_lat[31],tgt_lat});
 wire measured_qualified=actual_error<=ACT_BAND && actual_error>=-ACT_BAND &&
  abs_velocity<=4*ACT_VEL_Q8 && abs_theta<=2000 && abs_rate<=60;
 wire signed [31:0] next_lo=actual<still_lo?actual:still_lo;
 wire signed [31:0] next_hi=actual>still_hi?actual:still_hi;
 wire signed [32:0] still_range=$signed({next_hi[31],next_hi})-$signed({next_lo[31],next_lo});
 wire reference_done=(distance==0 && v_q16==0);
 wire base_done=reference_done && measured_qualified && window_settled && still_range<=STILL_SPAN;
 localparam [63:0] WINDOW_VELOCITY_LIMIT=64'd1*ACT_VEL_Q8*ACT_SETTLE;
 wire [63:0] next_velocity_sum=velocity_sum+abs_velocity;
 wire signed [63:0] speed_rounded=(v_next+(v_next<0?64'sd127:64'sd128))>>>8;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   pp_positive<=1;recovering<=0;timed_out<=0;mode_d<=0;tgt_lat<=0;
   p_q16<=0;v_q16<=0;dv_r<=0;v_next<=0;ph<=0;stopping_r<=0;
   dwell<=0;wait_count<=0;calm<=0;inband<=0;still_lo<=0;still_hi<=0;
   velocity_sum<=0;window_settled<=0;
   ref_pos<=0;final_target<=0;speed_q8<=0;busy<=0;arrived<=0;gated<=0;error<=0;
  end else begin
   mode_d<=mode;tgt_lat<=tgt_cnt;final_target<=tgt_lat;stopping_r<=stopping;
   if(!enable)begin
    ph<=0;p_q16<=actual_q16;v_q16<=0;v_next<=0;dv_r<=0;ref_pos<=actual;speed_q8<=0;
    busy<=0;arrived<=0;gated<=0;error<=0;recovering<=0;timed_out<=0;
    dwell<=0;wait_count<=0;calm<=0;inband<=0;still_lo<=actual;still_hi<=actual;pp_positive<=1;
   velocity_sum<=0;window_settled<=0;
   end else if(mode!=mode_d)begin
    // Preserve position/velocity for a smooth replan; discard pending work.
    ph<=0;dwell<=0;wait_count<=0;calm<=0;inband<=0;arrived<=0;
    timed_out<=0;error<=0;recovering<=0;velocity_sum<=0;window_settled<=0;
   end else if(timed_out)begin
    ph<=0;v_q16<=0;v_next<=0;speed_q8<=0;busy<=0;arrived<=0;error<=1;gated<=1;
   end else if(abs_theta>THETA_ABORT)begin
    // Keep the last commanded reference; do not chase the measured axis.
    ph<=0;v_q16<=0;v_next<=0;speed_q8<=0;calm<=0;inband<=0;dwell<=0;
    velocity_sum<=0;window_settled<=0;recovering<=1;error<=1;gated<=1;busy<=0;arrived<=0;
   end else if(recovering)begin
    ph<=0;gated<=1;arrived<=0;
    if(tick)begin
     if(abs_theta<=2000 && abs_rate<=60)begin
      if(calm>=CALM_TICKS-1)begin recovering<=0;error<=0;calm<=0;end
      else calm<=calm+1'b1;
     end else calm<=0;
    end
   end else if(abs_theta>THETA_HOLD)begin
    ph<=0;v_q16<=0;v_next<=0;speed_q8<=0;inband<=0;dwell<=0;gated<=1;arrived<=0;velocity_sum<=0;window_settled<=0;
   end else begin
    gated<=0;error<=0;
    if(tick)ph<=1;else ph<={ph[1:0],1'b0};
    if(tick)begin
     if(reference_done && measured_qualified)begin
      if((inband==0 && !window_settled) || still_range>STILL_SPAN)begin
       still_lo<=actual;still_hi<=actual;inband<=1;velocity_sum<=abs_velocity;window_settled<=0;
      end else if(inband>=ACT_SETTLE-1)begin
       // A quantization pulse need not reset 300 ms of otherwise still data.
       // Persistent motion fails mean-|v| and/or the measured position span.
       window_settled<=(next_velocity_sum<=WINDOW_VELOCITY_LIMIT);
       // Retain the position envelope through dwell, including window boundaries.
       inband<=0;velocity_sum<=0;still_lo<=next_lo;still_hi<=next_hi;
      end else begin
       still_lo<=next_lo;still_hi<=next_hi;inband<=inband+1'b1;velocity_sum<=next_velocity_sum;
      end
     end else begin inband<=0;velocity_sum<=0;window_settled<=0;still_lo<=actual;still_hi<=actual;end
     arrived<=base_done;busy<=!base_done;
     if(mode==3 && reference_done)begin
      if(base_done)begin
       wait_count<=0;
       if(dwell>=DWELL_TICKS-1)begin pp_positive<=~pp_positive;dwell<=0;inband<=0;arrived<=0;velocity_sum<=0;window_settled<=0;end
       else dwell<=dwell+1'b1;
      end else begin
       dwell<=0;
       if(wait_count>=REV_TIMEOUT-1)begin timed_out<=1;error<=1;end
       else wait_count<=wait_count+1'b1;
      end
     end else begin dwell<=0;wait_count<=0;end
    end
    if(ph[0])dv_r<=dv;
    if(ph[1])begin
     if(v_q16<dv_r)v_next<=(v_q16+ACCEL>dv_r)?dv_r:v_q16+ACCEL;
     else if(v_q16>dv_r)v_next<=(v_q16-ACCEL<dv_r)?dv_r:v_q16-ACCEL;
     else v_next<=dv_r;
    end
    if(ph[2])begin
     if(abs_distance<=ACCEL && abs_v<=ACCEL)begin
      p_q16<=goal_q16;v_q16<=0;v_next<=0;ref_pos<=tgt_lat;speed_q8<=0;
     end else begin
      p_q16<=p_q16+v_next;v_q16<=v_next;
      ref_pos<=(p_q16+v_next)>>>16;speed_q8<=speed_rounded[15:0];
     end
    end
   end
  end
 end
endmodule
