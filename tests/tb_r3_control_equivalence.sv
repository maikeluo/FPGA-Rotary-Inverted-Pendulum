`timescale 1ns/1ps
`include "baseline_r2.vh"
`timescale 1ns/1ps
module tb_r3_control_equivalence;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,start=0,stop=0;
 reg signed [31:0] angle=0,rate=0,pos=100,vel=0,goal=100;
 wire [2:0] state,newstate;wire [7:0] fault,newfault;
 wire signed [15:0] command,newcommand;wire [63:0] debug,newdebug;
 balance_control_uploaded #(.TRAVEL_LIMIT(780),.RETURN_DAMP_GAIN(5),.RETURN_LIMIT(1500),.RETURN_DEADBAND(12),.RETURN_SETTLE_TICKS(500),
 .TRIM_VELOCITY(-64),.BASE_VELOCITY_SPLIT_ENABLE(0),.TRIM_POSITION(-12),.POSITION_DEADBAND(8),.OUTER_SLEW_MDEG(8),.OUTER_FAST_SLEW_MDEG(32),.VEL_FILTER_SHIFT(5)) old(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(1'b1),.angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(vel),.mode(2'b00),
 .state(state),.fault(fault),.command(command),.active(),.target(),.reference_q16(),.reset_reference(1'b0),.recover_reference(1'b0),.travel_origin(),.capture_ticks(),.limit_mask(),.control_debug(debug),.target_ext(goal),.target_ext_en(1'b1));
 balance_control #(.SWING_FILTER_ENABLE(1),.SWING_COHERENT_ENABLE(1),.MOTION_FEEDBACK_ENABLE(1),.TRAVEL_LIMIT(780),.RETURN_DAMP_GAIN(5),.RETURN_LIMIT(1500),.RETURN_DEADBAND(12),.RETURN_SETTLE_TICKS(500),
 .TRIM_VELOCITY(-64),.BASE_VELOCITY_SPLIT_ENABLE(0),.TRIM_POSITION(-12),.POSITION_DEADBAND(8),.OUTER_SLEW_MDEG(8),.OUTER_FAST_SLEW_MDEG(32),.VEL_FILTER_SHIFT(5)) dut(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(1'b1),.angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(vel),.mode(2'b00),
 .state(newstate),.fault(newfault),.command(newcommand),.active(),.target(),.reference_q16(),.reset_reference(1'b0),.recover_reference(1'b0),.travel_origin(),.capture_ticks(),.limit_mask(),.control_debug(newdebug),.target_ext(goal),.target_ext_en(1'b1),.manual_reference(start),.target_velocity_ext(32'd0),.swing_sample_valid(1'b1));
 integer i,seed=5428,count=0;
 task cycle;begin @(negedge clk);#1;
  if(rst && (command!==newcommand || debug!==newdebug || state!==newstate || fault!==newfault))
   $fatal(1,"uploaded controller equivalence failed sample=%0d commands=%0d/%0d",count,command,newcommand);
 end endtask
 task step;begin tick=1;cycle;tick=0;repeat(14)cycle;count=count+1;end endtask
 initial begin
  repeat(3)cycle;rst=1;repeat(3)cycle;start=1;cycle;start=0;repeat(1030)step;
  if(newstate!=3)$fatal(1,"balance setup");
  for(i=0;i<7000;i=i+1)begin
   angle=$signed($random(seed))%5001;rate=$signed($random(seed))%180;
   pos=100+$signed($random(seed))%180;vel=$signed($random(seed))%128;
   if(i%500==0)goal=100+$signed($random(seed))%120;
   step;
  end
  stop=1;cycle;stop=0;angle=0;rate=0;vel=0;pos=150;goal=150;start=1;cycle;start=0;repeat(1030)step;
  angle=31000;step;if(newstate!=4 || newfault!=8)$fatal(1,"original fall cap");
  $display("PASS r3 control equivalence: %0d samples, uploaded r54r2 inner/outer gains, command and all debug fields bit-identical each clock during capture, balanced inputs, manual restart and fall protection; no plant model",count);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
