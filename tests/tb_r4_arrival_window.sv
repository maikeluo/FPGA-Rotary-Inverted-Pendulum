`timescale 1ns/1ps
module tb_r4_arrival_window;
 reg clk=0;always #10 clk=~clk;reg rst=0,tick=0,en=0;reg [1:0] mode=1;
 reg signed [31:0] actual=0,velocity=0;wire signed [31:0] refpos,goal;
 wire arrived,error;integer i,positive=0,negative=0;
 traj_gen #(.TARGET_COUNTS(12),.VMAX(1024),.ACCEL(16),.ACT_BAND(20),.ACT_VEL_Q8(2),
 .ACT_SETTLE(5),.DWELL_TICKS(7),.REV_TIMEOUT(40),.STILL_SPAN(1)) dut
 (.clk(clk),.rst_n(rst),.tick(tick),.enable(en),.mode(mode),.actual(actual),.origin(32'd0),
 .angle(32'd0),.angle_delta(32'd0),.step_plus(1'b0),.step_minus(1'b0),.go_home(1'b0),
 .ref_pos(refpos),.final_target(goal),.arrived(arrived),.error(error),.velocity_q8(velocity));
 task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);#1;end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;step;en=1;repeat(1500)step;actual=12;
  // One quantization pulse per window: old continuous-speed gate never arrived.
  for(i=0;i<20;i=i+1)begin velocity=(i%5==0)?3:0;step;end
  if(!arrived)$fatal(1,"isolated encoder speed pulses reset settling");
  velocity=3;repeat(20)step;if(arrived)$fatal(1,"persistent measured speed accepted");
  velocity=0;
  for(i=0;i<15;i=i+1)begin actual=10+(i%4);step;end
  if(arrived)$fatal(1,"position span ignored");
  en=0;actual=0;step;mode=3;en=1;i=0;
  while(!(goal==12 && dut.dwell==6) && i<3000)begin actual=refpos;velocity=0;step;i=i+1;end
  if(i>=3000)$fatal(1,"settled dwell setup");
  actual=14;velocity=0;step;
  if(goal!=12 || dut.dwell!=0 || arrived)$fatal(1,"position jump at last dwell tick forced reversal");
  for(i=0;i<4000;i=i+1)begin
   actual=refpos;velocity=(dut.reference_done && i%5==0)?3:0;step;
   if(goal==12)positive=1;if(positive && goal==-12)negative=1;
  end
  if(!positive || !negative || error)$fatal(1,"noise-tolerant actual dwell never reversed");
  $display("PASS r4 arrival windows: isolated quantization pulses accepted, persistent speed/span rejected, last-dwell-tick movement prevents reversal, actual settled dwell enables positive/negative shuttle without forced timeout reversal");$finish;
 end
 initial begin #4000000;$fatal(1,"timeout");end
endmodule
