`timescale 1ns/1ps
module tb_v54_trajectory;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,en=0;reg [1:0] mode=0;
 reg signed [31:0] actual=0,origin=0,angle=0,rate=0,velocity=0;
 wire signed [31:0] refpos,goal;wire signed [15:0] speed;
 wire busy,arrived,gated,error;
 traj_gen #(.TARGET_COUNTS(12),.VMAX(1024),.ACCEL(16),.ACT_BAND(2),.ACT_VEL_Q8(2),
 .ACT_SETTLE(5),.DWELL_TICKS(5),.REV_TIMEOUT(20),.CALM_TICKS(10),.STILL_SPAN(1)) dut
 (clk,rst,tick,en,mode,actual,origin,angle,rate,1'b0,1'b0,1'b0,refpos,goal,speed,busy,arrived,gated,error,velocity);
 integer i,saved,changes=0;reg signed [31:0] last_goal;reg signed [63:0] prev_v,prev_p;
 reg check_profile=1;
 task step;begin
  prev_v=dut.v_q16;prev_p=dut.p_q16;
  @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);#1;
  if(^refpos===1'bx || ^speed===1'bx || ^error===1'bx)$fatal(1,"undefined planner state");
  if(check_profile && en && !gated && !error && mode==dut.mode_d)begin
   if(dut.v_q16>1024 || dut.v_q16< -1024)$fatal(1,"velocity limit");
   if(dut.v_q16-prev_v>16 || prev_v-dut.v_q16>16)$fatal(1,"acceleration limit");
  end
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;repeat(3)step;en=1;mode=1;
  repeat(1500)step;
  if(goal!=12 || refpos!=12 || arrived)$fatal(1,"reference vs measured arrival");
  actual=12;repeat(10)step;if(!arrived)$fatal(1,"real settled arrival");
  mode=2;repeat(2500)step;
  if(goal!=-12 || refpos!=-12 || arrived)$fatal(1,"negative goal/sign/measured arrival");
  actual=-12;velocity=3;repeat(20)step;if(arrived)$fatal(1,"moving declared settled");
  velocity=0;repeat(10)step;if(!arrived)$fatal(1,"negative measured settle");
  mode=0;repeat(1500)step;actual=0;repeat(10)step;if(goal!=0 || refpos!=0 || !arrived)$fatal(1,"basic home hold");
  mode=1;repeat(100)step;saved=refpos;angle=5000;check_profile=0;repeat(10)step;
  if(refpos!=saved || !gated)$fatal(1,"tilt hold");
  angle=9000;step;if(!error || !gated)$fatal(1,"abort");
  angle=0;repeat(9)step;if(!error)$fatal(1,"recovery counted clocks instead of ticks");
  step;step;if(error)$fatal(1,"timed recovery");
  repeat(1500)step;
  mode=3;actual=0;repeat(1600)step;
  if(goal!=12 || !error || !dut.timed_out)$fatal(1,"timeout forced reverse");
  repeat(100)step;if(goal!=12)$fatal(1,"latched timeout direction changed");
  mode=0;repeat(1500)step;if(error)$fatal(1,"mode reset timeout");
  en=0;actual=-500;origin=-500;repeat(5)step;en=1;mode=1;
  repeat(1500)begin actual=refpos;step;end
  if(refpos!=-488 || goal!=-488)$fatal(1,"negative origin sign");
  en=0;actual=17;step;if(refpos!=17 || speed!=0 || arrived || error)$fatal(1,"stop reset");
  $display("PASS v54 trajectory: signed origins, bounded profile, actual position/velocity arrival, basic hold, angle gate, tick-based recovery, timeout pause instead of reversal, stop");$finish;
 end
 initial begin #6000000;$fatal(1,"timeout");end
endmodule
