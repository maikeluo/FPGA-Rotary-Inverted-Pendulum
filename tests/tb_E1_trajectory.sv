`timescale 1ns/1ps
module tb_E1_trajectory;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,en=0;reg [1:0] mode=0;
 reg signed [31:0] actual=0,angle=0,rate=0,vel=0;
 wire signed [31:0] refpos,goal;wire signed [15:0] speed;
 wire busy,arrived,gated,error;
 traj_gen #(.TARGET_COUNTS(40)) dut(clk,rst,tick,en,mode,actual,32'sd0,angle,rate,1'b0,1'b0,1'b0,refpos,goal,speed,busy,arrived,gated,error,vel);
 integer i,saved;
 task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);actual=refpos;end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;repeat(3)step;en=1;
  mode=1;repeat(3000)step;if(goal!=40 || refpos!=40 || !arrived)$fatal(1,"positive endpoint");
  mode=2;repeat(5000)step;if(goal!=-40 || refpos!=-40 || !arrived)$fatal(1,"negative endpoint");
  mode=0;repeat(3000)step;if(refpos!=0 || goal!=0)$fatal(1,"home endpoint");
  mode=1;repeat(100)step;saved=refpos;angle=5000;repeat(50)step;
  if(refpos!=saved || !gated)$fatal(1,"tilt gate");
  angle=9000;repeat(5)step;if(!error)$fatal(1,"tilt abort");
  angle=0;repeat(3000)step;if(error || refpos!=40)$fatal(1,"recovery");
  en=0;actual=-17;repeat(3)step;if(refpos!=-17 || speed!=0)$fatal(1,"disabled follow");
  $display("PASS E1 retained trajectory: positive/negative/home references, angle gate, abort/recovery and disabled follow; reference tests only");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
