`timescale 1ns/1ps
module tb_v54_damping;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,en=0,tick=0;reg signed [31:0] vel=0,probe=0;
 wire signed [31:0] vf,trim;
 base_return_damping #(.GAIN(5),.LIMIT(500)) dut(clk,rst,en,tick,vel,vf,trim);
 reg signed [63:0] expected;integer i,seed=54;
 task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(10)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;
  force dut.filtered_velocity_q8=probe;
  for(i=0;i<2000;i=i+1)begin
   probe=$random(seed);#1;
   expected=probe>4?-64'sd5*(64'sd1*probe-4):(probe< -4?-64'sd5*(64'sd1*probe+4):64'sd0);
   if(dut.damping_scaled!==expected)$fatal(1,"gain-five arithmetic");
  end
  probe=32'sh80000000;#1;expected=-64'sd5*(-64'sd2147483648+4);
  if(dut.damping_scaled!==expected)$fatal(1,"signed lower extreme");
  release dut.filtered_velocity_q8;
  rst=0;repeat(3)@(negedge clk);rst=1;en=1;vel=20;repeat(3000)step;
  if(trim!==-32'sd80)$fatal(1,"positive DC damper");
  vel=-20;repeat(4000)step;if(trim!==32'sd80)$fatal(1,"negative DC damper");
  en=0;@(negedge clk);#1;if(trim!==0)$fatal(1,"disabled output");
  $display("PASS v54 gain-five damper: 2001 signed coefficient vectors, exact shift/add arithmetic, +/- DC response and disable");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
