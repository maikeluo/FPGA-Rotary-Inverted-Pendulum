`timescale 1ns/1ps
module tb_E1_velocity_filter;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,en=0,tick=0;reg signed [31:0] vel=0;
 wire signed [31:0] trim,mirror_trim;
 wire signed [31:0] nvel=-vel;
 base_trim_filter #(.TP(-12),.TV(-64),.TV_BAL(-64),.POSITION_DEADBAND(40),.VEL_FILTER_SHIFT(5)) dut
 (clk,rst,en,tick,32'sd0,32'sd0,vel,12'd2048,trim,1'b1);
 base_trim_filter #(.TP(-12),.TV(-64),.TV_BAL(-64),.POSITION_DEADBAND(40),.VEL_FILTER_SHIFT(5)) mirror
 (clk,rst,en,tick,32'sd0,32'sd0,nvel,12'd2048,mirror_trim,1'b1);
 reg signed [63:0] model=0,err,delta,expected;integer count=0,i,j,seed=2610;
 reg symmetry=1;
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin
  err=64'sd1*vel*256-model;
  delta=err>=0?(err+16)/32:-((-err+16)/32);
  model=model+delta;
  expected=model>=0?(model+128)/256:-((-model+128)/256);
  tick=1;cycle;tick=0;repeat(12)cycle;
  if(dut.velocity_accum_q8!==model[40:0] || dut.velocity_f!==expected[31:0])
   $fatal(1,"independent fractional model sample=%0d v=%0d expected=%0d actual=%0d",count,vel,expected,dut.velocity_f);
  if(^trim===1'bx || trim>4000 || trim< -4000)$fatal(1,"unknown/out-of-range trim");
  if(symmetry && (dut.velocity_f!==-mirror.velocity_f || trim!==-mirror_trim))$fatal(1,"directional asymmetry");
  count=count+1;
 end endtask
 task restart;begin en=0;cycle;model=0;vel=0;en=1;cycle;end endtask
 initial begin
  repeat(3)cycle;rst=1;en=1;repeat(500)step;
  if(dut.velocity_f!==0 || trim!==0)$fatal(1,"power-on zero");
  vel=256;repeat(32)step;
  if(dut.velocity_f<155 || dut.velocity_f>170)$fatal(1,"32ms time constant");
  repeat(400)step;if(dut.velocity_f!=256)$fatal(1,"positive DC gain");
  vel=0;repeat(600)step;if(dut.velocity_f!==0 || trim!==0)$fatal(1,"positive coast residual");
  vel=-20;repeat(500)step;vel=0;repeat(600)step;
  if(dut.velocity_f!==0 || trim!==0)$fatal(1,"negative coast residual (old bug)");
  vel=1;repeat(500)step;if(dut.velocity_f!=1)$fatal(1,"small real speed suppressed");
  vel=-1;repeat(500)step;if(dut.velocity_f!=-1)$fatal(1,"small negative speed suppressed");
  for(i=0;i<10000;i=i+1)begin vel=$signed($random(seed))%4097;step;end
  symmetry=0;vel=32'sh7fffffff;repeat(800)step;vel=32'sh80000000;repeat(800)step;
  vel=0;repeat(1600)step;if(dut.velocity_f!==0)$fatal(1,"full-range convergence");
  for(j=0;j<=7;j=j+1)begin
   restart;vel=256;repeat(100)step;
   tick=1;cycle;tick=0;repeat(j)cycle;en=0;cycle;model=0;
   if(dut.velocity_f!==0 || dut.velocity_accum_q8!==0 || trim!==0)$fatal(1,"STOP during phase=%0d",j);
   vel=0;en=1;repeat(20)cycle;repeat(20)step;if(trim!==0)$fatal(1,"stale restart output");
  end
  rst=0;cycle;if(dut.velocity_f!==0 || trim!==0)$fatal(1,"asynchronous reset");
  $display("PASS E1 velocity filter: %0d independent Q8 model samples; no-X reset, symmetric directions, 32ms response, small DC speeds, full-range arithmetic, zero residue, all-stage STOP and restart",count);$finish;
 end
 initial begin #30000000;$fatal(1,"timeout");end
endmodule
