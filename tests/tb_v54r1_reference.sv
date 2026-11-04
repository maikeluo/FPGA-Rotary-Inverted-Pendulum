`timescale 1ns/1ps
module tb_v54r1_reference;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,en=0,tick=0,bal=0;reg signed [31:0] angle=0,request=0;
 wire signed [31:0] reference,mirror;
 outer_reference_slew #(.NORMAL_STEP(8),.FAST_STEP(32)) dut(clk,rst,tick,en,bal,angle,request,reference);
 outer_reference_slew #(.NORMAL_STEP(8),.FAST_STEP(32)) other(clk,rst,tick,en,bal,-angle,-request,mirror);
 reg signed [63:0] model=0,diff,lim;integer i,seed=2710,count=0;
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin
  diff=64'sd1*request-model;lim=(angle>=3000 || angle<=-3000)?32:8;
  if(!en)model=0;else if(!bal)model=request;
  else if(diff>lim)model=model+lim;else if(diff< -lim)model=model-lim;else model=request;
  tick=1;cycle;tick=0;repeat(10)cycle;
  if(reference!==model[31:0])$fatal(1,"independent slew expected=%0d got=%0d",model,reference);
  if(request!=32'sh80000000 && angle!=32'sh80000000 && reference!==-mirror)$fatal(1,"asymmetric slew");
  count=count+1;
 end endtask
 initial begin
  repeat(3)cycle;rst=1;en=1;request=701;cycle;model=701;
  if(reference!=701)$fatal(1,"capture transparent");bal=1;step;
  for(i=0;i<4000;i=i+1)begin request=$signed($random(seed))%4001;angle=$signed($random(seed))%5001;step;end
  request=1234;repeat(5)cycle;if(reference!==model[31:0])$fatal(1,"advanced without control tick");
  request=32'sh7fffffff;angle=3000;step;
  request=32'sh80000000;angle=32'sh80000000;step;
  en=0;cycle;model=0;if(reference!==0)$fatal(1,"stop did not clear");
  en=1;request=0;angle=0;repeat(20)step;if(reference!=0)$fatal(1,"stale restart");
  bal=0;request=-999;cycle;model=-999;bal=1;step;
  if(reference!=-999)$fatal(1,"capture to balance discontinuity");
  rst=0;cycle;if(reference!==0)$fatal(1,"reset");
  $display("PASS v54r1 reference: %0d independent slew samples, +/- symmetry, 8/32 mdeg limits, tick-only update, signed extremes, transparent capture, bumpless balance, immediate stop/reset",count);$finish;
 end
 initial begin #5000000;$fatal(1,"timeout");end
endmodule
