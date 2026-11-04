`timescale 1ns/1ps
module tb_r9_io_observer;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,a=1,b=1,launch=0,en=0,pwm=1,in1=0,in2=0;
 reg [3:0] key_n=15,keys=0;
 wire [63:0] debug;wire [31:0] counts;wire [15:0] pulses;wire [7:0] seen;
 io_observer d(clk,rst,key_n,keys,a,b,launch,en,pwm,in1,in2,debug,counts,pulses,seen);
 task cycles(input integer n);repeat(n)@(negedge clk);endtask
 integer i;
 initial begin
  cycles(3);rst=1;cycles(30);
  if(debug[63:16] || counts || pulses || seen || debug[9:8]!=3)$fatal(1,"idle phase/coast counted as activity");
  // Short raw input is remembered even though the main debounce never accepts it.
  key_n[0]=0;cycles(6);key_n=15;cycles(10);
  if(seen!=1 || counts || debug[31:16])$fatal(1,"short input trace");
  key_n[0]=0;keys=1;cycles(10);launch=1;en=1;in1=1;cycles(1);launch=0;
  if(counts!=1 || debug[31:16]!=1 || seen!=8'h11 || pulses!=1 || debug[15:14]!=1)$fatal(1,"accepted launch trace");
  for(i=0;i<5;i=i+1)begin pwm=0;cycles(1);pwm=1;cycles(1);end
  if(pulses!=6)$fatal(1,"physical drive pulse count");
  a=0;cycles(6);b=0;cycles(6);a=1;cycles(6);b=1;cycles(6);
  if(debug[47:32]!=2 || debug[63:48]!=2)$fatal(1,"A/B edge counts");
  key_n=15;keys=0;en=0;in1=0;cycles(100);
  if(pulses!=6 || counts!=1 || debug[31:16]!=1 || seen!=8'h11)$fatal(1,"release erased evidence or idle PWM inflated pulses");
  key_n[1]=0;keys=2;cycles(10);launch=1;cycles(1);launch=0;cycles(1);
  if(pulses || debug[31:16]!=2 || counts!=32'h00010001 || debug[15:14]!=2)$fatal(1,"second trial did not reset only trial PWM");
  // Saturation never wraps old evidence back to zero.
  d.a_edges=16'hffff;a=0;cycles(10);if(debug[47:32]!=65535)$fatal(1,"A count wrapped");
  rst=0;cycles(2);if(debug[63:14] || counts || pulses || seen)$fatal(1,"reset evidence");
  $display("PASS r9 passive IO trace: no boot/coast artefacts, raw-only taps, debounced press/launch counts, active PWM pulses, real A/B transitions, retained release evidence, per-jog reset and saturation");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
