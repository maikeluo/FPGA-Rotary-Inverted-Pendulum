`timescale 1ns/1ps
module tb_r3_pwm40k;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,en=0;reg signed [15:0] command=74;
 wire pwm,in1,in2;
 motor_pwm #(.PERIOD(1250),.DEAD_CYCLES(250)) dut(clk,rst,en,command,pwm,in1,in2);
 integer i,high_ticks,neutral_ticks;
 task cycle;begin @(negedge clk);#1;end endtask
 initial begin
  repeat(3)cycle;rst=1;en=1;repeat(2500)cycle;
  while(dut.phase!=0)cycle;high_ticks=0;
  for(i=0;i<1250;i=i+1)begin if(pwm)high_ticks=high_ticks+1;if(!in1 || in2)$fatal(1,"positive direction");cycle;end
  if(dut.phase!=0 || high_ticks!=92)$fatal(1,"40kHz period/duty quantization");
  command=-74;cycle;neutral_ticks=0;
  while(!in2 && neutral_ticks<1600)begin
   if(in1 || !pwm)$fatal(1,"non-neutral during reverse wait");neutral_ticks=neutral_ticks+1;cycle;
  end
  if(!in2 || neutral_ticks<250 || neutral_ticks>1501)$fatal(1,"250-cycle reversal neutral");
  while(dut.phase!=0)cycle;high_ticks=0;
  for(i=0;i<1250;i=i+1)begin if(pwm)high_ticks=high_ticks+1;cycle;end
  if(high_ticks!=92)$fatal(1,"negative duty");
  en=0;#1;if(!pwm || in1 || in2)$fatal(1,"immediate stop");
  cycle;en=1;command=0;repeat(3)cycle;if(!pwm || in1 || in2)$fatal(1,"zero command neutral");
  $display("PASS r3 PWM: actual 50MHz/1250 = 40kHz, 74 permille gives 92 high clocks, both directions, >=250 neutral clocks on reversal, immediate high-Z stop");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
