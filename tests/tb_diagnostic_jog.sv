`timescale 1ns/1ps
module tb_diagnostic_jog;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,healthy=1;
 reg [1:0] keys=1,raw=1;
 reg signed [31:0] pos=0,vel=0;
 wire en;wire signed [15:0] cmd;wire signed [31:0] origin;
 wire [15:0] elapsed;wire [7:0] fault,status;wire launch_pulse;
 diagnostic_jog d(clk,rst,tick,keys,raw,healthy,pos,vel,en,cmd,origin,elapsed,fault,status,launch_pulse);
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(3)cycle;end endtask
 task release_all;begin raw=0;keys=0;cycle;repeat(21)step;end endtask
 task launch(input [1:0] key);begin release_all;keys=key;raw=key;step;if(!en)$fatal(1,"launch failed");end endtask
 task raw_stop;begin raw=0;#1;if(en || cmd!=0)$fatal(1,"release waits for tick/debounce");cycle;keys=0;end endtask
 integer i;
 initial begin
  repeat(3)cycle;rst=1;repeat(600)step;
  if(en || cmd || status[6])$fatal(1,"boot-held key started motor");
  raw=0;keys=0;repeat(19)step;if(status[6])$fatal(1,"early release rearm");
  step;if(!status[6])$fatal(1,"20ms rearm missing");
  raw=1;keys=1;step;if(cmd!=80 || origin!=0)$fatal(1,"start duty/origin");
  for(i=1;i<=40;i=i+1)begin step;if(cmd!=80+3*i)$fatal(1,"ramp step %d cmd=%d",i,cmd);end
  repeat(40)step;if(cmd!=200)$fatal(1,"duty cap");
  pos=4;cycle;if(!status[4])$fatal(1,"motion not recorded");
  while(elapsed<499)step;
  if(!en)$fatal(1,"time cap too early");step;
  if(en || cmd || !status[3] || fault)$fatal(1,"500ms cap");
  repeat(100)step;if(en)$fatal(1,"held key restarted");
  raw=2;keys=2;repeat(40)step;if(en)$fatal(1,"direct reverse bypassed release");
  raw=3;keys=3;repeat(40)step;if(en)$fatal(1,"both keys enabled output");
  raw=0;keys=0;repeat(19)step;raw=2;keys=2;step;if(en)$fatal(1,"short release rearmed");
  launch(2);if(cmd!=-80)$fatal(1,"negative launch");repeat(40)step;if(cmd!=-200)$fatal(1,"negative ramp");
  raw_stop;raw=2;keys=2;repeat(30)step;if(en)$fatal(1,"release bounce restarts output");
  pos=100;launch(1);
  for(i=0;i<200;i=i+1)begin pos=100+(i%2);step;end
  if(en || fault!=8'h20 || !status[2] || status[4])$fatal(1,"one-count chatter counted as movement");
  repeat(100)step;if(en)$fatal(1,"no-motion stop restarted");
  pos=-100;launch(2);pos=-163;cycle;if(!en)$fatal(1,"travel stops below64");
  pos=-164;#1;if(en)$fatal(1,"travel waits for tick");cycle;
  if(fault!=2 || !status[0])$fatal(1,"negative travel stop");
  pos=100;launch(1);pos=164;cycle;if(en || fault!=2 || !status[0])$fatal(1,"positive travel stop");
  pos=0;launch(1);vel=512;#1;if(en)$fatal(1,"speed gate");cycle;if(fault!=2 || !status[1])$fatal(1,"positive speed");
  vel=0;launch(2);vel=-512;cycle;if(en || fault!=2)$fatal(1,"negative speed");vel=0;
  launch(1);healthy=0;#1;if(en)$fatal(1,"health immediate gate");cycle;if(fault!=1)$fatal(1,"health fault");
  healthy=1;repeat(50)step;if(en)$fatal(1,"health recovery while held restarts");
  launch(1);raw=3;#1;if(en)$fatal(1,"raw both-key stop");cycle;keys=3;
  raw=1;keys=1;repeat(25)step;if(en)$fatal(1,"release only one key restarts");
  raw_stop;pos=32'sh80000000;launch(1);pos=32'sh7fffffff;cycle;
  if(en || fault!=2)$fatal(1,"wide signed displacement wrapped");
  pos=0;launch(2);rst=0;#1;if(en || cmd)$fatal(1,"reset immediate gate");cycle;rst=1;repeat(25)step;
  if(en)$fatal(1,"reset with held key restarts");
  $display("PASS tb_diagnostic_jog: ramp +/-8..20%%, immediate release/reset/health, 500ms/64-count/speed limits, 200ms no-motion including 1-count chatter, boot-held/direct-reverse/both-key rearm, signed-wide offset");$finish;
 end
 initial begin #1000000;$fatal(1,"jog unit timeout");end
endmodule
