`timescale 1ns/1ps
module tb_recovery_session;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,en=1,ms=0,go=0,healthy=1;
 reg [2:0] state=0;reg [7:0] fault=0;
 reg signed [31:0] angle=179000,rate=0;
 wire astop,astart,abandon,armed;wire [3:0] retries;
 integer launches=0,stops=0,i,j,elapsed,prev_launch,prev_stop;
 auto_recover #(.HANG_TICKS(3),.BASE_WAIT(2),.WAIT_STEP(2),.MAX_WAIT(6),.IDLE_TICKS(2),.MAX_RETRY(10),.GOOD_TICKS(4)) dut
 (clk,rst,tick,en,ms,go,healthy,state,fault,angle,rate,astop,astart,retries,abandon,armed,1'b0);
 always @(negedge clk)begin
  if(astop)stops=stops+1;
  if(astart)launches=launches+1;
  if(ms && (astop || astart))$fatal(1,"manual stop priority");
 end
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(8)cycle;end endtask
 task reset;begin rst=0;ms=0;go=0;en=1;healthy=1;state=0;fault=0;angle=179000;rate=0;repeat(3)cycle;rst=1;cycle;end endtask
 task user_start;begin state=0;fault=0;go=1;cycle;go=0;state=1;repeat(3)cycle;end endtask
 task fall;begin state=4;fault=8;repeat(3)cycle;end endtask
 task recover_one;begin
  prev_stop=stops;elapsed=0;
  while(stops==prev_stop && elapsed<30)begin step;elapsed=elapsed+1;end
  if(stops!=prev_stop+1)$fatal(1,"no recovery stop");
  if(elapsed != 3+(i==0?2:(i==1?4:6)))$fatal(1,"backoff count %0d i=%0d",elapsed,i);
  state=0;fault=0;prev_launch=launches;step;step;
  if(launches!=prev_launch+1 || retries!=i+1 || armed)$fatal(1,"launch counted incorrectly");
  repeat(3)step;if(launches!=prev_launch+1)$fatal(1,"repeated pulse in IDLE");
  state=1;cycle;
 end endtask
 initial begin
  reset;state=4;fault=8;repeat(20)step;if(stops || launches)$fatal(1,"power-up unarmed restart");
  user_start;
  // Fault changes BETWEEN tick pulses: old r15c lost every such event.
  for(i=0;i<10;i=i+1)begin fall;recover_one;end
  fall;prev_stop=stops;repeat(30)step;
  if(!abandon || retries!=10 || stops!=prev_stop)$fatal(1,"ten retry cap");
  // Explicit manual restart clears abandonment and old budget.
  user_start;if(abandon || retries!=0)$fatal(1,"manual rearm");
  fall;healthy=0;prev_stop=stops;repeat(20)step;if(stops!=prev_stop)$fatal(1,"invalid sample recovery");
  healthy=1;repeat(4)step;healthy=0;step;healthy=1;repeat(4)step;
  if(stops!=prev_stop)$fatal(1,"nonconsecutive healthy qualification");
  step;if(stops!=prev_stop+1)$fatal(1,"healthy settling restart");
  state=0;fault=0;ms=1;step;ms=0;prev_launch=launches;repeat(20)step;
  if(launches!=prev_launch || armed)$fatal(1,"stop during armed IDLE");
  // Every non-retryable error remains latched, including 0x02.
  for(j=0;j<3;j=j+1)begin
   user_start;state=4;fault=(j==0)?2:((j==1)?1:16);prev_stop=stops;repeat(20)step;
   if(stops!=prev_stop || !abandon)$fatal(1,"hardware fault retried");
  end
  user_start;fall;ms=1;repeat(10)step;ms=0;prev_stop=stops;repeat(20)step;
  if(stops!=prev_stop)$fatal(1,"stop did not disarm FAULT");
  user_start;i=0;fall;recover_one;state=3;angle=0;rate=0;repeat(4)step;
  if(retries || abandon || dut.wait_need!=2)$fatal(1,"qualified success reset");
  angle=179000;fall;en=0;prev_stop=stops;repeat(20)step;en=1;repeat(20)step;
  if(stops!=prev_stop)$fatal(1,"disable did not disarm");
  $display("PASS r15d recovery: asynchronous fault arrival, ten launch cap, bounded backoff, single pulses, fresh-sample settling, manual stop/rearm, excluded faults, qualified success and disable");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
