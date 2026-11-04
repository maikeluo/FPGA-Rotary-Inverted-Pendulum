`timescale 1ns/1ps
module tb_acg720_r12_swing_phase;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,start=0,stop=0,healthy=1,valid=1;
 reg signed [31:0] angle=179000,rate=0,pos=0;
 wire [2:0] state,oldstate;wire [7:0] fault;wire signed [15:0] cmd,oldcmd;
 balance_control #(.SWING_FILTER_ENABLE(1),.SWING_COHERENT_ENABLE(1),.TRAVEL_LIMIT(780)) d(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(healthy),
 .angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(32'd0),.mode(2'd0),
 .state(state),.fault(fault),.command(cmd),.reset_reference(1'b0),.recover_reference(1'b0),
 .target_ext(32'd0),.target_ext_en(1'b0),.manual_reference(start),.target_velocity_ext(32'd0),.swing_sample_valid(valid));
 balance_control #(.SWING_FILTER_ENABLE(1),.TRAVEL_LIMIT(780)) old(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(healthy),
 .angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(32'd0),.mode(2'd0),
 .state(oldstate),.command(oldcmd),.reset_reference(1'b0),.recover_reference(1'b0),
 .target_ext(32'd0),.target_ext_en(1'b0),.manual_reference(start),.target_velocity_ext(32'd0),.swing_sample_valid(1'b1));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(15)cycle;end endtask
 task begin_run;begin stop=1;cycle;stop=0;valid=1;angle=179000;rate=0;pos=0;start=1;cycle;start=0;end endtask
 integer i,phi,nchanges,oldchanges,last,lastold,lastchange;
 reg [63:0] replay[0:4499];integer j,rejected;
 initial begin
  repeat(3)cycle;rst=1;repeat(3)cycle;begin_run;
  repeat(31)step;if(cmd!=0)$fatal(1,"drove before clean qualification");
  nchanges=0;oldchanges=0;last=0;lastold=0;lastchange=-100;
  for(i=0;i<5000;i=i+1)begin
   phi=$rtoi(6000.0*$sin(6.28318530718*i/1000.0));
   angle=phi>=0?-180000+phi:180000+phi;rate=$rtoi(37.69911184*$cos(6.28318530718*i/1000.0));
   step;
   if(cmd && last && (cmd<0)!=(last<0))begin
    nchanges=nchanges+1;if(i-lastchange<60)$fatal(1,"direction dwell below 60ms");lastchange=i;
   end
   if(oldcmd && lastold && (oldcmd<0)!=(lastold<0))oldchanges=oldchanges+1;
   last=cmd;lastold=oldcmd;
   if(i>150 && !d.coherent_tracking)$fatal(1,"normal turning point reset energy phase");
   if(cmd>400 || cmd< -400 || state!=1)$fatal(1,"limits/state");
  end
  if(nchanges>14 || oldchanges<35)$fatal(1,"bootstrap starvation contrast new=%d old=%d",nchanges,oldchanges);
  $display("small pendulum input (not a plant): new=%0d r9=%0d reversals / 5 seconds",nchanges,oldchanges);
  rate=0;angle=179000;repeat(100)step;if(!d.coherent_tracking)$fatal(1,"short zero-rate interval reset tracking");
  repeat(600)step;if(d.coherent_tracking)$fatal(1,"settled-down restart missing");
  // Current invalid ADC or a derivative spike cancels output immediately.
  valid=0;angle=0;rate=0;cycle;if(cmd!=0)$fatal(1,"bad ADC output cancellation");
  repeat(5)step;if(state!=1)$fatal(1,"bad ADC falsely captured");
  valid=1;repeat(31)step;if(state!=1 || cmd!=0)$fatal(1,"early capture on recovery");
  repeat(5)step;if(state!=2)$fatal(1,"clean upright handoff");
  repeat(1010)step;if(state!=3)$fatal(1,"balance handoff");
  angle=31000;step;if(state!=4 || fault!=8 || cmd!=0)$fatal(1,"fall protection");
  begin_run;repeat(80)step;rate=-9068;step;
  if(cmd!=0 || d.swing_rate_q8!=0 || d.coherent_rejected==0)$fatal(1,"logged spike poisoned pump");
  rate=0;repeat(31)step;if(cmd!=0)$fatal(1,"spike quarantine too short");
  repeat(10)step;if(cmd==0)$fatal(1,"clean down not rearmed");
  stop=1;cycle;if(cmd!=0 || state!=0 || d.coherent_tracking)$fatal(1,"stop clear");stop=0;
  begin_run;for(i=0;i<1000;i=i+1)begin rate=i%2?800:-800;step;end
  if(d.coherent_tracking)$fatal(1,"alternating noise accepted as sustained direction");
  pos=781;step;if(state!=4 || fault!=2 || cmd!=0)$fatal(1,"travel limit");
  // All three recorded SWING segments, each 100Hz value held ten 1ms ticks.
  // This checks software response only, not the new closed-loop trajectory.
  $readmemh("../../tests/acg720_log5_swing.hex",replay);
  for(j=0;j<3;j=j+1)begin
   begin_run;nchanges=0;last=0;lastchange=-100;
   for(i=0;i<15000;i=i+1)begin
    angle=$signed(replay[j*1500+i/10][63:32]);rate=$signed(replay[j*1500+i/10][31:0]);step;
    if(cmd && last && (cmd<0)!=(last<0))begin
     nchanges=nchanges+1;if(i-lastchange<60)$fatal(1,"replay reversal gate");lastchange=i;
    end
    last=cmd;
    if(cmd>400 || cmd< -400)$fatal(1,"replay output cap");
   end
   repeat(3)step;if(state!=4 || fault!=4 || cmd!=0)$fatal(1,"15 second timeout removed");
   $display("log5 held-input run %0d: %0d reversals; no mechanical plant",j+1,nchanges);
  end
  $display("PASS r12 coherent swing: measured motion/turning points, seed rearm, 60ms direction hold, invalid ADC/spike rejection, clean capture, balance/fall, noise, travel, stop, recorded-input replay and 15s timeout");$finish;
 end
 initial begin #35000000;$fatal(1,"test timeout");end
endmodule
