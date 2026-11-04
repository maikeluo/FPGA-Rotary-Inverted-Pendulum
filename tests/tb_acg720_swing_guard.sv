`timescale 1ns/1ps
module tb_acg720_swing_guard;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,start=0,stop=0,healthy=1;
 reg signed [31:0] angle=170000,rate=0,pos=0;
 wire [2:0] state,oldstate;wire [7:0] fault;wire signed [15:0] cmd,oldcmd;
 balance_control #(.SWING_FILTER_ENABLE(1),.TRAVEL_LIMIT(780),.SWING_TIMEOUT(15000)) d(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(healthy),
 .angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(32'sd0),.mode(2'd0),
 .state(state),.fault(fault),.command(cmd),.active(),.target(),.reference_q16(),
 .reset_reference(1'b0),.recover_reference(1'b0),.travel_origin(),.capture_ticks(),.limit_mask(),
 .control_debug(),.target_ext(32'sd0),.target_ext_en(1'b0),.manual_reference(start),.target_velocity_ext(32'sd0));
 balance_control #(.SWING_FILTER_ENABLE(0),.TRAVEL_LIMIT(780),.SWING_TIMEOUT(15000)) old(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(healthy),
 .angle(angle),.angle_delta(rate),.position(pos),.velocity_q8(32'sd0),.mode(2'd0),
 .state(oldstate),.fault(),.command(oldcmd),.active(),.target(),.reference_q16(),
 .reset_reference(1'b0),.recover_reference(1'b0),.travel_origin(),.capture_ticks(),.limit_mask(),
 .control_debug(),.target_ext(32'sd0),.target_ext_en(1'b0),.manual_reference(start),.target_velocity_ext(32'sd0));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(15)cycle;end endtask
 task begin_run;begin stop=1;cycle;stop=0;angle=170000;rate=0;pos=0;start=1;cycle;start=0;end endtask
 integer i,j,nchanges,ochanges,lastsign,lastold,lastchange,elapsed;
 reg [63:0] replay[0:1499];
 initial begin
  repeat(3)cycle;rst=1;repeat(3)cycle;begin_run;
  nchanges=0;ochanges=0;lastsign=0;lastold=0;lastchange=-100;
  for(i=0;i<1000;i=i+1)begin
   rate=(i%2)?800:-800;step;
   if(cmd!=0 && lastsign!=0 && (cmd<0)!=(lastsign<0))begin
    nchanges=nchanges+1;if(i-lastchange<20)$fatal(1,"direction dwell violated");lastchange=i;
   end
   if(oldcmd!=0 && lastold!=0 && (oldcmd<0)!=(lastold<0))ochanges=ochanges+1;
   lastsign=cmd;lastold=oldcmd;
   if(cmd>400 || cmd< -400 || state!=1)$fatal(1,"output/state bounds");
  end
  if(nchanges>10 || ochanges<900)$fatal(1,"alternating-noise rejection %d/%d",nchanges,ochanges);
  $display("noise at 1kHz: new=%0d old=%0d direction changes",nchanges,ochanges);
  rate=500;repeat(100)step;if(cmd<=0)$fatal(1,"positive sustained motion ignored");
  rate=-500;repeat(100)step;if(cmd>=0)$fatal(1,"negative sustained motion ignored");
  stop=1;cycle;if(cmd!=0 || state!=0 || d.pump_direction_valid)$fatal(1,"stop must clear pump");
  stop=0;cycle;if(d.swing_rate_q8!=0)$fatal(1,"stop must clear filter");
  begin_run;repeat(3)step;pos=781;step;if(state!=4 || fault!=2 || cmd!=0)$fatal(1,"travel protection");
  begin_run;healthy=0;cycle;if(state!=4 || fault!=1 || cmd!=0)$fatal(1,"health protection");healthy=1;
  begin_run;angle=0;rate=0;repeat(1050)step;if(state!=3)$fatal(1,"capture/balance handoff");
  angle=31000;step;if(state!=4 || fault!=8)$fatal(1,"fall protection");
  // CSV is 100Hz. Zero-order hold for 10 control ticks is an offline input
  // experiment, not reconstruction of the missing 1kHz samples or a plant.
  $readmemh("../../tests/acg720_swing_replay.hex",replay);
  begin_run;nchanges=0;ochanges=0;lastsign=0;lastold=0;lastchange=-100;elapsed=0;
  for(i=0;i<1500;i=i+1)begin
   angle=$signed(replay[i][63:32]);rate=$signed(replay[i][31:0]);
   for(j=0;j<10;j=j+1)begin
    step;elapsed=elapsed+1;
    if(cmd!=0 && lastsign!=0 && (cmd<0)!=(lastsign<0))begin
     nchanges=nchanges+1;if(elapsed-lastchange<20)$fatal(1,"replay dwell violated");lastchange=elapsed;
    end
    if(oldcmd!=0 && lastold!=0 && (oldcmd<0)!=(lastold<0))ochanges=ochanges+1;
    lastsign=cmd;lastold=oldcmd;
   end
  end
  repeat(2)step;if(state!=4 || fault!=4 || cmd!=0)$fatal(1,"timeout retained");
  if(nchanges>=ochanges)$fatal(1,"replay did not reduce reversals");
  $display("100Hz CSV held 10 ticks: new=%0d old=%0d direction changes; open loop only",nchanges,ochanges);
  $display("PASS tb_acg720_swing_guard: noise, dwell, sustained reversals, stop/restart, health/travel/fall/timeout, capture/balance, recorded-input replay; no mechanical plant");$finish;
 end
 initial begin #20000000;$fatal(1,"test timeout");end
endmodule
