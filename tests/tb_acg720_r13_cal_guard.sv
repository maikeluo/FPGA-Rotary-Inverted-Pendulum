`timescale 1ns/1ps
module tb_acg720_r13_cal_guard;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,idle=1,stop=0,up=0,down=0,held=0,adc_ok=1,cal=0,busy=0,still=1,start=0;
 reg [9:0] raw=324;reg signed [31:0] angle=-180000,rate=0;
 wire pulse,pending,release_key,allowed,blocked,ready;
 acg720_cal_guard d(clk,rst,tick,idle,stop,up,down,held,adc_ok,cal,busy,still,raw,angle,rate,
   start,pulse,pending,release_key,allowed,blocked,ready);
 integer events=0,i;
 always @(posedge clk)if(pulse)events=events+1;
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;cycle;end endtask
 task request_down;begin down=1;held=1;cycle;down=0;cycle;end endtask
 initial begin
  repeat(3)cycle;rst=1;repeat(3)cycle;
  angle=0;start=1;cycle;start=0;if(allowed || !blocked)$fatal(1,"uncalibrated upright allowed");
  request_down;repeat(1500)step;
  if(events || !pending || !release_key || allowed)$fatal(1,"held S3 measured wrong DOWN");
  held=0;repeat(300)step;raw=258;repeat(999)step;
  if(events || !pending)$fatal(1,"ADC change did not restart entire settling window");
  step;if(events!=1 || pending || release_key)$fatal(1,"stable release measurement missing %d",events);
  repeat(50)step;if(events!=1)$fatal(1,"calibration request repeated");
  // The log6 stationary offset must not silently become a new calibration.
  cal=1;angle=-156000;repeat(1000)step;start=1;cycle;start=0;
  if(allowed || !blocked || events!=1)$fatal(1,"log6 mismatched DOWN accepted/rewritten");
  angle=-180000;repeat(199)step;if(allowed)$fatal(1,"down launch too early");
  step;if(!allowed || !ready)$fatal(1,"quiet calibrated down not permitted");
  start=1;cycle;start=0;if(blocked)$fatal(1,"accepted launch remains blocked");
  rate=30;cycle;if(allowed || ready)$fatal(1,"moving down permitted");
  rate=0;repeat(200)step;if(!allowed)$fatal(1,"down requalification");
  still=0;cycle;if(allowed)$fatal(1,"base moving allowed");still=1;
  angle=1000;cycle;if(!allowed)$fatal(1,"manual upright capture removed");
  adc_ok=0;cycle;if(allowed)$fatal(1,"ADC not gated");adc_ok=1;
  busy=1;cycle;if(allowed)$fatal(1,"calibration not gated");busy=0;
  // Bad ADC and moving base restart the DOWN averaging precondition.
  request_down;held=0;repeat(800)step;adc_ok=0;step;adc_ok=1;repeat(800)step;
  still=0;step;still=1;repeat(999)step;if(events!=1)$fatal(1,"not a fresh window");
  step;if(events!=2)$fatal(1,"fresh valid/still window failed");
  request_down;held=0;
  for(i=0;i<1800;i=i+1)begin raw=i%100<50?258:280;step;end
  if(events!=2 || !pending)$fatal(1,"oscillation treated as stable");
  stop=1;cycle;stop=0;repeat(1100)step;if(pending || pulse || events!=2)$fatal(1,"stop did not cancel");
  request_down;up=1;cycle;up=0;if(pending)$fatal(1,"UP did not cancel DOWN");
  request_down;rst=0;cycle;rst=1;cycle;if(pending || blocked || pulse)$fatal(1,"reset residues");
  $display("PASS r13 calibration/start guard: no held-button measurement, 1000ms stable full window, drift/ADC/base motion reset, no automatic rewriting, log6 offset rejected, 200ms down qualification, upright capture, stop/UP/reset cancellation");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
