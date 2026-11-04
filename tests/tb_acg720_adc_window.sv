`timescale 1ns/1ps
`include "adc128s102_model.svh"
module tb_acg720_adc_window;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,clear=0,ripple=1;
 reg [11:0] level=1280;
 integer pwm_phase=0;
 always @(posedge clk) if(!rst)pwm_phase<=0;
 else if(pwm_phase==1249)pwm_phase<=0;else pwm_phase<=pwm_phase+1;
 wire [11:0] code=ripple ? (pwm_phase<625 ? level+12'd128:level-12'd128):level;
 wire [1:0] cs,sc,di,do_,valid,ready,fault,bad;
 wire [9:0] raw[0:1];wire [2:0] channel[0:1];
 genvar k;
 generate for(k=0;k<2;k=k+1)begin:g
  adc128s102_acg720 #(.AVERAGE_LOG2(k==0?4:9)) d(
   clk,rst,do_[k],sc[k],di[k],cs[k],raw[k],valid[k],ready[k],fault[k],clear,bad[k]);
  adc128s102_model m(cs[k],sc[k],di[k],code,do_[k],channel[k]);
 end endgenerate
 integer lo0=1023,hi0=0,lo1=1023,hi1=0,n0=0,n1=0;
 time prev_valid=0;reg measure=0;
 always @(posedge valid[0]) if(measure)begin
  if(raw[0]<lo0)lo0=raw[0];if(raw[0]>hi0)hi0=raw[0];n0=n0+1;
 end
 always @(posedge valid[1])begin
  if(prev_valid!=0 && $time-prev_valid!=983040)$fatal(1,"512-sample cadence %0t",$time-prev_valid);
  prev_valid=$time;
  if(measure)begin
   if(raw[1]<lo1)lo1=raw[1];if(raw[1]>hi1)hi1=raw[1];n1=n1+1;
  end
 end
 task wait_clean(input [11:0] value);
 begin
  @(negedge clk);ripple=0;level=value;
  repeat(3)begin @(posedge valid[1]);@(negedge clk);end
  if(raw[1]!==value[11:2])$fatal(1,"sum width/scaling: value=%d raw=%d",value,raw[1]);
 end endtask
 initial begin
  #100;rst=1;
  repeat(2)@(posedge valid[1]);@(negedge clk);measure=1;
  repeat(8)@(posedge valid[1]);@(negedge clk);measure=0;
  if(n0<200 || n1<8 || hi0-lo0<8 || hi1-lo1>1 || lo1<319 || hi1>320)
   $fatal(1,"ripple rejection old=%d..%d new=%d..%d",lo0,hi0,lo1,hi1);
  $display("40kHz synthetic ADC ripple, 256/4096 pk-pk: 16-sample=%0d codes pk-pk, 512-sample=%0d codes pk-pk",hi0-lo0,hi1-lo1);
  wait_clean(4091);wait_clean(4);wait_clean(2048);
  if(fault[1] || bad[1])$fatal(1,"clean ADC health");
  // Individual rail must trip within two converter frames, not after 983us.
  @(posedge valid[1]);@(negedge clk);level=0;
  #4000;if(!fault[1])$fatal(1,"averaging hid a single rail frame");
  level=2048;
  #2000;if(!bad[1])$fatal(1,"bad block must remain visible");
  wait_clean(2048);if(fault[1] || bad[1])$fatal(1,"clean-window recovery");
  // Reset halfway through a block: old partial sum must not leak into new data.
  #137000;rst=0;prev_valid=0;#200;level=3000;rst=1;
  @(posedge valid[1]);#1;
  if(raw[1]!=750)$fatal(1,"reset retained old averaging history");
  $display("PASS tb_acg720_adc_window: 983.04us cadence, 40kHz ripple rejection, full-scale width, low code, immediate rail, recovery, partial-block reset");
  $finish;
 end
 initial begin #40000000;$fatal(1,"ADC window timeout");end
endmodule
