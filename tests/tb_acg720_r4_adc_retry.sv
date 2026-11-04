`timescale 1ns/1ps
module tb_acg720_r4_adc_retry;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0,sample=0,start=0,stop=0,bad=0,encoder_bad=0;
 reg signed [31:0] angle=0,rate=0,pos=100,vel=0;
 reg [9:0] raw=749;wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 acg720_top #(.ADC_AVERAGE_LOG2(4),.FIXED_CAL_ENABLE(1),.FIXED_UP(749),.FIXED_DOWN(204)) dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b11),.adc_dout(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 task user_start;begin start=1;cycle;start=0;repeat(4)cycle;end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.raw=raw;force dut.angle=angle;force dut.angle_delta=rate;
  force dut.position=pos;force dut.velocity_q8=vel;force dut.keys=keys;force dut.pressed=pressed;
  force dut.stop_held=stop;force dut.recovery_lock=0;force dut.zero_encoder=0;force dut.clear_fault=0;
  force dut.adc_range_bad=0;force dut.adc_ready=1;force dut.adc_bad=bad;force dut.adc_timeout=0;force dut.encoder_bad=encoder_bad;force dut.stall_bad=0;
  repeat(3)cycle;reset_n=1;repeat(100)cycle;user_start;repeat(1100)step;
  if(dut.state!=3)$fatal(1,"balance setup");
  bad=1;repeat(4)cycle;
  if(dut.state!=4 || dut.fault!=1 || !dut.adc_only_trip || dut.control_command!=0)$fatal(1,"ADC cause not latched/stopped");
  angle=179000;raw=204;repeat(3000)step;
  if(dut.ar_retry || dut.ar_auto_start)$fatal(1,"retry while ADC still invalid");
  bad=0;repeat(2400)step;if(dut.ar_retry)$fatal(1,"retry before fresh down settling + backoff");
  repeat(500)step;
  if(dut.state!=1 || dut.ar_retry!=1 || dut.travel_origin!=100 || !dut.recovery_basic_hold)$fatal(1,"qualified ADC-only retry/origin/mode");
  stop=1;repeat(4)cycle;stop=0;repeat(3000)step;
  if(dut.state!=0 || dut.ar_retry!=1 || dut.ar_armed)$fatal(1,"SW2 did not cancel session");
  angle=0;raw=749;user_start;repeat(1100)step;
  encoder_bad=1;repeat(4)cycle;
  if(dut.state!=4 || dut.fault!=1 || dut.adc_only_trip)$fatal(1,"encoder fault relabelled ADC-only");
  encoder_bad=0;angle=179000;raw=204;repeat(4000)step;
  if(dut.state!=4 || dut.ar_retry || !dut.ar_gave_up)$fatal(1,"encoder fault automatically restarted");
  $display("PASS ACG720 normal top: r4 ADC recovery: actual top cause latch, immediate stop, invalid ADC blocks retry, fresh-down timing, one launch, preserved origin/basic mode, SW2 cancellation, encoder health faults excluded");$finish;
 end
 initial begin #12000000;$fatal(1,"timeout");end
endmodule
