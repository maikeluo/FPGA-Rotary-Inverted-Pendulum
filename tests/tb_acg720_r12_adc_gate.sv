`timescale 1ns/1ps
module tb_acg720_r12_adc_gate;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,sample=0,start=0,stop=0,bad=0,range_bad=0;
 reg signed [31:0] angle=179000,rate=0,pos=0;reg [9:0] raw=257;
 wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 acg720_top dut(.clk_50m(clk),.reset_n(rst),.key_n(4'b1111),.dip_n(2'b11),.adc_dout(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.raw=raw;force dut.angle=angle;force dut.angle_delta=rate;
  force dut.position=pos;force dut.velocity_q8=0;force dut.keys=keys;force dut.pressed=pressed;
  force dut.calibrated=1;force dut.calibrating=0;force dut.stop_held=stop;force dut.recovery_lock=0;
  force dut.zero_encoder=0;force dut.clear_fault=0;force dut.adc_ready=1;force dut.adc_bad=bad;
  force dut.adc_range_bad=range_bad;force dut.adc_timeout=0;force dut.encoder_bad=0;force dut.stall_bad=0;
  repeat(3)cycle;rst=1;repeat(6)cycle;repeat(210)step;start=1;cycle;start=0;repeat(80)step;
  if(dut.state!=1 || dut.control_command==0)$fatal(1,"top bootstrap");
  range_bad=1;angle=0;raw=790;cycle;
  if(dut.control_command!=0)$fatal(1,"current ADC range not wired to stop pumping");
  // The bad block is shorter than a control period; it must still invalidate
  // the old clean window rather than silently allowing capture on next tick.
  range_bad=0;cycle;
  if(dut.closed_loop.control.coherent_clean!=0)$fatal(1,"between-tick fault did not clear clean window");
  repeat(31)step;if(dut.state!=1 || dut.control_command!=0)$fatal(1,"top bypassed clean wait");
  repeat(5)step;if(dut.state!=2)$fatal(1,"top capture handoff");
  repeat(1050)step;if(dut.state!=3)$fatal(1,"top balance");
  bad=1;repeat(4)cycle;if(dut.state!=4 || dut.fault!=1 || dut.control_command!=0)$fatal(1,"hold ADC fault protection");
  stop=1;repeat(4)cycle;if(dut.state!=0 || dut.ar_armed || dut.control_command!=0)$fatal(1,"top stop disarm");
  $display("PASS r12 actual ACG720 top: range-bad cancels pump, prevents false capture, 32 fresh samples requalify, real capture/balance, hold ADC fault and stop disarm");$finish;
 end
 initial begin #4000000;$fatal(1,"timeout");end
endmodule
