`timescale 1ns/1ps
module tb_acg720_r4_calibration_boot;
 reg clk=0;always #10 clk=~clk;reg reset_n=0,start=0;wire [5:0] pressed={5'd0,start};
 acg720_top dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b11),.adc_dout(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
 initial begin
  force dut.pressed=pressed;force dut.keys=6'd0;
  repeat(4)@(negedge clk);reset_n=1;repeat(100)@(negedge clk);
  if(dut.FIXED_CAL_ENABLE!=0 || dut.calibrated || dut.healthy)$fatal(1,"unverified borrowed calibration active at boot");
  start=1;repeat(3)@(negedge clk);start=0;
  if(dut.state!=0 || dut.drive_enable || dut.autorec.session_enabled)$fatal(1,"uncalibrated board started motor/automatic session");
  if(dut.TELEMETRY_HZ!=100 || dut.logger.DEBUG!=2 || dut.logger.FRAME_BYTES!=65)$fatal(1,"release telemetry configuration");
  $display("PASS ACG720 normal top: r4 production boot: no borrowed calibration, start blocked until calibrated, automatic session unarmed, V4 65-byte/100Hz telemetry selected");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
