`timescale 1ns/1ps
module tb_r3_limits;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0,sample=0,start=0,stop=0;
 reg signed [31:0] angle=179000,pos=100;
 wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 j280_top #(.TRAVEL_LIMIT_COUNTS(600)) dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b11),.adc_data(10'd749),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.uart_rx_o(1'b1),.adc_clk(),.adc_oe_n(),.pwma(),.ain1(),.ain2(),.pwmb(),.bin1(),.bin2(),.led(),.uart_tx_o());
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.angle=angle;force dut.angle_delta=0;
  force dut.position=pos;force dut.velocity_q8=0;force dut.keys=keys;force dut.pressed=pressed;
  force dut.stop_held=stop;force dut.recovery_lock=0;force dut.zero_encoder=0;force dut.clear_fault=0;force dut.healthy=1;
  repeat(3)cycle;reset_n=1;repeat(5)cycle;start=1;cycle;start=0;step;
  pos=700;step;if(dut.state!=1)$fatal(1,"swing inclusive 600 boundary");
  pos=701;step;if(dut.state!=4 || dut.fault!=2)$fatal(1,"swing ignored limit override");
  stop=1;cycle;stop=0;pos=100;angle=0;start=1;cycle;start=0;repeat(1020)step;
  if(dut.state!=3)$fatal(1,"balance setup");
  pos=-500;step;if(dut.state!=3)$fatal(1,"hold inclusive negative boundary");
  pos=-501;step;if(dut.state!=4 || dut.fault!=2)$fatal(1,"hold ignored same limit override");
  $display("PASS r3 limit override: one 600-count parameter reaches both SWING/BALANCE, inclusive positive/negative boundaries, FAULT02 beyond budget");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
