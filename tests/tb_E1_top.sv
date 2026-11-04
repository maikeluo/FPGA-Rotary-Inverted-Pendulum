`timescale 1ns/1ps
module tb_E1_top;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0,sample=0,start=0,stop=0;
 reg signed [31:0] angle=179000,rate=0,pos=0,vel=0;
 wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 j280_top dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b11),.adc_data(10'd750),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.uart_rx_o(1'b1),.adc_clk(),.adc_oe_n(),.pwma(),.ain1(),.ain2(),.pwmb(),.bin1(),.bin2(),.led(),.uart_tx_o());
 integer i;
 task cycle;begin @(negedge clk);#1;
  if(reset_n && (^dut.control_command===1'bx || dut.control_command>400 || dut.control_command< -400))$fatal(1,"command undefined or over cap");
 end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.angle=angle;force dut.angle_delta=rate;
  force dut.position=pos;force dut.velocity_q8=vel;force dut.keys=keys;force dut.pressed=pressed;
  force dut.stop_held=stop;force dut.recovery_lock=0;force dut.zero_encoder=0;force dut.clear_fault=0;
  force dut.healthy=1;
  repeat(3)cycle;reset_n=1;repeat(5)cycle;start=1;cycle;start=0;repeat(100)step;
  if(dut.state!=1)$fatal(1,"automatic swing removed");
  angle=0;rate=0;step;repeat(1010)step;if(dut.state!=3)$fatal(1,"capture/balance entry");
  vel=-20;repeat(500)step;if(dut.closed_loop.control.outer_filter.velocity_f!=-20)$fatal(1,"production filter DC");
  vel=0;repeat(700)step;
  if(dut.closed_loop.control.outer_filter.velocity_f!==0)$fatal(1,"production stationary residual");
  stop=1;repeat(12)cycle;
  if(dut.state!=0 || dut.control_command!=0 || dut.closed_loop.control.outer_filter.velocity_f!==0)$fatal(1,"production stop");
  stop=0;start=1;cycle;start=0;repeat(1010)step;
  if(dut.state!=3 || dut.closed_loop.control.outer_filter.velocity_f!==0)$fatal(1,"restart stale filter");
  angle=31000;step;if(dut.state!=4 || dut.fault!=8 || dut.control_command!=0)$fatal(1,"fall protection");
  repeat(5)cycle;if(dut.closed_loop.control.outer_filter.velocity_f!==0)$fatal(1,"fault filter clear");
  $display("PASS E1 production top: automatic swing/capture/balance, actual velocity filter path, stationary decay, STOP/restart/fall reset and command cap; no plant model");$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
