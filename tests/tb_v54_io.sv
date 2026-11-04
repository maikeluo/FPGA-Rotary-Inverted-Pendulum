`timescale 1ns/1ps
module tb_v54_io;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0;reg [5:0] keys=0;reg [3:0] key_n=4'b1111;
 reg adc_bad=0;wire pwm,in1,in2;
 j280_top #(.DIAGNOSTIC(1),.PWM_PERIOD(20),.PWM_DEAD_CYCLES(4)) dut(
  .clk_50m(clk),.reset_n(reset_n),.key_n(key_n),.dip_n(2'b11),
  .adc_data(10'd210),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),
  .enc2_a(1'b0),.enc2_b(1'b0),.uart_rx_o(1'b1),
  .adc_clk(),.adc_oe_n(),.pwma(),.ain1(),.ain2(),
  .pwmb(pwm),.bin1(in1),.bin2(in2),.led(),.uart_tx_o());
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(30)cycle;end endtask
 task neutral;begin
  if(dut.drive_enable || in1 || in2 || !pwm)$fatal(1,"diagnostic neutral output");
 end endtask
 initial begin
  force dut.tick=tick;force dut.keys=keys;force dut.pressed=0;
  force dut.adc_ready=1;force dut.adc_bad=adc_bad;
  force dut.adc_timeout=0;force dut.encoder_bad=0;
  repeat(3)cycle;reset_n=1;repeat(5)cycle;neutral;
  keys=1;key_n=4'b1110;repeat(45)cycle;
  if(!dut.drive_enable || !in1 || in2 || dut.command!=80)$fatal(1,"positive jog");
  keys=0;key_n=4'b1111;step;neutral;
  keys=2;key_n=4'b1101;repeat(45)cycle;
  if(!dut.drive_enable || in1 || !in2 || dut.command!= -80)$fatal(1,"SW2 reverse jog blocked");
  keys=3;key_n=4'b1100;step;neutral;
  keys=1;key_n=4'b1110;repeat(500)step;neutral;
  repeat(10)step;neutral;
  keys=0;key_n=4'b1111;step;
  keys=1;key_n=4'b1110;repeat(45)cycle;
  if(!dut.drive_enable)$fatal(1,"release did not rearm jog");
  adc_bad=1;cycle;neutral;
  if(dut.state!=0 || dut.active)$fatal(1,"diagnostic controller active");
  $display("PASS v54 IO: both jog directions, physical SW2 reverse, both-key neutral, 500-tick cap, release rearm, ADC inhibition, no closed-loop controller");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
