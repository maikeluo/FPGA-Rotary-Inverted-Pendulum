`timescale 1ns/1ps
`include "adc128s102_model.svh"
module tb_acg720_calibrate_start;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg [3:0] keys=15;reg [11:0] code=3000;
 wire dout,sc,di,cs,pwm,a1,a2,tx;wire[3:0]led;wire[2:0]channel;
 acg720_top #(.ADC_AVERAGE_LOG2(4),.CONTROL_DIV(100),.DEBOUNCE_TICKS(2)) d(
 .clk_50m(clk),.reset_n(rst),.key_n(keys),.dip_n(2'b11),.adc_dout(dout),
 .enc1_a(1'b0),.enc1_b(1'b0),.adc_sclk(sc),.adc_din(di),.adc_cs_n(cs),
 .pwma(pwm),.ain1(a1),.ain2(a2),.led(led),.uart_tx_o(tx));
 adc128s102_model m(cs,sc,di,code,dout,channel);
 task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
 initial begin
   #100;rst=1;wait(d.adc_ready);cycles(1000);
   keys[0]=0;cycles(1000);keys=15;cycles(1000);
   if(d.state!=0 || a1 || a2)$fatal(1,"uncalibrated start not blocked");
   // Real debounced long-press inputs, SPI data, calibration and controller.
   keys[2]=0;cycles(130000);keys=15;cycles(8000);
   if(d.upright_raw!=750)$fatal(1,"S2 upright calibration");
   code=840;cycles(8000);keys[3]=0;cycles(130000);
   if(d.down_raw!=0 || d.calibrating)$fatal(1,"DOWN recorded before button release");
   keys=15;cycles(1000);
   if(!d.down_cal_pending || d.down_raw!=0)$fatal(1,"released long S3 did not queue stable calibration");
   cycles(111000);
   if(!d.calibrated || d.down_raw!=210 || d.sensor_status!=0)$fatal(1,"S3 down calibration");
   code=3000;cycles(20000);
   keys[0]=0;cycles(1000);keys=15;cycles(110000);
   if(d.state!=3)$fatal(1,"manual capture did not reach balance: %d",d.state);
   // Apply a small synthetic angle error to exercise the normal A outputs.
   code=2992;cycles(8000);
   if(!d.active || !(a1 || a2))$fatal(1,"normal controller not connected to A channel");
   keys[1]=0;#1;if(a1 || a2)$fatal(1,"physical S1 must immediately stop A outputs");
   cycles(1000);keys=15;cycles(1000);
   if(d.state!=0 || d.ar_armed)$fatal(1,"S1 did not cancel run");
   $display("PASS tb_acg720_calibrate_start: real SPI + S2/S3 calibration + S0 capture/balance + A output + immediate S1 stop; synthetic sensor, no plant");$finish;
 end
 initial begin #15000000;$fatal(1,"calibrate/start timeout");end
endmodule
