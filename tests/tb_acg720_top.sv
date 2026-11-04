`timescale 1ns/1ps
`include "adc128s102_model.svh"
module tb_acg720_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg [3:0] keys=15;reg [1:0] dip=3;
 reg ea=0,eb=0;
 wire dout,sc,di,cs,pwm,a1,a2,tx;wire[3:0]led;wire[2:0]channel;
 acg720_top #(.ADC_AVERAGE_LOG2(4),.DIAGNOSTIC(1),.CONTROL_DIV(100),.DEBOUNCE_TICKS(2)) d(
 .clk_50m(clk),.reset_n(rst),.key_n(keys),.dip_n(dip),.adc_dout(dout),
 .enc1_a(ea),.enc1_b(eb),.adc_sclk(sc),.adc_din(di),.adc_cs_n(cs),
 .pwma(pwm),.ain1(a1),.ain2(a2),.led(led),.uart_tx_o(tx));
 adc128s102_model m(cs,sc,di,12'd2048,dout,channel);
 integer highs,lows;reg signed [31:0] before_pos;
 task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
 initial begin
  #100;rst=1;wait(d.adc_ready);cycles(200);
  if(d.raw!==512 || d.adc_bad || d.adc_timeout)$fatal(1,"ADC top integration");
  if(a1 || a2 || !pwm)$fatal(1,"boot must be neutral");
  before_pos=d.position;
  // Real A/B pins, no disconnected enc2 ports.
  ea=1;cycles(40);eb=1;cycles(40);ea=0;cycles(40);eb=0;cycles(40);
  if(d.position-before_pos!=4 && d.position-before_pos!=-4)$fatal(1,"encoder pins not routed");
  cycles(3000);keys[0]=0;cycles(6000);
  highs=0;lows=0;
  repeat(1250)begin @(negedge clk);if(pwm)highs=highs+1;else lows=lows+1;end
  if(!a1 || a2 || highs!=250 || lows!=1000)$fatal(1,"A channel positive 20%% PWM missing %d %d",highs,lows);
  cycles(51000);if(a1 || a2 || d.fault!=8'h20)$fatal(1,"no-motion stop");
  keys=15;cycles(3000);keys[1]=0;cycles(6000);
  if(a1 || !a2)$fatal(1,"A channel negative jog missing");
  keys=15;#1;if(a1 || a2 || !pwm)$fatal(1,"raw release must be immediate");
  cycles(1000);if(a1 || a2 || !pwm)$fatal(1,"jog release neutral");
  rst=0;#1;if(a1 || a2)$fatal(1,"reset must immediately neutralize bridge");
  $display("PASS tb_acg720_top: SPI, encoder, A channel +/- jog, 40kHz/20%%, no-motion stop, release/reset");$finish;
 end
 initial begin #5000000;$fatal(1,"top timeout");end
endmodule
