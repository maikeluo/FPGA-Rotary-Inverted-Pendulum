`timescale 1ns/1ps
`include "adc128s102_model.svh"
module tb_acg720_r6_defaults;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,ea=0,eb=0;reg [3:0] keys=15;
 wire dout,sc,di,cs,pwm,a1,a2,tx;wire [3:0] led;wire [2:0] channel;
 // Use the shipped default averaging, 50MHz, 1ms and encoder polarity.
 acg720_top #(.DIAGNOSTIC(1),.DEBOUNCE_TICKS(2)) d(
 .clk_50m(clk),.reset_n(rst),.key_n(keys),.dip_n(2'b11),.adc_dout(dout),
 .enc1_a(ea),.enc1_b(eb),.adc_sclk(sc),.adc_din(di),.adc_cs_n(cs),
 .pwma(pwm),.ain1(a1),.ain2(a2),.led(led),.uart_tx_o(tx));
 adc128s102_model m(cs,sc,di,12'd1280,dout,channel);
 wire signed [31:0] old_position;
 wire [31:0] old_errors;
 quadrature #(.REVERSE(1)) old_encoder(clk,rst,ea,eb,1'b0,old_position,old_errors);
 task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
 task encoder_cycle;begin
  ea=1;cycles(40);eb=1;cycles(40);ea=0;cycles(40);eb=0;cycles(40);
 end endtask
 initial begin
  if(d.ENCODER_REVERSE!=0 || d.MOTOR_REVERSE!=0 || d.ADC_AVERAGE_LOG2!=9)$fatal(1,"release defaults");
  if(d.PWM_PERIOD!=1250 || d.TRAVEL_LIMIT_COUNTS!=780)$fatal(1,"drive/safety limits changed");
  #100;rst=1;
  wait(d.adc_ready);cycles(10);
  if(d.raw!=320 || d.adc_timeout || d.adc_bad)$fatal(1,"default ADC not healthy at first publish");
  cycles(1100000);if(d.adc_timeout)$fatal(1,"983us publications trigger 2ms watchdog");
  repeat(10)encoder_cycle;
  if(d.position!=40 || old_position!=-40)$fatal(1,"encoder polarity migration %d / %d",d.position,old_position);
  // The 32us encoder burst above is deliberately much faster than normal motion.
  // Let the existing velocity filter settle before asking the guarded jog to start.
  cycles(1000000);if(d.velocity_q8>=512)$fatal(1,"velocity did not settle after encoder test");
  keys[0]=0;cycles(180000);
  if(d.command<80 || d.command>200 || !a1 || a2 || !d.drive_enable)$fatal(1,"S0 positive jog cmd=%d status=%h fault=%h keys=%h v=%d adc_bad=%b encoder_bad=%b",d.command,d.jog_status,d.fault,d.keys,d.velocity_q8,d.adc_bad,d.encoder_bad);
  keys=15;cycles(1200000);keys[1]=0;cycles(180000);
  if(d.command> -80 || d.command< -200 || a1 || !a2 || !d.drive_enable)$fatal(1,"S1 negative jog");
  keys=15;cycles(180000);
  if(a1 || a2 || !pwm)$fatal(1,"jog release");
  $display("PASS tb_acg720_r6_defaults: production ADC cadence/watchdog, encoder sign vs r5, unchanged PWM/travel, real S0/S1 polarity; no physical motor model");$finish;
 end
 initial begin #95000000;$fatal(1,"default top timeout");end
endmodule
