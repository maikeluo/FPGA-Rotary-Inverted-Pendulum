`timescale 1ns/1ps
`include "adc128s102_model.svh"
// Accelerate the control/debounce/telemetry cadence only for this integration
// fixture. The ADC, motor_pwm, physical key inputs, encoder and UART path run
// as real RTL; no forced internal inputs. Production-rate tests remain present.
module tb_r9_io_uart;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg [3:0] keys=15;reg [1:0] ab=2'b10;
 wire dout,sc,di,cs,pwm,a1,a2,tx;wire [2:0] channel;
 acg720_top #(.DIAGNOSTIC(1),.CONTROL_DIV(50),.DEBOUNCE_TICKS(3),.ADC_AVERAGE_LOG2(4)) d(
 .clk_50m(clk),.reset_n(rst),.key_n(keys),.dip_n(2'b11),.adc_dout(dout),
 .enc1_a(ab[1]),.enc1_b(ab[0]),.adc_sclk(sc),.adc_din(di),.adc_cs_n(cs),
 .pwma(pwm),.ain1(a1),.ain2(a2),.uart_tx_o(tx));
 defparam d.logger.INTERVAL=5000,d.logger.UART_DIV=4;
 adc128s102_model m(cs,sc,di,12'd1280,dout,channel);
 reg finished=0;integer fd,idx=0,j,i,frames=0;reg [7:0] byte_value,bytes[0:64],checksum;
 task cycles(input integer n);repeat(n)@(negedge clk);endtask
 task phase(input [1:0] value);begin ab=value;cycles(30);end endtask
 initial begin
   fd=$fopen("io_trace_uart.bin","wb");cycles(5);rst=1;
   cycles(6000);
   if(d.position || d.enc_errors)$fatal(1,"actual top invented startup motion");
   keys[0]=0;cycles(3500);
   if(!d.drive_enable || d.io_debug[31:16]!=1 || d.io_pwm_pulses==0)$fatal(1,"actual key/motor trace missing");
   keys=15;#1;if(d.drive_enable)$fatal(1,"release gate delayed");cycles(1500);
   repeat(2)begin phase(2'b11);phase(2'b01);phase(2'b00);phase(2'b10);end
   cycles(1000);
   if(d.position!=8 || d.io_debug[47:32]!=4 || d.io_debug[63:48]!=4 || d.io_key_counts!=1 || d.io_seen!=8'h11)$fatal(1,"actual encoder/input trace missing");
   if(a1 || a2 || !pwm || d.io_pwm_pulses==0)$fatal(1,"stop erased pulse record");
   finished=1;
 end
 initial forever begin
   @(negedge tx);#40;
   if(tx!==0)$fatal(1,"UART start");
   for(j=0;j<8;j=j+1)begin #80;byte_value[j]=tx;end
   #80;if(tx!==1)$fatal(1,"UART stop");
   bytes[idx]=byte_value;$fwrite(fd,"%c",byte_value);
   if(idx==64)begin
     checksum=0;for(i=2;i<64;i=i+1)checksum=checksum^bytes[i];
     if(bytes[0]!=8'ha5 || bytes[1]!=8'h5a || bytes[2]!=4 || bytes[64]!=checksum || {bytes[53],bytes[52]}!=16'hA413)$fatal(1,"frame/tag/checksum");
     frames=frames+1;idx=0;
     if(finished && {bytes[51],bytes[50]}==4 && {bytes[49],bytes[48]}==4)begin
       if({bytes[17],bytes[16],bytes[15],bytes[14]}!=8 || {bytes[47],bytes[46]}!=1 ||
          {bytes[13],bytes[12],bytes[11],bytes[10]}!=1 || bytes[55]!=8'h11 ||
          {bytes[61],bytes[60]}==0 || bytes[7]!=8'h20 || bytes[5]!=0 || {bytes[45],bytes[44]}!=16'h5100)$fatal(1,"retained telemetry mapping");
       $fclose(fd);$display("PASS r9 real IO/UART integration: boot AB10 no false count, S0 accepted once, active PWM recorded, release stops, two real encoder cycles count +8, A/B=4/4, 65-byte A413 snapshot retains evidence after stop");$finish;
     end
   end else idx=idx+1;
 end
 initial begin #1000000;$fatal(1,"IO UART timeout");end
endmodule
