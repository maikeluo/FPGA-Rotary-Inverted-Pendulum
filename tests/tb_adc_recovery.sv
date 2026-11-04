`timescale 1ns/1ps
module tb_adc_recovery;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,otr=0,clear_fault=0;reg [9:0] code=210,data=210;
 wire adc_clk,oe,valid,ready,fault,range_bad;wire [9:0] raw;
 adc_3pa1030 dut(clk,rst,data,otr,adc_clk,oe,raw,valid,ready,fault,clear_fault,range_bad);
 always @(posedge adc_clk)data<=#25 code;
 task cycles(input integer n);repeat(n)@(negedge clk);endtask
 initial begin
   cycles(4);rst=1;cycles(1000);
   if(!ready || range_bad || raw!=210)$fatal(1,"clean ADC fixture");
   otr=1;wait(dut.captured && dut.captured_otr);
   @(negedge clk);clear_fault=1;cycles(2);
   if(!fault || !range_bad)$fatal(1,"new captured OTR must override clear");
   cycles(800);if(!fault)$fatal(1,"persistent OTR must not be cleared");
   clear_fault=0;otr=0;cycles(1000);
   if(range_bad || !fault)$fatal(1,"live qualifier must recover while fault stays latched");
   code=0;cycles(1000);clear_fault=1;cycles(10);
   if(!fault)$fatal(1,"zero code must reject clear");
   clear_fault=0;code=1023;cycles(1000);clear_fault=1;cycles(10);
   if(!fault)$fatal(1,"full-scale code must reject clear");
   clear_fault=0;code=210;cycles(1000);clear_fault=1;cycles(2);clear_fault=0;
   if(fault || range_bad || raw!=210)$fatal(1,"clean explicit clear failed");
   otr=1;cycles(40);otr=0;cycles(1000);
   if(!fault || range_bad)$fatal(1,"short subsequent OTR must latch independently");
   $display("PASS ADC recovery: clear cannot defeat captured/live OTR; endpoint rejection; live versus latched status; explicit clear; short subsequent OTR");
   $finish;
 end
 initial begin #1000000;$fatal(1,"ADC recovery timeout");end
endmodule
