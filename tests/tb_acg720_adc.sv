`timescale 1ns/1ps
`include "adc128s102_model.svh"
module tb_acg720_adc;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,clear=0; reg [11:0] code=12'hA54;
 wire [7:0] cs,sc,di,do_,valid,ready,fault,bad;
 wire [9:0] raw[0:7];wire [2:0] channel[0:7];
 genvar k;
 generate for(k=0;k<8;k=k+1)begin:g
   adc128s102_acg720 #(.CHANNEL(k)) d(clk,rst,do_[k],sc[k],di[k],cs[k],raw[k],valid[k],ready[k],fault[k],clear,bad[k]);
   adc128s102_model #(.DELAY_NS(k==0 ? 0 : 35)) m(cs[k],sc[k],di[k],code,do_[k],channel[k]);
 end endgenerate
 task check(input [11:0] c);
 integer j;begin
   @(negedge clk);code=c;
   repeat(3)begin @(posedge valid[0]);@(negedge clk);end
   for(j=0;j<8;j=j+1)begin
     if(raw[j]!==c[11:2])$fatal(1,"data alignment c=%h raw=%h channel=%d",c,raw[j],j);
     if(channel[j]!==j[2:0])$fatal(1,"channel address at wrong rising edges");
   end
 end endtask
 integer bitn;
 initial begin
   #100;rst=1;
   check(12'hA54);check(12'h5A8);check(12'h804);check(12'h7F8);
   for(bitn=2;bitn<12;bitn=bitn+1)check(12'b1<<bitn);
   check(12'hFFC);if(fault!==8'hff || bad!==8'hff)$fatal(1,"high rail not detected");
   check(12'h000);if(fault!==8'hff || bad!==8'hff)$fatal(1,"low rail not detected");
   check(12'h800);repeat(5)@(negedge clk);
   if(fault!==0 || bad!==0)$fatal(1,"clean samples did not restore ADC health");
   // A single rail sample must not disappear inside averaging.
   @(posedge valid[0]);
   @(negedge clk);code=0;
   #2000;code=12'h800;
   #2200;if(fault!==8'hff)$fatal(1,"isolated rail not detected");
   check(12'h800);repeat(5)@(negedge clk);
   if(fault!==0)$fatal(1,"rail recovery");
   #730;
   @(negedge clk);rst=0;#200;
   if(ready!==0 || cs!==8'hff || sc!==8'hff)$fatal(1,"reset state");
   code=12'hB6C;rst=1;
   repeat(3)@(posedge valid[0]);#2;
   if(raw[0]!==10'h2DB)$fatal(1,"midframe restart alignment");
   $display("PASS tb_acg720_adc: delayed DOUT, 8 channel commands, bit alignment, rails, reset");$finish;
 end
 initial begin #4000000;$fatal(1,"ADC timeout");end
endmodule
