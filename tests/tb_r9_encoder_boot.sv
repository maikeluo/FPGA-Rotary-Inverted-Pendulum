`timescale 1ns/1ps
module tb_r9_encoder_boot;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,zero=0;reg [1:0] ab=0;
 wire signed [31:0] pos,rev;wire [31:0] errors,rev_errors;
 quadrature d(clk,rst,ab[1],ab[0],zero,pos,errors);
 quadrature #(.REVERSE(1),.FILTER_CYCLES(0)) inverse(clk,rst,ab[1],ab[0],zero,rev,rev_errors);
 task cycles(input integer n);repeat(n)@(negedge clk);endtask
 function [1:0] next_positive(input [1:0] phase);
  case(phase)0:next_positive=2;2:next_positive=3;3:next_positive=1;1:next_positive=0;endcase
 endfunction
 integer phase,i;
 initial begin
  for(phase=0;phase<4;phase=phase+1)begin
   rst=0;ab=phase;cycles(3);rst=1;cycles(40);
   if(pos || rev || errors || rev_errors)$fatal(1,"phantom start phase=%b pos=%d errors=%d",ab,pos,errors);
   for(i=1;i<=8;i=i+1)begin
    ab=next_positive(ab);cycles(20);
    if(pos!=i || rev!=-i || errors || rev_errors)$fatal(1,"first real steps lost after priming");
   end
   ab=ab^2'b11;cycles(20);
   if(errors!=1 || rev_errors!=1 || pos!=8)$fatal(1,"illegal transition guard removed");
   zero=1;cycles(2);zero=0;cycles(20);
   if(pos || rev || errors || rev_errors)$fatal(1,"explicit zero");
  end
  $display("PASS r9 encoder: all four power-up phases produce zero position/errors, default and zero-length filters prime correctly, first genuine edge retained, reverse/illegal/zero behavior preserved");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
