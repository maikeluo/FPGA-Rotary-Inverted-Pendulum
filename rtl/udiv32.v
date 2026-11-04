`timescale 1ns/1ps
// Restoring divider used only when calibrating: 32 cycles, no large / operator.
module udiv32(input clk,input rst_n,input start,input [31:0] numerator,
 input [31:0] denominator,output reg busy,output reg done,output reg [31:0] quotient);
 reg [31:0] num,den,q;reg [32:0] rem;reg [5:0] count;
 wire [32:0] shifted={rem[31:0],num[31]};
 wire ge=shifted>={1'b0,den};
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin busy<=0;done<=0;quotient<=0;num<=0;den<=1;q<=0;rem<=0;count<=0;end
   else begin
     done<=0;
     if(start && !busy) begin
       if(denominator==0) begin quotient<=0;done<=1;end
       else begin busy<=1;num<=numerator;den<=denominator;q<=0;rem<=0;count<=0;end
     end else if(busy) begin
       rem<=ge ? shifted-{1'b0,den}:shifted;
       num<={num[30:0],1'b0};q<={q[30:0],ge};count<=count+1;
       if(count==31) begin quotient<={q[30:0],ge};busy<=0;done<=1;end
     end
   end
 end
endmodule
