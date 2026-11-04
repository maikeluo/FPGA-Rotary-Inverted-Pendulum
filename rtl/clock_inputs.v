`timescale 1ns/1ps
// All internal logic remains in the 50 MHz domain. tick is an enable.
module tick_gen #(parameter integer DIV=50000)(input clk,input rst_n,output reg tick);
  reg [31:0] count;
  always @(posedge clk or negedge rst_n)
    if(!rst_n) begin count<=0;tick<=0;end
    else begin tick<=0;if(count==DIV-1) begin count<=0;tick<=1;end else count<=count+1;end
endmodule

module input_filter #(parameter integer WIDTH=6, STABLE_TICKS=10)(
  input clk,input rst_n,input tick,input [WIDTH-1:0] pins,
  output reg [WIDTH-1:0] level,output reg [WIDTH-1:0] rise);
  (* ASYNC_REG="TRUE" *) reg [WIDTH-1:0] meta,sync;
  reg [7:0] count[0:WIDTH-1];integer i;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin meta<=0;sync<=0;level<=0;rise<=0;for(i=0;i<WIDTH;i=i+1) count[i]<=0;end
    else begin
      meta<=pins;sync<=meta;rise<=0;
      if(tick) for(i=0;i<WIDTH;i=i+1) begin
        if(sync[i]==level[i]) count[i]<=0;
        else if(count[i]==STABLE_TICKS-1) begin
          level[i]<=sync[i];rise[i]<=sync[i];count[i]<=0;
        end else count[i]<=count[i]+1;
      end
    end
  end
endmodule
