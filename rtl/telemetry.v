`timescale 1ns/1ps
module uart_tx #(parameter integer DIV=434)(input clk,input rst_n,input start,
 input [7:0] data,output reg tx,output reg busy);
 reg [9:0] shift;reg [15:0] count;reg [3:0] bit_index;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin tx<=1;busy<=0;shift<=10'h3ff;count<=0;bit_index<=0;end
   else if(!busy) begin
     tx<=1;
     if(start) begin shift<={1'b1,data,1'b0};tx<=0;busy<=1;count<=0;bit_index<=0;end
   end else if(count==DIV-1) begin
     count<=0;shift<={1'b1,shift[9:1]};
     if(bit_index==9)begin busy<=0;tx<=1;end
     else begin bit_index<=bit_index+1;tx<=shift[1];end
   end else count<=count+1;
 end
endmodule

module telemetry #(parameter integer INTERVAL=1000000, UART_DIV=434, EXTENDED=0, DEBUG=0)(
 input clk,input rst_n,input [2:0] state,input [7:0] fault,input [7:0] sensor_status,
 input [7:0] flags,input [9:0] raw,input signed [31:0] angle,position,target,
 input signed [15:0] command,input [9:0] upright_raw,down_raw,output tx,
 input signed [31:0] angle_delta,velocity_q8,travel_origin,
 input [15:0] capture_ticks,input [7:0] limit_mask,keys,input [63:0] control_debug,input [95:0] release_debug);
 localparam integer FRAME_BYTES=(DEBUG>=2)?65:(DEBUG ? 53:(EXTENDED ? 45:29));
 reg [31:0] timer;reg [519:0] frame;reg [7:0] seq,index,data,checksum;
 reg sending,prepare,tx_start;wire busy;integer i;
 uart_tx #(.DIV(UART_DIV)) serial(clk,rst_n,tx_start,data,tx,busy);
 always @* begin checksum=0;for(i=2;i<FRAME_BYTES-1;i=i+1) checksum=checksum^frame[i*8+:8];end
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin timer<=0;frame<=0;seq<=0;index<=0;data<=0;sending<=0;prepare<=0;tx_start<=0;end
   else begin
     tx_start<=0;
     if(timer==INTERVAL-1)begin
       timer<=0;
       if(!sending && !prepare && !busy)begin
         frame[0+:8]<=8'ha5;frame[8+:8]<=8'h5a;frame[16+:8]<=(DEBUG>=2)?4:(DEBUG ? 3:(EXTENDED ? 2:1));frame[24+:8]<=seq;
         frame[32+:8]<={5'b0,state};frame[40+:8]<=fault;frame[48+:8]<=sensor_status;
         frame[56+:8]<=flags;frame[64+:16]<={6'b0,raw};frame[80+:32]<=angle;
         frame[112+:32]<=position;frame[144+:32]<=target;frame[176+:16]<=command;
         frame[192+:16]<={6'b0,upright_raw};frame[208+:16]<={6'b0,down_raw};
         if(EXTENDED || DEBUG)begin
           frame[224+:32]<=angle_delta;frame[256+:32]<=velocity_q8;frame[288+:32]<=travel_origin;
           frame[320+:16]<=capture_ticks;frame[336+:8]<=limit_mask;frame[344+:8]<=keys;
         end
         if(DEBUG)frame[352+:64]<=control_debug;
         if(DEBUG>=2)frame[416+:96]<=release_debug;
         prepare<=1;seq<=seq+1;
       end
     end else timer<=timer+1;
     if(prepare)begin frame[(FRAME_BYTES-1)*8+:8]<=checksum;prepare<=0;sending<=1;index<=0;end
     if(sending && !busy && !tx_start)begin
       data<=frame[index*8+:8];tx_start<=1;
       if(index==FRAME_BYTES-1)sending<=0;else index<=index+1;
     end
   end
 end
endmodule
