`timescale 1ns/1ps
module tb_telemetry_extended;
 reg clk=0;always #5 clk=~clk;reg rst=0;wire tx;
 reg signed [31:0] origin=1122;
 telemetry #(.INTERVAL(2400),.UART_DIV(4),.EXTENDED(1)) dut(clk,rst,3'd2,8'd0,8'd0,
 8'd1,10'd746,-32'sd1200,32'sd1122,32'sd1122,-16'sd40,10'd746,10'd210,tx,
 -32'sd83,-32'sd256,origin,16'd999,8'd1,8'd3,64'd0,96'd0);
 reg [7:0] bytes[0:44];reg [7:0] check;integer i,j;
 initial begin
   repeat(3)@(negedge clk);rst=1;
   for(i=0;i<45;i=i+1)begin
     @(negedge tx);#60;
     for(j=0;j<8;j=j+1)begin bytes[i][j]=tx;#40;end
     if(tx!==1)$fatal(1,"UART stop bit");
     origin=9999; // the entire frame must retain its initial snapshot
   end
   check=0;for(i=2;i<44;i=i+1)check=check^bytes[i];
   if(bytes[0]!=8'ha5||bytes[1]!=8'h5a||bytes[2]!=2||check!=bytes[44])$fatal(1,"extended header/checksum");
   if({bytes[31],bytes[30],bytes[29],bytes[28]}!==-32'sd83)$fatal(1,"rate byte order");
   if({bytes[35],bytes[34],bytes[33],bytes[32]}!==-32'sd256)$fatal(1,"velocity byte order");
   if({bytes[39],bytes[38],bytes[37],bytes[36]}!==32'd1122)$fatal(1,"origin snapshot");
   if({bytes[41],bytes[40]}!==16'd999 || bytes[42]!=1 || bytes[43]!=3)$fatal(1,"settle/limit/keys");
   $display("PASS extended UART version 2: 45 bytes, signed rate/velocity, origin snapshot, capture ticks, limits, keys, checksum");$finish;
 end
 initial begin #1000000;$fatal(1,"extended UART timeout");end
endmodule
