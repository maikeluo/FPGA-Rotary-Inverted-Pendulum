`timescale 1ns/1ps
module tb_telemetry;
 reg clk=0;always #5 clk=~clk;reg rst=0;wire tx;
 telemetry #(.INTERVAL(1600),.UART_DIV(4)) dut(clk,rst,3'd3,8'd0,8'd0,
 8'd1,10'd512,-32'sd1200,32'sd123,-32'sd130,-16'sd40,10'd400,10'd700,tx,
 32'sd0,32'sd0,32'sd0,16'd0,8'd0,8'd0,64'd0,96'd0);
 reg [7:0] bytes[0:28];reg [7:0] check;integer i,j;
 initial begin
   repeat(3)@(negedge clk);rst=1;
   for(i=0;i<29;i=i+1)begin
     @(negedge tx);#60; // 1.5 baud periods to first data-bit center
     for(j=0;j<8;j=j+1)begin bytes[i][j]=tx;#40;end
     if(tx!==1)$fatal(1,"UART stop bit");
   end
   check=0;for(i=2;i<28;i=i+1)check=check^bytes[i];
   if(bytes[0]!=8'ha5||bytes[1]!=8'h5a||bytes[2]!=1||bytes[4]!=3||check!=bytes[28])$fatal(1,"frame header checksum");
   if({bytes[13],bytes[12],bytes[11],bytes[10]}!==-32'sd1200)$fatal(1,"signed angle byte order");
   if({bytes[23],bytes[22]}!==-16'sd40)$fatal(1,"signed command");
   $display("PASS UART 8N1 telemetry framing signed fields checksum");$finish;
 end
 initial begin #1000000;$fatal(1,"UART timeout");end
endmodule
