`timescale 1ns/1ps
module tb_telemetry_200hz;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;wire tx;
 // Real 50 MHz clock, 115200 baud and 5 ms interval; capacity test, no
 // accelerated UART. Decode every byte at the external TX pin.
 telemetry #(.INTERVAL(250000),.UART_DIV(434),.EXTENDED(1)) dut(
  clk,rst,3'd3,8'd0,8'd0,8'd1,10'd750,32'sd0,-32'sd200,32'sd0,
  -16'sd400,10'd750,10'd213,tx,32'sd100,-32'sd20,32'sd0,16'd0,8'd0,8'd0,64'd0,96'd0);
 integer frames=0,idx=0,i;reg [7:0] value,checksum=0;
 time previous_start=0,frame_start;
 initial begin repeat(5)@(negedge clk);rst=1;end
 initial forever begin
   @(negedge tx);frame_start=$time;
   #(434*20/2);if(tx!==0)$fatal(1,"start bit");
   for(i=0;i<8;i=i+1)begin #(434*20);value[i]=tx;end
   #(434*20);if(tx!==1)$fatal(1,"stop bit");
   if(idx==0)begin
     if(value!=8'ha5)$fatal(1,"sync A5");
     if(frames && frame_start-previous_start!=5000000)$fatal(1,"dropped frame / wrong interval");
     previous_start=frame_start;
   end
   if(idx==1 && value!=8'h5a)$fatal(1,"sync 5A");
   if(idx==2 && value!=2)$fatal(1,"protocol version");
   if(idx==3 && value!=frames)$fatal(1,"sequence gap");
   if(idx==2)checksum=value;else if(idx>2 && idx<44)checksum=checksum^value;
   if(idx==44)begin
     if(value!=checksum)$fatal(1,"checksum");
     frames=frames+1;idx=0;
     if(frames==10)begin $display("PASS ten complete 45-byte UART frames, 115200 baud, exact 5ms cadence, sequence and checksum");$finish;end
   end else idx=idx+1;
 end
 initial begin #60000000;$fatal(1,"telemetry timeout");end
endmodule
