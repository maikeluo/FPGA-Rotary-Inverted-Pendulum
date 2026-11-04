`timescale 1ns/1ps
module tb_telemetry_debug_200hz;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;wire tx;
 reg [63:0] debug_value={16'sd384,-16'sd123,16'sd275,-16'sd87};
 telemetry #(.INTERVAL(250000),.UART_DIV(434),.EXTENDED(1),.DEBUG(1)) d(
  clk,rst,3'd3,8'd0,8'd0,8'd1,10'd750,-32'sd333,-32'sd200,32'sd0,
  -16'sd400,10'd750,10'd210,tx,-32'sd100,-32'sd20,32'sd0,16'd0,8'd0,8'd0,debug_value,96'd0);
 integer frames=0,idx=0,i,fd;reg [7:0] value,checksum=0,bytes[0:52];
 time previous_start=0,frame_start;
 initial begin fd=$fopen("telemetry_v3_uart.bin","wb");if(!fd)$fatal(1,"capture open");repeat(5)@(negedge clk);rst=1;end
 initial forever begin
   @(negedge tx);frame_start=$time;
   #(434*20/2);if(tx!==0)$fatal(1,"start bit");
   for(i=0;i<8;i=i+1)begin #(434*20);value[i]=tx;end
   #(434*20);if(tx!==1)$fatal(1,"stop bit");
   bytes[idx]=value;$fwrite(fd,"%c",value);
   if(idx==0)begin
     if(value!=8'ha5)$fatal(1,"sync A5");
     if(frames && frame_start-previous_start!=5000000)$fatal(1,"53-byte frame dropped / cadence changed");
     previous_start=frame_start;
     debug_value=0; // mutate inputs DURING serialization; keep frame snapshot
   end
   if(idx==1 && value!=8'h5a)$fatal(1,"sync 5A");
   if(idx==2 && value!=3)$fatal(1,"version 3");
   if(idx==3 && value!=frames)$fatal(1,"sequence gap");
   if(idx==2)checksum=value;else if(idx>2 && idx<52)checksum=checksum^value;
   if(idx==52)begin
     if(value!=checksum)$fatal(1,"checksum");
     if({bytes[45],bytes[44]}!==-16'sd87 || {bytes[47],bytes[46]}!==16'sd275 ||
        {bytes[49],bytes[48]}!==-16'sd123 || {bytes[51],bytes[50]}!==16'sd384)
       $fatal(1,"signed diagnostics/order/snapshot");
     frames=frames+1;idx=0;
     debug_value={16'sd384,-16'sd123,16'sd275,-16'sd87};
     if(frames==10)begin $fclose(fd);$display("PASS ten complete 53-byte v3 frames, real 115200 baud, exact 5ms cadence, signed debug snapshots/checksum and raw UART capture");$finish;end
   end else idx=idx+1;
 end
 initial begin #60000000;$fatal(1,"telemetry timeout");end
endmodule
