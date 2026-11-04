`timescale 1ns/1ps
module tb_r4_telemetry;
 reg clk=0;always #10 clk=~clk;reg rst=0;wire tx;
 reg [95:0] release_value={16'd400,-16'sd16,-32'sd401,8'h50,8'd2,16'h5404};
 telemetry #(.INTERVAL(500000),.UART_DIV(434),.EXTENDED(1),.DEBUG(2)) dut(
 clk,rst,3'd4,8'd1,8'd0,8'd1,10'd219,-32'sd179000,-32'sd350,-32'sd390,-16'sd8,
 10'd759,10'd219,tx,-32'sd12,-32'sd32,-32'sd1,16'd0,8'd0,8'h30,64'd0,release_value);
 integer i,j,idx=0,frames=0,fd;reg [7:0] b,bytes[0:64],checksum=0;time frame_start,previous=0;
 initial begin fd=$fopen("telemetry_v4_uart.bin","wb");repeat(4)@(negedge clk);rst=1;end
 initial forever begin
  @(negedge tx);frame_start=$time;#4340;
  if(tx!==0)$fatal(1,"start bit");
  for(j=0;j<8;j=j+1)begin #8680;b[j]=tx;end
  #8680;if(tx!==1)$fatal(1,"stop bit");
  bytes[idx]=b;$fwrite(fd,"%c",b);
  if(idx==0)begin
   if(frames && frame_start-previous!=10000000)$fatal(1,"100Hz UART frame gap");
   previous=frame_start;release_value=0;
  end
  if(idx==2)checksum=b;else if(idx>2 && idx<64)checksum=checksum^b;
  if(idx==64)begin
   if(b!=checksum || bytes[0]!=8'ha5 || bytes[1]!=8'h5a || bytes[2]!=4 || bytes[3]!=frames)$fatal(1,"V4 frame/checksum");
   if({bytes[53],bytes[52]}!=16'h5404 || bytes[54]!=2 || bytes[55]!=8'h50 ||
      {bytes[59],bytes[58],bytes[57],bytes[56]}!==-32'sd401 ||
      {bytes[61],bytes[60]}!==-16'sd16 || {bytes[63],bytes[62]}!=400)$fatal(1,"release field order/snapshot");
   idx=0;frames=frames+1;release_value={16'd400,-16'sd16,-32'sd401,8'h50,8'd2,16'h5404};
   if(frames==5)begin $fclose(fd);$display("PASS r4 V4: real 115200 UART, five 65-byte frames at 100Hz, checksum, firmware tag, retry/cause flags, signed final goal/reference speed and excursion snapshots");$finish;end
  end else idx=idx+1;
 end
 initial begin #70000000;$fatal(1,"timeout");end
endmodule
