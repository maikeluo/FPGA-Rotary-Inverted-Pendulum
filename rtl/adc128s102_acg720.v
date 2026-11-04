`timescale 1ns/1ps
// TI ADC128S102, SNAS298G sections 6.6 and 7.5.
// 50 MHz / 6 = 8.333 MHz, 50% duty, continuous 16-clock conversions.
// CS starts low while SCLK is HIGH. First falling edge launches leading zero #1.
// DOUT is sampled on the following rising edge, 60 ns after launch (>27 ns tDACC).
// DIN address bits are sampled on rising edges 3/4/5, for the NEXT conversion.
// Discard first conversion (power-up channel 0). Average 2**AVERAGE_LOG2
// full 12-bit samples BEFORE reducing to the J280 10-bit interface.
// Default 4 preserves the standalone 16-sample interface. ACG720 top selects 9:
// 512 conversions / 520833.33 SPS = 983.04 us, about 39.3 PWM periods at 40 kHz.
// This reduces carrier ripple without adding a multi-millisecond angle IIR.
// A bad individual frame is still detected before averaging.
module adc128s102_acg720 #(
 parameter [2:0] CHANNEL=3'd0,
 parameter integer AVERAGE_LOG2=4,
 parameter integer AUTOCLEAR_FAULT=1
)(input clk,input rst_n,input dout,
 output reg sclk,output reg din,output reg cs_n,
 output reg [9:0] raw,output reg valid,output reg ready,output reg fault,
 input clear_fault,output range_bad);
 localparam integer AVG_SHIFT=(AVERAGE_LOG2>=4 && AVERAGE_LOG2<=9)?AVERAGE_LOG2:4;
 localparam integer SUM_WIDTH=12+AVG_SHIFT;
 localparam integer LAST_COUNT=(1<<AVG_SHIFT)-1;
 reg [2:0] phase;
 reg [3:0] bit_index;
 reg [15:0] rx_shift;
 reg [SUM_WIDTH-1:0] sum;
 reg [AVG_SHIFT-1:0] count;
 reg first_frame,block_bad,published_bad;
 reg [4:0] startup;
 reg [15:0] captured;
 reg captured_valid;
 wire [15:0] frame_next={rx_shift[14:0],dout};
 wire [SUM_WIDTH-1:0] total=sum+{{AVG_SHIFT{1'b0}},captured[11:0]};
 wire captured_bad=(captured[15:12]!=0) || (captured[11:2]==0) || (captured[11:2]==1023);
 assign range_bad=block_bad || published_bad;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     sclk<=1;cs_n<=1;din<=0;phase<=0;bit_index<=0;rx_shift<=0;
     raw<=0;valid<=0;ready<=0;fault<=0;sum<=0;count<=0;
     first_frame<=1;block_bad<=0;published_bad<=1;startup<=0;
     captured<=0;captured_valid<=0;
   end else begin
     valid<=0;captured_valid<=0;
     if((clear_fault || AUTOCLEAR_FAULT) && ready && !range_bad && raw>0 && raw<1023)
       fault<=0;
     if(startup<16) begin
       startup<=startup+1'b1;
       // 80 ns CS setup before the first falling edge. SCLK remains HIGH.
       if(startup==12)cs_n<=0;
     end else begin
       if(phase==5)phase<=0;else phase<=phase+1'b1;
       if(phase==0) begin
         sclk<=0;
         case(bit_index)
           2:din<=CHANNEL[2];
           3:din<=CHANNEL[1];
           4:din<=CHANNEL[0];
           default:din<=0;
         endcase
       end
       if(phase==3) begin
         sclk<=1;
         rx_shift<=frame_next;
         if(bit_index==15) begin
           bit_index<=0;
           if(first_frame)first_frame<=0;
           else begin captured<=frame_next;captured_valid<=1;end
         end else bit_index<=bit_index+1'b1;
       end
     end
     if(captured_valid) begin
       if(captured_bad)fault<=1;
       if(count==LAST_COUNT) begin
         raw<=total[SUM_WIDTH-1:AVG_SHIFT+2];valid<=1;ready<=1;
         published_bad<=block_bad || captured_bad;
         block_bad<=0;sum<=0;count<=0;
       end else begin
         sum<=total;count<=count+1'b1;
         block_bad<=block_bad || captured_bad;
       end
     end
   end
 end
endmodule
