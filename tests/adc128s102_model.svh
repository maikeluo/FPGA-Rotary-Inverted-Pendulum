// Independent pin-level model of TI SNAS298G Figure 1 / Table 1.
// 35 ns includes the 27 ns maximum DOUT access plus 8 ns routing margin.
module adc128s102_model #(parameter integer DELAY_NS=35)(
 input cs_n,input sclk,input din,input [11:0] code,
 output reg dout,output reg [2:0] selected_channel);
 integer falls=0,rises=0;
 reg [15:0] frame;
 reg [7:0] control_byte;
 time last_fall=0,last_rise=0,last_din=0;
 always @(din) last_din=$time;
 always @(negedge cs_n) begin
   if(sclk!==1'b1)$fatal(1,"CS must fall while SCLK high");
   falls=0;rises=0;selected_channel=0;control_byte=0;
 end
 always @(posedge cs_n) dout<=1'bz;
 always @(negedge sclk) if(!cs_n) begin
   if(falls==0)frame={4'b0000,code};
   if(last_fall!=0 && $time-last_fall<120)$fatal(1,"SCLK above specified frequency");
   last_fall=$time;
   dout<=#(DELAY_NS) frame[15-falls];
   falls=(falls+1)%16;
 end
 always @(posedge sclk) if(!cs_n) begin
   if($time-last_din<10)$fatal(1,"DIN setup <10 ns");
   if($time-last_fall!=60)$fatal(1,"wrong low phase");
   if(rises<8)control_byte={control_byte[6:0],din};
   rises=(rises+1)%16;
   if(rises==0)selected_channel=control_byte[5:3];
   last_rise=$time;
 end
endmodule
