`timescale 1ns/1ps
// TB6612 STBY is tied HIGH on the supplied PCB. Neutral therefore MUST be
// IN1=0, IN2=0, PWM=1 (datasheet Stop/high-Z), not a fictional STBY output.
// Reverse requests pass through a full neutral interval before direction changes.
module motor_pwm #(parameter integer PERIOD=2500, DEAD_CYCLES=500, REVERSE=0)(
 input clk,input rst_n,input enable,input signed [15:0] command,
 output reg pwm,output reg in1,output reg in2);
 reg [31:0] phase,dead;reg signed [15:0] applied;
 reg signed [31:0] requested;reg [31:0] magnitude,threshold;
 function integer gcd(input integer a,input integer b);
   integer t;begin while(b!=0)begin t=a%b;a=b;b=t;end gcd=a;end
 endfunction
 localparam integer SCALE_GCD=gcd(PERIOD,1000);
 always @* begin
   requested=REVERSE ? -$signed(command):$signed(command);
   if(requested>1000)requested=1000;if(requested< -1000)requested=-1000;
   magnitude=(applied<0)?-applied:applied;
   threshold=(magnitude*(PERIOD/SCALE_GCD))/(1000/SCALE_GCD);
   in1=0;in2=0;pwm=1;
   if(rst_n && enable && dead==0 && applied!=0) begin
     in1=applied>0;in2=applied<0;pwm=phase<threshold;
   end
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin phase<=0;dead<=0;applied<=0;end
   else begin
     if(phase==PERIOD-1) phase<=0;else phase<=phase+1;
     if(!enable || requested==0)begin applied<=0;dead<=0;end
     else if(dead!=0) begin dead<=dead-1;applied<=0;end
     else if(applied!=0 && ((applied<0)!=(requested<0)))begin applied<=0;dead<=DEAD_CYCLES;end
     else if(phase==PERIOD-1) applied<=requested[15:0];
   end
 end
endmodule
