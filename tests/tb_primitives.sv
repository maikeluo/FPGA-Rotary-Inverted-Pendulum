`timescale 1ns/1ps
module tb_primitives;
 reg clk=0;always #10 clk=~clk;reg rst=0;
 reg ds=0;reg [31:0] num=0,den=1;wire db,dd;wire [31:0] quotient;
 udiv32 div(clk,rst,ds,num,den,db,dd,quotient);
 reg a=0,b=0,zero=0;wire signed [31:0] pos;wire [31:0] errors;
 quadrature enc(clk,rst,a,b,zero,pos,errors);
 reg enable=0;reg signed [15:0] cmd=0;wire pwm,in1,in2;
 motor_pwm #(.PERIOD(100),.DEAD_CYCLES(12)) motor(clk,rst,enable,cmd,pwm,in1,in2);
 reg [9:0] adc_data=0;reg otr=0;wire adclk,oe;wire [9:0] raw;wire av,ready,bad;
 wire adc_live_bad;
 adc_3pa1030 adc(clk,rst,adc_data,otr,adclk,oe,raw,av,ready,bad,1'b0,adc_live_bad);
 always @(posedge adclk) adc_data<=#25 10'd511;
 task cycles(input integer n);repeat(n)@(negedge clk);endtask
 task divide(input [31:0] n,input [31:0] d,input [31:0] expected);
   begin num=n;den=d;ds=1;cycles(1);ds=0;wait(dd);#1;if(quotient!==expected)$fatal(1,"division %d / %d got %d expected %d",n,d,quotient,expected);cycles(2);end
 endtask
 task qstep(input [1:0] ab);begin {a,b}=ab;cycles(14);end endtask
 integer high_count,i;
 initial begin
   cycles(3);rst=1;cycles(3);
   divide(737280000,300,2457600);divide(32'hffffffff,1023,4198404);divide(7,10,0);divide(1,0,0);
   qstep(2'b10);qstep(2'b11);qstep(2'b01);qstep(2'b00);
   if(pos!==4)$fatal(1,"quadrature positive %d",pos);
   qstep(2'b01);qstep(2'b11);qstep(2'b10);qstep(2'b00);
   if(pos!==0)$fatal(1,"quadrature reverse");
   qstep(2'b11);if(errors!==1 || pos!==0)$fatal(1,"illegal transition");
   zero=1;cycles(1);zero=0;cycles(2);if(pos!==0||errors!==0)$fatal(1,"zero");
   enable=1;cmd=250;cycles(220);high_count=0;
   for(i=0;i<100;i=i+1)begin cycles(1);if(pwm)high_count=high_count+1;if(!in1||in2)$fatal(1,"positive direction");end
   if(high_count!=25)$fatal(1,"PWM duty %d",high_count);
   cmd=-400;cycles(1);
   if(in1||in2||!pwm)$fatal(1,"reverse neutral");
   cycles(10);if(in1||in2)$fatal(1,"dead time too short");
   cycles(200);if(in1||!in2)$fatal(1,"reverse direction");
   enable=0;#1;if(in1||in2||!pwm)$fatal(1,"disabled must be high-Z stop");
   wait(ready);cycles(1);if(raw!==511 || oe!==0)$fatal(1,"ADC acquisition %d",raw);
   otr=1;cycles(40);if(!bad)$fatal(1,"OTR not latched");
   $display("PASS primitives divider encoder ADC PWM");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
