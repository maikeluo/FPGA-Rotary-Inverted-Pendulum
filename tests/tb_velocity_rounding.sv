`timescale 1ns/1ps
module tb_velocity_rounding;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,zero=0,recover=0;
 reg signed [31:0] pos=0;wire signed [31:0] mirror_pos=-pos;
 wire signed [31:0] last,velocity,mlast,mvelocity;
 encoder_velocity_filter d(clk,rst,tick,zero,recover,pos,last,velocity);
 encoder_velocity_filter m(clk,rst,tick,zero,recover,mirror_pos,mlast,mvelocity);
 integer i,j,seed=531019,expected=0,previous_position=0,old;
 integer checks=0,rounded;reg check_mirror=1;
 reg signed [63:0] delta,input_velocity,error;
 task cycle;@(negedge clk);endtask
 task step;
 begin
   old=velocity;delta=$signed(pos);delta=delta-previous_position;
   input_velocity=(delta>65535?65535:(delta< -65535?-65535:delta))*256;
   error=input_velocity-expected;
   rounded=error>=0?(error+4)/8:-((-error+4)/8);
   expected=(pos==previous_position && expected> -8 && expected<8)?0:expected+rounded;
   previous_position=pos;tick=1;cycle;tick=0;
   repeat(3)begin cycle;if(velocity!==old)$fatal(1,"velocity published early");end
   cycle;
   if(velocity!==expected || last!==pos)$fatal(1,"filter mismatch got=%0d exp=%0d",velocity,expected);
   if(check_mirror && (velocity!==-mvelocity || last!==-mlast))$fatal(1,"directional rounding asymmetry");
   if(d.phase!=0)$fatal(1,"not finished before sensor sample");
   repeat(3)cycle;checks=checks+1;
 end endtask
 task reset_history;begin recover=1;cycle;recover=0;expected=0;previous_position=pos;cycle;end endtask
 initial begin
   repeat(3)cycle;rst=1;cycle;
   // Opposite trajectories must have bit-exact opposite velocities, including
   // tiny reversals and long stationary intervals where old floor rounding differed.
   for(i=0;i<12000;i=i+1)begin pos=pos+$signed($random(seed))%6;step;end
   repeat(300)step;if(velocity!=0)$fatal(1,"stationary residue");
   for(i=0;i<2000;i=i+1)begin pos=pos+(i%2?1:-1);step;end
   check_mirror=0;pos=32'sh7fffffff;reset_history;pos=32'sh80000000;step;
   if(velocity>=-4096)$fatal(1,"negative full-width jump escaped speed guard");
   reset_history;pos=32'sh7fffffff;step;
   if(velocity<=4096)$fatal(1,"positive full-width jump escaped speed guard");
   for(j=1;j<=4;j=j+1)begin
     pos=100;reset_history;pos=102;tick=1;cycle;tick=0;repeat(j-1)cycle;
     recover=1;cycle;recover=0;repeat(6)cycle;
     if(velocity!=0 || last!=102 || d.phase!=0)$fatal(1,"recovery stage %0d",j);
     expected=0;previous_position=pos;step;if(velocity!=0)$fatal(1,"recovery false velocity");
     pos=100;reset_history;pos=102;tick=1;cycle;tick=0;repeat(j-1)cycle;
     zero=1;recover=1;cycle;zero=0;recover=0;pos=0;repeat(6)cycle;
     if(last!=0 || velocity!=0 || d.phase!=0)$fatal(1,"zero priority stage %0d",j);
     expected=0;previous_position=0;step;
   end
   pos=1;tick=1;cycle;tick=0;cycle;tick=1;cycle;tick=0;repeat(6)cycle;
   if(velocity<=4096 || d.phase!=0)$fatal(1,"impossible overlapping tick must fail high");
   $display("PASS %0d encoder filter samples: independent integer reference, mirror symmetry, four-clock commit, zero settling, wide signed jump guard, all-stage recovery/zero and overlap",checks);
   $finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
