`timescale 1ns/1ps
module tb_calibration_reuse;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,up=0,down=0,clear_fault=0,bad=0;
 reg [9:0] raw=750;
 wire calibrated,calibrating,zero,sample;
 wire [9:0] upright,downward;wire [7:0] status;
 wire signed [31:0] angle,rate;
 angle_sensor dut(clk,rst,tick,raw,1'b1,bad,up,down,1'b1,
   calibrated,calibrating,zero,upright,downward,status,angle,rate,sample,clear_fault);
 // Fixed constants are test fixtures, not the delivered board calibration.
 wire boot_ok,invalid_ok,reverse_ok;wire signed [31:0] boot_angle,reverse_angle;
 angle_sensor #(.FIXED_CAL_ENABLE(1),.FIXED_UP(750),.FIXED_DOWN(210)) boot(
   .clk(clk),.rst_n(rst),.tick(tick),.raw(raw),.adc_ready(1'b1),.adc_bad(1'b0),
   .cal_up(1'b0),.cal_down(1'b0),.allow_cal(1'b1),.calibrated(boot_ok),
   .angle(boot_angle),.clear_fault(1'b0));
 angle_sensor #(.FIXED_CAL_ENABLE(1),.FIXED_UP(750),.FIXED_DOWN(751)) invalid_boot(
   .clk(clk),.rst_n(rst),.tick(tick),.raw(raw),.adc_ready(1'b1),.adc_bad(1'b0),
   .cal_up(1'b0),.cal_down(1'b0),.allow_cal(1'b1),.calibrated(invalid_ok),.clear_fault(1'b0));
 angle_sensor #(.FIXED_CAL_ENABLE(1),.FIXED_UP(210),.FIXED_DOWN(750)) reverse_boot(
   .clk(clk),.rst_n(rst),.tick(tick),.raw(raw),.adc_ready(1'b1),.adc_bad(1'b0),
   .cal_up(1'b0),.cal_down(1'b0),.allow_cal(1'b1),.calibrated(reverse_ok),
   .angle(reverse_angle),.clear_fault(1'b0));
 task clocks(input integer n);repeat(n)@(negedge clk);endtask
 task sample_tick;begin tick=1;clocks(1);tick=0;clocks(12);end endtask
 task reset_board;begin rst=0;clocks(3);rst=1;clocks(60);end endtask
 task calibrate(input integer value,input integer is_down);
   begin raw=value;up=!is_down;down=is_down;clocks(1);up=0;down=0;
     repeat(64)sample_tick();clocks(60);
   end
 endtask
 task check_pair(input integer u,input integer d);
   if(!calibrated || upright!=u || downward!=d || status!=0)
     $fatal(1,"wrong restored pair cal=%d up=%d down=%d status=%h",calibrated,upright,downward,status);
 endtask
 initial begin
   reset_board();
   if(calibrated || !boot_ok || invalid_ok || !reverse_ok)$fatal(1,"boot validation");
   raw=750;sample_tick();if(boot_angle!=0)$fatal(1,"fixed upright zero");
   raw=480;sample_tick();
   if(boot_angle<89999 || boot_angle>90001 || reverse_angle<89999 || reverse_angle>90001)
     $fatal(1,"boot scale/sign");
   calibrate(750,0);if(calibrated)$fatal(1,"partial calibration accepted");
   reset_board();if(calibrated)$fatal(1,"partial calibration cached");
   calibrate(750,0);calibrate(210,1);check_pair(750,210);
   raw=600;sample_tick();clear_fault=1;clocks(1);clear_fault=0;sample_tick();
   check_pair(750,210);if(rate!=0)$fatal(1,"recovery derivative not primed");
   repeat(3)begin
     reset_board();check_pair(750,210);raw=750;sample_tick();
     if(angle!=0 || rate!=0)$fatal(1,"reset derivative/zero mismatch");
   end
   calibrate(760,0);check_pair(760,210);reset_board();check_pair(760,210);
   // Failed recalibration blocks starting, but SW5 restores previous valid pair.
   bad=1;calibrate(780,0);if(calibrated || status!=1)$fatal(1,"bad calibration accepted");
   bad=0;reset_board();check_pair(760,210);
   // A reset during acquisition restores the old pair, never partial data.
   raw=770;up=1;clocks(1);up=0;repeat(10)sample_tick();
   reset_board();check_pair(760,210);
   // Span failure also cannot replace the saved pair.
   calibrate(750,1);if(calibrated || status!=4)$fatal(1,"short span accepted");
   reset_board();check_pair(760,210);
   $display("PASS calibration default boot, fixed/reverse scale, invalid defaults, SW5 reuse, fault recovery, successful/failed/interrupted recalibration");
   $finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
