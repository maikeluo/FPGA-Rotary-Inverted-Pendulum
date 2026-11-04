`timescale 1ns/1ps
module tb_r4_motion;
 reg clk=0;always #10 clk=~clk;reg rst=0,tick=0,start=0,stop=0,en=1;
 reg signed [31:0] vel=0,planned=0;
 wire [2:0] state;wire [7:0] fault;wire signed [15:0] command;
 balance_control #(.MOTION_FEEDBACK_ENABLE(1),.TRIM_VELOCITY(-64),.BASE_VELOCITY_SPLIT_ENABLE(0)) dut(
 .clk(clk),.rst_n(rst),.tick(tick),.start(start),.stop(stop),.healthy(1'b1),.angle(32'd0),.angle_delta(32'd0),
 .position(32'd0),.velocity_q8(vel),.mode(2'd0),.state(state),.fault(fault),.command(command),
 .reset_reference(1'b0),.recover_reference(1'b0),.target_ext(32'd0),.target_ext_en(en),.manual_reference(start),.target_velocity_ext(planned));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(20)cycle;end endtask
 task vector(input signed [31:0] actual,input signed [31:0] expected,input signed [31:0] result);begin
  vel=actual;planned=expected;cycle;if(dut.hold_velocity_q8!==result)$fatal(1,"relative velocity sign/saturation %d",dut.hold_velocity_q8);
 end endtask
 initial begin
  repeat(3)cycle;rst=1;start=1;cycle;start=0;repeat(1020)step;if(state!=3)$fatal(1,"balance setup");
  vector(32,32,0);repeat(1500)step;
  if(dut.base_trim_pd!=0 || dut.return_damping!=0 || dut.slow_return.ready_to_return)$fatal(1,"planned velocity opposed or learned as DC bias");
  vector(-32,-32,0);repeat(1500)step;if(dut.base_trim_pd!=0)$fatal(1,"negative planned motion opposed");
  vector(-12,-32,20);vector(12,32,-20);
  vector(32'sh7fffffff,-32'sd1,32'sh7fffffff);
  vector(32'sh80000000,32'sd1,32'sh80000000);
  vector(5000,5000,0);if(!dut.speed_bad)$fatal(1,"relative speed bypassed actual-speed protection");
  en=0;cycle;if(dut.hold_velocity_q8!=5000)$fatal(1,"inactive reference still subtracted");
  vel=0;planned=0;en=1;cycle;if(!dut.slow_return.ready_to_return)$fatal(1,"stationary return loop not restored");
  stop=1;cycle;if(state!=0 || command!=0)$fatal(1,"stop");
  $display("PASS r4 motion feedback: both reference signs, zero error while following, planned motion does not train return bias, signed saturation, actual speed protection unchanged, stationary return/stop");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
