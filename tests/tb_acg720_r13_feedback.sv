`timescale 1ns/1ps
module tb_acg720_r13_feedback;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,sample=0,start=0,stop=0;
 reg signed [31:0] sensor_angle=-156000,sensor_rate=0,pos=0;
 wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 acg720_top d(.clk_50m(clk),.reset_n(rst),.key_n(4'b1111),.dip_n(2'b11),.adc_dout(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
 acg720_top #(.PENDULUM_REVERSE(0)) legacy(.clk_50m(clk),.reset_n(rst),.key_n(4'b1111),.dip_n(2'b11),.adc_dout(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 initial begin
  force d.tick=tick;force d.sensor_sample=sample;force d.sensor_angle=sensor_angle;force d.sensor_delta=sensor_rate;
  force d.position=pos;force d.velocity_q8=0;force d.keys=keys;force d.pressed=pressed;
  force d.calibrated=1;force d.calibrating=0;force d.stop_held=stop;force d.recovery_lock=0;
  force d.zero_encoder=0;force d.clear_fault=0;force d.adc_health_ok=1;force d.adc_range_bad=0;
  force d.encoder_bad=0;force d.stall_bad=0;
  force legacy.tick=tick;force legacy.sensor_sample=sample;force legacy.sensor_angle=sensor_angle;force legacy.sensor_delta=sensor_rate;
  force legacy.position=pos;force legacy.velocity_q8=0;force legacy.keys=keys;force legacy.pressed=pressed;
  force legacy.calibrated=1;force legacy.calibrating=0;force legacy.stop_held=stop;force legacy.recovery_lock=0;
  force legacy.zero_encoder=0;force legacy.clear_fault=0;force legacy.adc_health_ok=1;force legacy.adc_range_bad=0;
  force legacy.encoder_bad=0;force legacy.stall_bad=0;
  repeat(3)cycle;rst=1;repeat(6)cycle;repeat(300)step;
  start=1;cycle;start=0;
  if(d.state!=0 || d.control_command!=0 || !d.start_blocked || !d.sensor_report[7])$fatal(1,"log6 preflight not blocked/reported");
  if(d.angle!=156000 || legacy.angle!=-156000 || !d.sensor_report[4] || legacy.sensor_report[4])$fatal(1,"coordinate/polarity report");
  sensor_angle=-179000;repeat(210)step;start=1;cycle;start=0;repeat(85)step;
  if(d.state!=1 || legacy.state!=1 || d.control_command<=0 || legacy.control_command<=0)$fatal(1,"seed changed");
  // log6's first positive motor pulse produced negative unnormalized angle rate.
  sensor_rate=-80;repeat(100)step;
  if(d.angle_delta!=80 || legacy.angle_delta!=-80 || d.control_command<=0 || legacy.control_command>=0)$fatal(1,"negative coupling pumping direction");
  sensor_rate=80;repeat(200)step;
  if(d.control_command>=0 || legacy.control_command<=0)$fatal(1,"opposite half-cycle direction");
  sensor_angle=1000;sensor_rate=0;repeat(1200)step;
  if(d.state!=3 || legacy.state!=3 || d.control_command>=0 || legacy.control_command<=0)$fatal(1,"polarity not applied through capture/balance");
  if(d.PWM_PERIOD!=1250 || d.MOTOR_REVERSE!=0 || d.ENCODER_REVERSE!=0)$fatal(1,"motor/encoder configuration changed");
  pos=781;step;if(d.state!=4 || d.fault!=2 || d.control_command!=0)$fatal(1,"travel protection");
  stop=1;cycle;if(d.state!=0 || d.control_command!=0 || d.ar_armed)$fatal(1,"stop/retry cancellation");
  $display("PASS r13 actual top: log6 preflight, independent angle/rate normalization, positive bootstrap, both pumping directions, capture/balance polarity, unchanged motor/encoder/PWM, travel and stop; prescribed sensor fixture, not mechanical validation");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
