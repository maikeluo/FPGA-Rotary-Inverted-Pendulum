`timescale 1ns/1ps
module tb_r3_health;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0,sample=0,start=0,stop=0,bad=0;
 reg signed [31:0] angle=0,rate=0,pos=0,vel=0;
 reg [9:0] raw=749;wire [5:0] keys={4'd0,stop,1'b0},pressed={5'd0,start};
 j280_top #(.FIXED_CAL_ENABLE(1),.FIXED_UP(749),.FIXED_DOWN(204)) dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b00),.adc_data(10'd749),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.uart_rx_o(1'b1),.adc_clk(),.adc_oe_n(),.pwma(),.ain1(),.ain2(),.pwmb(),.bin1(),.bin2(),.led(),.uart_tx_o());
 task cycle;begin @(negedge clk);#1;end endtask
 task step;begin tick=1;cycle;tick=0;repeat(6)cycle;sample=1;cycle;sample=0;repeat(20)cycle;end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.raw=raw;force dut.angle=angle;force dut.angle_delta=rate;
  force dut.position=pos;force dut.velocity_q8=vel;force dut.keys=keys;force dut.pressed=pressed;
  force dut.stop_held=stop;force dut.recovery_lock=0;force dut.zero_encoder=0;force dut.clear_fault=0;
  force dut.adc_ready=1;force dut.adc_bad=bad;force dut.adc_timeout=0;force dut.encoder_bad=0;force dut.stall_bad=0;
  if(dut.PWM_PERIOD!=1250 || dut.PWM_DEAD_CYCLES!=250 || dut.TRIM_VELOCITY_V!=-64 || dut.TRAJ_TARGET_COUNTS!=400 || dut.TRAJ_VMAX!=4096 || dut.AUTO_RETRY_LIMIT!=3 || dut.TRAVEL_LIMIT_COUNTS!=780)$fatal(1,"production defaults changed/wrong");
  repeat(3)cycle;reset_n=1;repeat(100)cycle;
  if(!dut.calibrated || dut.upright_raw!=749 || dut.down_raw!=204)$fatal(1,"current client fixed calibration changed");
  if(dut.mode_sel!=0)$fatal(1,"raw DIP bypassed filtered keys");
  start=1;cycle;start=0;repeat(1400)step;
  if(dut.state!=3 || !dut.traj_arrived || !dut.led[2] || !dut.led[1])$fatal(1,"balance/real-arrival LED mapping");
  bad=1;repeat(3)cycle;if(dut.state!=4 || dut.fault!=1 || dut.control_command!=0)$fatal(1,"live ADC fault ignored in BALANCE");
  stop=1;cycle;bad=0;stop=0;angle=179000;raw=210;start=1;cycle;start=0;repeat(4)step;
  if(dut.state!=1)$fatal(1,"swing removed");
  bad=1;repeat(4)step;if(dut.state!=1)$fatal(1,"analogue swing dead-region exemption removed");
  stop=1;repeat(3)cycle;if(dut.state!=0 || dut.control_command!=0 || dut.ar_armed)$fatal(1,"stop did not disarm");
  $display("PASS r3 production: preserved 40kHz/5us reversal, -64 gains, +/-400 targets, 749/204 fixed calibration and LEDs; debounced mode, true arrival, current ADC fault stop, SWING exemption, SW2 disarm");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
