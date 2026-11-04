`timescale 1ns/1ps
module tb_r3_unified;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,tick=0,sample=0,start=0,stop=0;
 reg [1:0] mode=0;
 reg signed [31:0] angle=179000,rate=0,pos=100,vel=0;
 wire [5:0] keys={mode,2'b00,stop,1'b0},pressed={5'd0,start};
 j280_top dut(.clk_50m(clk),.reset_n(reset_n),.key_n(4'b1111),.dip_n(2'b11),.adc_data(10'd750),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.uart_rx_o(1'b1),.adc_clk(),.adc_oe_n(),.pwma(),.ain1(),.ain2(),.pwmb(),.bin1(),.bin2(),.led(),.uart_tx_o());
 integer i,seen_positive=0,seen_negative=0;reg follow=0;
 task cycle;begin @(negedge clk);#1;
  if(reset_n && (^dut.control_command===1'bx || dut.control_command>400 || dut.control_command< -400))$fatal(1,"command unknown/limit");
 end endtask
 task step;begin
  if(follow)begin pos=dut.traj_ref;vel=dut.traj_speed;angle=dut.closed_loop.control.base_trim;end
  tick=1;cycle;tick=0;repeat(6)cycle;
  if(follow)begin pos=dut.traj_ref;vel=dut.traj_speed;end
  sample=1;cycle;sample=0;repeat(20)cycle;
 end endtask
 initial begin
  force dut.tick=tick;force dut.sample=sample;force dut.angle=angle;force dut.angle_delta=rate;
  force dut.position=pos;force dut.velocity_q8=vel;force dut.keys=keys;force dut.pressed=pressed;
  force dut.stop_held=stop;force dut.recovery_lock=0;force dut.zero_encoder=0;force dut.clear_fault=0;force dut.healthy=1;
  repeat(3)cycle;reset_n=1;repeat(5)cycle;start=1;cycle;start=0;repeat(100)step;
  if(dut.state!=1 || dut.travel_origin!=100)$fatal(1,"manual origin/automatic swing");
  pos=105;angle=0;step;repeat(1010)step;if(dut.state!=3)$fatal(1,"balance entry");
  follow=1;repeat(2500)step;
  if(dut.target!=100 || dut.traj_final!=100)$fatal(1,"basic mode did not hold start origin");
  mode=1;repeat(9500)step;
  if(dut.traj_final!=500 || dut.target!=500 || !dut.traj_arrived)$fatal(1,"positive extension");
  mode=2;repeat(16000)step;
  if(dut.traj_final!=-300 || dut.target!=-300 || !dut.traj_arrived)$fatal(1,"negative extension");
  // Keep the uploaded velocity feedback law while testing all four modes.
  mode=3;
  for(i=0;i<35000;i=i+1)begin
   step;
   if(dut.traj_final==500)seen_positive=1;
   if(seen_positive && dut.traj_final==-300)seen_negative=1;
  end
  if(!seen_positive || !seen_negative || dut.traj_error)$fatal(1,"shuttle integration");
  stop=1;repeat(15)cycle;
  if(dut.state!=0 || dut.control_command!=0 || dut.traj_speed!=0 || dut.traj_error)$fatal(1,"common stop");
  stop=0;follow=0;pos=101;vel=0;angle=179000;
  // Automatic launch preserves manual origin and resumes in basic mode.
  force dut.ar_auto_start=1;cycle;release dut.ar_auto_start;repeat(5)step;
  if(dut.travel_origin!=100 || !dut.recovery_basic_hold || dut.mode_sel!=0)$fatal(1,"automatic origin/mode preservation");
  pos=1000;step;
  if(dut.state!=4 || dut.fault!=2 || dut.control_command!=0)$fatal(1,"bounded travel protection");
  $display("PASS v54 one firmware: automatic swing, captured manual origin, basic hold, +/-400 targets, settled shuttle, common stop, automatic recovery origin/mode and travel protection; synthetic tracking only");$finish;
 end
 initial begin #80000000;$fatal(1,"timeout");end
endmodule
