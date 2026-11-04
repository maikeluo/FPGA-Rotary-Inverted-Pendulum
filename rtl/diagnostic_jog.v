`timescale 1ns/1ps
// ACG720 manual diagnostic drive. No self-start, timed latch or auto reversal.
// A held key gives one bounded pulse. Physical release/direction change removes
// enable without waiting for the button debounce or the next 1ms control tick.
// Re-arm requires BOTH buttons released for REARM_TICKS consecutive samples.
module diagnostic_jog #(
 parameter integer START_DUTY=80, MAX_DUTY=200, RAMP_STEP=3,
 parameter integer MAX_TICKS=500, NO_MOTION_TICKS=200,
 parameter integer TRAVEL_COUNTS=64, MIN_MOTION_COUNTS=4,
 parameter integer SPEED_Q8=512, REARM_TICKS=20
)(input clk,input rst_n,input tick,input [1:0] keys,input [1:0] raw_keys,
 input healthy,input signed [31:0] position,input signed [31:0] velocity_q8,
 output enable,output signed [15:0] command,
 output reg signed [31:0] origin,output reg [15:0] elapsed,
 output reg [7:0] fault,output [7:0] status,output reg launch_pulse);
 // Invalid parameter overrides fall back to the documented bounded defaults.
 localparam integer DUTY_CAP=(MAX_DUTY>=80 && MAX_DUTY<=200)?MAX_DUTY:200;
 localparam integer DUTY_START=(START_DUTY>=1 && START_DUTY<=DUTY_CAP)?START_DUTY:80;
 localparam integer DUTY_STEP=(RAMP_STEP>=1 && RAMP_STEP<=10)?RAMP_STEP:3;
 localparam integer TIME_CAP=(MAX_TICKS>=1 && MAX_TICKS<=500)?MAX_TICKS:500;
 localparam integer MOTION_TIME=(NO_MOTION_TICKS>=1 && NO_MOTION_TICKS<=TIME_CAP)?NO_MOTION_TICKS:TIME_CAP;
 localparam integer TRAVEL_CAP=(TRAVEL_COUNTS>=4 && TRAVEL_COUNTS<=64)?TRAVEL_COUNTS:64;
 localparam integer MOTION_MIN=(MIN_MOTION_COUNTS>=2 && MIN_MOTION_COUNTS<TRAVEL_CAP)?MIN_MOTION_COUNTS:4;
 localparam integer SPEED_CAP=(SPEED_Q8>=32 && SPEED_Q8<=512)?SPEED_Q8:512;
 localparam integer REARM_TIME=(REARM_TICKS>=1 && REARM_TICKS<=100)?REARM_TICKS:20;
 reg running,armed,reverse,motion_seen;
 reg [6:0] release_ticks;
 reg [3:0] stop_reason;
 reg [8:0] duty;
 wire [1:0] selected=reverse?2'b10:2'b01;
 wire single_key=(keys==2'b01 || keys==2'b10) && raw_keys==keys;
 wire released=keys==0 && raw_keys==0;
 wire held=keys==selected && raw_keys==selected;
 wire signed [32:0] offset=$signed({position[31],position})-$signed({origin[31],origin});
 wire travel_bad=offset>=TRAVEL_CAP || offset<=-TRAVEL_CAP;
 wire speed_bad=velocity_q8>=SPEED_CAP || velocity_q8<=-SPEED_CAP;
 wire motion_now=motion_seen || offset>=MOTION_MIN || offset<=-MOTION_MIN;
 wire no_motion=elapsed>=MOTION_TIME && !motion_now;
 wire timed_out=elapsed>=TIME_CAP;
 assign enable=rst_n && running && held && healthy && !travel_bad && !speed_bad && !no_motion && !timed_out;
 wire signed [15:0] magnitude=$signed({7'd0,duty});
 assign command=enable ? (reverse ? -magnitude:magnitude):16'sd0;
 // DIAGNOSTIC-only telemetry bits, documented separately from balance limits:
 // [6] armed [5] driving [4] >=MIN_MOTION displacement seen
 // [3] time cap [2] no movement [1] speed cap [0] travel cap.
 assign status={1'b0,armed,enable,motion_seen,stop_reason};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   running<=0;armed<=0;reverse<=0;motion_seen<=0;release_ticks<=0;
   stop_reason<=0;duty<=DUTY_START;origin<=0;elapsed<=0;fault<=0;launch_pulse<=0;
  end else begin
   launch_pulse<=0;
   if(running)begin
   armed<=0;release_ticks<=0;
   if(motion_now)motion_seen<=1;
   // Gates above already suppress the physical output in these conditions.
   if(!healthy)begin running<=0;fault<=8'h01;end
   else if(travel_bad)begin running<=0;fault<=8'h02;stop_reason<=4'b0001;end
   else if(speed_bad)begin running<=0;fault<=8'h02;stop_reason<=4'b0010;end
   else if(no_motion)begin running<=0;fault<=8'h20;stop_reason<=4'b0100;end
   else if(timed_out)begin running<=0;stop_reason<=4'b1000;end
   else if(!held)running<=0;
   else if(tick)begin
    if(elapsed<TIME_CAP)elapsed<=elapsed+1'b1;
    if(duty+DUTY_STEP<DUTY_CAP)duty<=duty+DUTY_STEP;
    else duty<=DUTY_CAP;
   end
  end else begin
   // A health fault while a key remains held never restarts an interrupted jog.
   if(!released)release_ticks<=0;
   else if(tick)begin
    if(release_ticks<REARM_TIME)release_ticks<=release_ticks+1'b1;
    if(release_ticks>=REARM_TIME-1)armed<=1;
   end
   if(armed && tick && single_key)begin
    armed<=0;release_ticks<=0;reverse<=keys[1];origin<=position;
    elapsed<=0;duty<=DUTY_START;motion_seen<=0;stop_reason<=0;
    if(healthy && !speed_bad)begin running<=1;fault<=0;launch_pulse<=1;end
    else begin running<=0;fault<=healthy ? 8'h02:8'h01;stop_reason<=healthy ? 4'b0010:4'b0000;end
   end else if(armed && (raw_keys==2'b11 || keys==2'b11))begin
    // Both keys pressed cancel the arm; releasing only one cannot start motion.
    armed<=0;release_ticks<=0;
   end
  end
  end
 end
endmodule
