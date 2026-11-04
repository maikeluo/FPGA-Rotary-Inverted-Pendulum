`timescale 1ns/1ps
// ACG720 only. A DOWN request is not a measurement while the button is held.
// Release S3, then require one complete quiet ADC window before requesting
// angle_sensor's existing 64-sample average. No automatic rewriting of UP/DOWN.
module acg720_cal_guard #(
 parameter integer DOWN_STABLE_TICKS=1000, DOWN_MAX_SPREAD=6,
 parameter integer START_STABLE_TICKS=200, START_DOWN_ANGLE=165000,
 parameter integer START_DOWN_RATE=30
)(input clk,rst_n,tick,idle,stop_request,up_request,down_request,down_key_held,
 input adc_ok,calibrated,calibrating,position_still,
 input [9:0] raw,
 input signed [31:0] angle,rate,
 input start_request,
 output reg down_cal_pulse,output reg down_pending,output reg wait_release,
 output start_allowed,output reg start_blocked,output down_ready);
 reg [15:0] down_count,start_count;
 reg [9:0] low_raw,high_raw;
 wire [9:0] next_low=(raw<low_raw)?raw:low_raw;
 wire [9:0] next_high=(raw>high_raw)?raw:high_raw;
 wire [10:0] spread={1'b0,next_high}-{1'b0,next_low};
 wire [31:0] abs_angle=angle<0?-angle:angle;
 wire [31:0] abs_rate=rate<0?-rate:rate;
 wire quiet_down=abs_angle>=START_DOWN_ANGLE && abs_angle<=180000 &&
   abs_rate<START_DOWN_RATE && position_still;
 wire sample_good=adc_ok && calibrated && !calibrating && !down_pending &&
   !down_cal_pulse && !up_request && !down_request;
 wire near_upright=abs_angle<8000 && abs_rate<200;
 assign down_ready=start_count>=START_STABLE_TICKS && quiet_down && sample_good;
 assign start_allowed=idle && !stop_request && sample_good && (near_upright || down_ready);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   down_cal_pulse<=0;down_pending<=0;wait_release<=0;
   down_count<=0;low_raw<=0;high_raw<=0;start_count<=0;start_blocked<=0;
  end else begin
   down_cal_pulse<=0;
   // STOP and a new UP request cancel a pending DOWN measurement immediately.
   if(!idle || stop_request || up_request)begin
    down_pending<=0;wait_release<=0;down_count<=0;
   end else if(down_request && !calibrating)begin
    down_pending<=1;wait_release<=1;down_count<=0;low_raw<=raw;high_raw<=raw;
   end else if(down_pending)begin
    if(down_key_held)begin wait_release<=1;down_count<=0;end
    else begin
     wait_release<=0;
     if(tick)begin
      if(!adc_ok || !position_still || calibrating)down_count<=0;
      else if(down_count==0 || spread>DOWN_MAX_SPREAD)begin
       low_raw<=raw;high_raw<=raw;down_count<=1;
      end else begin
       low_raw<=next_low;high_raw<=next_high;
       if(down_count>=DOWN_STABLE_TICKS-1)begin
        down_cal_pulse<=1;down_pending<=0;wait_release<=0;down_count<=0;
       end else down_count<=down_count+1'b1;
      end
     end
    end
   end
   // This qualifies a requested launch, never starts the motor by itself.
   if(!idle || stop_request || !sample_good || !quiet_down)start_count<=0;
   else if(tick && start_count<START_STABLE_TICKS)start_count<=start_count+1'b1;
   if(stop_request || up_request || down_request || !idle)start_blocked<=0;
   else if(start_request)start_blocked<=!start_allowed;
  end
 end
endmodule
