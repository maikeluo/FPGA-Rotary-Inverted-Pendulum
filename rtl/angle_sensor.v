`timescale 1ns/1ps
// Two-point calibration: hold upright for SW3, down for SW4. 64 ms averages.
// Q12 scale maps signed (down-up) to +180000 millidegrees. No assumption that
// the analogue front end is 0..2V proportional to 0..360 degrees.
// v53r3: a validated pair survives the external SW5 reset in FPGA registers.
// These registers are initialized on FPGA configuration, NOT nonvolatile writes.
// Optional boot constants must be measured on this particular assembly.
module angle_sensor #(
 parameter integer MIN_SPAN=64, MAX_CAL_SPREAD=24,
 parameter integer FIXED_CAL_ENABLE=0, FIXED_UP=0, FIXED_DOWN=0)(
 input clk,input rst_n,input tick,input [9:0] raw,input adc_ready,input adc_bad,
 input cal_up,input cal_down,input allow_cal,
 output reg calibrated,output reg calibrating,output reg zero_encoder,
 output reg [9:0] upright_raw,down_raw,output reg [7:0] status,
 output reg signed [31:0] angle,delta,output reg sample,input clear_fault);
 localparam integer FIXED_SPAN=FIXED_DOWN-FIXED_UP;
 localparam FIXED_VALID=(FIXED_CAL_ENABLE==1) &&
   (FIXED_UP>0) && (FIXED_UP<1023) && (FIXED_DOWN>0) && (FIXED_DOWN<1023) &&
   ((FIXED_SPAN>=MIN_SPAN) || (FIXED_SPAN<=-MIN_SPAN));
 reg saved_valid=FIXED_VALID;
 reg [9:0] saved_up=FIXED_VALID ? FIXED_UP : 0;
 reg [9:0] saved_down=FIXED_VALID ? FIXED_DOWN : 0;
 // Deliberately outside rst_n. Only publish a complete, successful pair.
 // Failed/partial recalibration does not overwrite the previous good pair.
 always @(posedge clk) begin
   if(rst_n && calibrated && !calibrating && status==0 && !cal_up && !cal_down) begin
     saved_up<=upright_raw;saved_down<=down_raw;saved_valid<=1;
   end
 end
 reg have_up,have_down,which,scale_negative,div_start;
 reg [5:0] n;reg [15:0] sum;reg [9:0] lo,hi;
 reg [31:0] denominator,scale;wire div_busy,div_done;wire [31:0] quotient;
 reg recompute,prime,restore_pending;
 // A control tick snapshots the ADC code. Seven 50 MHz stages (140 ns)
 // separate subtraction, multiplication, wrapping and velocity filtering.
 // This avoids a single ADC-to-angle/filter combinational timing path.
 reg [2:0] sample_stage;
 reg signed [10:0] sample_offset;
 reg [9:0] sample_raw;
 reg sample_invalid,sample_jump;
 reg signed [31:0] last_angle,difference,unwrapped;
 reg signed [63:0] product,converted;
 reg signed [31:0] angle_work,difference_work;
 reg signed [31:0] history0,history1,history2,history3;
 reg signed [31:0] rate_q8,rate_work,rate_next,rounded_rate;
 reg [9:0] last_raw;reg [15:0] averaged;
 integer span,spread,raw_jump;
 udiv32 divider(clk,rst_n,div_start,32'd737280000,denominator,div_busy,div_done,quotient);
 always @* begin
   converted=product>>>12;
   if(scale_negative) converted=-converted;
   unwrapped=converted[31:0];
   if(converted>=180000) unwrapped=converted-360000;
   else if(converted< -180000) unwrapped=converted+360000;
   difference=difference_work;
   if(difference>180000) difference=difference-360000;
   else if(difference< -180000) difference=difference+360000;
   // Four-sample angular displacement / 4 ms, then the existing 1/4 IIR.
   // Q8 preserves fractional mdeg/tick; signed rounding removes the old
   // negative residual at rest. Steady angular velocity keeps the same units.
   rate_next=rate_q8+(((difference_work<<<6)-rate_q8)>>>2);
   if(rate_work>=0)rounded_rate=(rate_work+128)>>>8;
   else rounded_rate=-((-rate_work+128)>>>8);
   span=$signed({1'b0,down_raw})-$signed({1'b0,upright_raw});
   raw_jump=(raw>last_raw) ? raw-last_raw:last_raw-raw;
   spread=hi-lo;
   averaged=(sum+{6'b0,raw})>>6;
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     calibrated<=0;calibrating<=0;zero_encoder<=0;upright_raw<=0;down_raw<=0;
     status<=0;angle<=0;delta<=0;sample<=0;have_up<=0;have_down<=0;which<=0;
     scale_negative<=0;div_start<=0;n<=0;sum<=0;lo<=1023;hi<=0;
     denominator<=1;scale<=0;recompute<=0;prime<=1;restore_pending<=1;last_angle<=0;last_raw<=0;
     sample_stage<=0;sample_offset<=0;sample_raw<=0;sample_invalid<=0;sample_jump<=0;
     product<=0;angle_work<=0;difference_work<=0;
     history0<=0;history1<=0;history2<=0;history3<=0;
     rate_q8<=0;rate_work<=0;
   end else if(restore_pending) begin
     restore_pending<=0;
     if(saved_valid) begin
       upright_raw<=saved_up;down_raw<=saved_down;have_up<=1;have_down<=1;
       recompute<=1;
     end
   end else if(clear_fault && !calibrating && !div_busy && !div_start && !recompute && !div_done) begin
     // Runtime recovery preserves both measured calibration points and scale.
     // Cancel old pipeline data and re-prime the derivative at the new position.
     status<=0;delta<=0;prime<=1;sample_stage<=0;sample<=0;
     zero_encoder<=0;div_start<=0;last_raw<=raw;
     sample_invalid<=0;sample_jump<=0;
     rate_q8<=0;rate_work<=0;
   end else begin
     zero_encoder<=0;sample<=0;div_start<=0;
     if(allow_cal && (cal_up ^ cal_down) && !calibrating && !div_busy && !recompute) begin
       calibrating<=1;calibrated<=0;which<=cal_down;n<=0;sum<=0;lo<=1023;hi<=0;status<=0;prime<=1;
       if(cal_down) have_down<=0;else have_up<=0;
     end
     if(calibrating && tick) begin
       if(!adc_ready || adc_bad || raw==0 || raw==1023) begin calibrating<=0;status<=8'h01;end
       else begin
         if(raw<lo) lo<=raw;if(raw>hi) hi<=raw;
         if(n==63) begin
           calibrating<=0;
           if(spread>MAX_CAL_SPREAD || (raw>hi && raw-lo>MAX_CAL_SPREAD) ||
              (raw<lo && hi-raw>MAX_CAL_SPREAD)) status<=8'h02;
           else begin
             if(which) begin down_raw<=averaged[9:0];have_down<=1;zero_encoder<=1;end
             else begin upright_raw<=averaged[9:0];have_up<=1;end
             recompute<=1;
           end
         end else begin sum<=sum+raw;n<=n+1;end
       end
     end
     if(recompute) begin
       recompute<=0;
       if(have_up && have_down) begin
         if(span>=MIN_SPAN || span<=-MIN_SPAN) begin
           denominator<=(span<0)?-span:span;scale_negative<=span<0;div_start<=1;
         end else status<=8'h04;
       end
     end
     if(div_done) begin scale<=quotient;calibrated<=1;status<=0;prime<=1;end
     // Recalibration cancels any in-flight conversion before it can publish.
     if(!calibrated || calibrating || cal_up || cal_down) sample_stage<=0;
     else case(sample_stage)
       0: if(tick) begin
         sample_offset<=$signed({1'b0,raw})-$signed({1'b0,upright_raw});
         sample_raw<=raw;
         sample_invalid<=adc_bad || !adc_ready || raw==0 || raw==1023;
         sample_jump<=!prime && raw_jump>96;
         sample_stage<=1;
       end
       1: begin
         product<=sample_offset*$signed({1'b0,scale});sample_stage<=2;
       end
       2: begin angle_work<=unwrapped;sample_stage<=3;end
       3: begin difference_work<=angle_work-history3;sample_stage<=4;end
       4: begin difference_work<=difference;sample_stage<=5;end
       5: begin rate_work<=prime ? 32'sd0:rate_next;sample_stage<=6;end
       6: begin
         sample<=1;last_raw<=sample_raw;last_angle<=angle_work;angle<=angle_work;prime<=0;
         rate_q8<=rate_work;delta<=rounded_rate;
         if(prime)begin history0<=angle_work;history1<=angle_work;history2<=angle_work;history3<=angle_work;end
         else begin history0<=angle_work;history1<=history0;history2<=history1;history3<=history2;end
         // Jump checks still reject dead-region transitions; invalid data is
         // never reinterpreted as a wrap of a 360-degree ADC sensor.
         if(sample_invalid || adc_bad || !adc_ready) status<=8'h01;
         else if(converted>360000 || converted< -360000 || sample_jump) status<=8'h08;
         sample_stage<=0;
       end
       default: sample_stage<=0;
     endcase
   end
 end
endmodule
