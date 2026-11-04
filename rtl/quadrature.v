`timescale 1ns/1ps
// x4 decoding: 13 lines * 20 gear ratio * 4 = 1040 counts/output revolution.
// Positive sequence: 00 -> 10 -> 11 -> 01 -> 00 (A leads B).
module quadrature #(parameter integer REVERSE=0, FILTER_CYCLES=4)(
  input clk,input rst_n,input a,input b,input zero,
  output reg signed [31:0] position,output reg [31:0] errors);
  (* ASYNC_REG="TRUE" *) reg [1:0] meta,sync;
  reg [1:0] candidate,ab,previous;reg [7:0] stable;
  reg primed;reg [1:0] sync_fill;
  reg signed [2:0] step;
  always @* begin
    step=0;
    case({previous,ab})
      4'b0010,4'b1011,4'b1101,4'b0100:step=1;
      4'b0001,4'b0111,4'b1110,4'b1000:step=-1;
      default:step=0;
    endcase
  end
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin meta<=0;sync<=0;candidate<=0;ab<=0;previous<=0;stable<=0;position<=0;errors<=0;primed<=0;sync_fill<=0;end
    else begin
      meta<={a,b};sync<=meta;
      if(sync_fill<3)sync_fill<=sync_fill+1'b1;
      if(sync!=candidate) begin candidate<=sync;stable<=0;end
      else if(stable<FILTER_CYCLES) stable<=stable+1;
      else ab<=candidate;
      previous<=ab;
      if(zero) begin position<=0;previous<=ab;errors<=0;end
      else if(primed && previous!=ab) begin
        if(step==0) errors<=errors+1;
        else if((position==32'sh7fffffff && (REVERSE ? -step:step)>0) ||
                (position==32'sh80000000 && (REVERSE ? -step:step)<0)) errors<=errors+1;
        else position<=position+(REVERSE ? -step:step);
      end
      // r54r9: observe the real, stable power-up phase before counting.
      // Starting at physical 01/10/11 must not invent -1/+1/an error.
      if(!primed && sync_fill==3 && sync==candidate && stable>=FILTER_CYCLES)begin
        previous<=candidate;primed<=1;
      end
    end
  end
endmodule


// v53r10: same 1/8 encoder velocity filter, symmetric nearest rounding.
// Q8 counts/ms at 1kHz. Five register stages finish before angle_sensor.sample
// (six stages after the shared tick), so the controller gets this tick's value.
// Reset/recovery/encoder-zero cancels pending samples and primes position.
module encoder_velocity_filter(input clk,input rst_n,input tick,input zero,
 input recover,input signed [31:0] position,
 output reg signed [31:0] last_position,output reg signed [31:0] velocity_q8);
 reg [2:0] phase;
 reg signed [32:0] delta_s;
 reg signed [31:0] previous_s,input_q8,error_q8,step_q8;
 reg stationary_s;
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin phase<=0;last_position<=0;velocity_q8<=0;end
   else if(zero || recover)begin
     phase<=0;last_position<=zero?32'sd0:position;velocity_q8<=0;
   end else if(tick)begin
     last_position<=position;
     if(phase==0)phase<=1;
     else begin phase<=0;velocity_q8<=32'sd8192;end // fail high on impossible cadence
   end else if(phase==4)begin
     velocity_q8<=stationary_s?32'sd0:previous_s+step_q8;phase<=0;
   end else if(phase!=0)phase<=phase+1'b1;
 end
 always @(posedge clk)begin
   if(rst_n && !zero && !recover)case(phase)
     0:if(tick)begin
       delta_s<=$signed({position[31],position})-$signed({last_position[31],last_position});
       previous_s<=velocity_q8;
       stationary_s<=position==last_position && velocity_q8> -8 && velocity_q8<8;
     end
     1:begin
       // Bound before shifting. Extreme corrupt jumps still produce velocity
       // FAR beyond the inherited +/-4096 speed guard, never wrap to zero.
       if(delta_s>65535)input_q8<=16776960;
       else if(delta_s< -65535)input_q8<=-16776960;
       else input_q8<=delta_s<<<8;
     end
     2:error_q8<=input_q8-previous_s;
     3:step_q8<=error_q8>=0?((error_q8+4)>>>3):-(((-error_q8+4)>>>3));
     default:begin end
   endcase
 end
endmodule
