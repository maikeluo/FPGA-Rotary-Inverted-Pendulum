`timescale 1ns/1ps
// 2.5 MHz forwarded ADC clock at 50 MHz system clock. Capture on falling
// edge (200 ns after launch) in the SAME clk domain, not per-bit 2FF CDC.
// Ignore the first 8 conversions (>4-cycle ADC latency); average 16 samples.
// v10 SWING-DIAGNOSTIC BUILD (not a release): AUTOCLEAR_FAULT lets the latched
// OTR fault release by itself once the input is back in range and clean, instead
// of waiting for the manual SW1+SW2 recovery.  Needed only to observe whether the
// SWING energy pump can reach CAPTURE while the rod keeps passing the sensor rail.
module adc_3pa1030 #(parameter integer HALF_DIV=10, AUTOCLEAR_FAULT=0)(
  input clk,input rst_n,input [9:0] adc_data,input adc_otr,
  output reg adc_clk,output adc_oe_n,output reg [9:0] raw,
  output reg valid,output reg ready,output reg fault,
  input clear_fault,output range_bad);
  reg [15:0] div;reg [3:0] warmup,n;reg [13:0] sum;
  // Keep the internal phase separate from the forwarded output register.
  // The generated clock on adc_clk must not label an internal feedback net
  // as a clock: all acquisition registers are clocked by clk_50m.
  (* syn_preserve=1, syn_keep=1 *) reg phase;
  reg [9:0] captured_data;reg captured_otr,captured;
  reg block_otr,raw_otr;
  // Live qualification includes every conversion in the current and previous
  // averaging blocks. A brief OTR pulse cannot disappear between control ticks.
  assign range_bad=captured_otr | block_otr | raw_otr;
  assign adc_oe_n=1'b0;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin div<=0;phase<=0;adc_clk<=0;warmup<=0;n<=0;sum<=0;raw<=0;valid<=0;ready<=0;fault<=0;captured<=0;captured_data<=0;captured_otr<=0;block_otr<=0;raw_otr<=1;end
    else begin
      valid<=0;captured<=0;
      if((clear_fault || AUTOCLEAR_FAULT) && ready && !range_bad && raw!=0 && raw!=1023) fault<=0;
      if(captured) begin
        // A newly captured fault takes priority over a simultaneous clear.
        if(captured_otr) fault<=1;
        if(n==15) begin
          raw<=(sum+{4'b0,captured_data})>>4;sum<=0;n<=0;valid<=1;ready<=1;
          raw_otr<=block_otr | captured_otr;block_otr<=0;
        end else begin sum<=sum+captured_data;n<=n+1;block_otr<=block_otr | captured_otr;end
      end
      if(div==HALF_DIV-1) begin
        div<=0;phase<=~phase;adc_clk<=~phase;
        if(phase) begin
          if(warmup<8) warmup<=warmup+1;
          else begin captured_data<=adc_data;captured_otr<=adc_otr;captured<=1;end
        end
      end else div<=div+1;
    end
  end
endmodule
