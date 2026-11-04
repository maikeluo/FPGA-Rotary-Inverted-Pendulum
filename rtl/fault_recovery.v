`timescale 1ns/1ps
// Manual recovery at the 1 ms control tick. No automatic re-arm or motion.
// v40: SINGLE-BUTTON recovery. SW2 (stop) alone, held for 2 s, clears the fault latch
// and re-references the travel origin. Holding SW1+SW2 still works because that also
// asserts SW2. Previously SW1+SW2 had to be pressed together for 1 s, which is awkward
// on the bench and was a common source of "the button does nothing" confusion.
module fault_recovery #(
 parameter integer HOLD_TICKS=2000, STABLE_TICKS=200,
 parameter integer MIN_RAW=16, MAX_RAW=1007, MAX_SPREAD=8
)(input clk,input rst_n,input tick,input [1:0] keys,input allow_clear,
 input adc_valid,input adc_ready,input adc_timeout,input adc_range_bad,
 input [9:0] raw,output reg clear_fault,output reg locked);
 reg [31:0] hold_count,stable_count;
 reg [9:0] lo,hi;
 reg tracking,ack_sent;
 wire in_range=raw>=MIN_RAW && raw<=MAX_RAW;
 wire within_spread=!(raw>hi && raw-lo>MAX_SPREAD) &&
                    !(raw<lo && hi-raw>MAX_SPREAD);
 wire clean=adc_ready && !adc_timeout && !adc_range_bad && in_range;
 wire stable=clean && tracking && within_spread && stable_count>=STABLE_TICKS;
 wire hold_key=keys[1];   // v40: SW2 alone is the recovery button
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     hold_count<=0;stable_count<=0;lo<=0;hi<=0;tracking<=0;
     clear_fault<=0;locked<=0;ack_sent<=0;
   end else begin
     clear_fault<=0;
     if(hold_key) locked<=1;
     // Releasing the button re-arms the next recovery.
     else begin locked<=0;ack_sent<=0;end
     if(!hold_key) hold_count<=0;
     else if(tick) begin
       if(hold_count<HOLD_TICKS) hold_count<=hold_count+1;
       if(hold_count>=HOLD_TICKS-1 && stable && allow_clear && !ack_sent) begin
         clear_fault<=1;ack_sent<=1;
       end
     end

     // The live OTR qualifier resets this timer even BETWEEN control ticks.
     if(!clean) begin stable_count<=0;tracking<=0;end
     else if(adc_valid) begin
       if(!tracking || !within_spread) begin
         lo<=raw;hi<=raw;tracking<=1;stable_count<=0;
       end else begin
         if(raw<lo)lo<=raw;
         if(raw>hi)hi<=raw;
         if(tick && stable_count<STABLE_TICKS)stable_count<=stable_count+1;
       end
     end else if(tick && tracking && stable_count<STABLE_TICKS)
       stable_count<=stable_count+1;
   end
 end
endmodule
