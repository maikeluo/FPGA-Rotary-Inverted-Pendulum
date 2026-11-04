create_clock -name sys_clk -period 20.000 [get_ports {clk_50m}]
// Continuous 8.333 MHz SCLK, 60 ns low/high. DOUT launches on falling SCLK.
create_generated_clock -name adc_sclk_clock -source [get_ports {clk_50m}] -divide_by 6 [get_ports {adc_sclk}]
// TI tDACC <=27 ns plus provisional 8 ns PCB/IO margin; verify on hardware.
set_input_delay -clock adc_sclk_clock -clock_fall -max 35.000 [get_ports {adc_dout}]
// Conservative minimum: no guaranteed minimum DOUT/PCB delay assumed.
set_input_delay -clock adc_sclk_clock -clock_fall -min 0.000 [get_ports {adc_dout}]
// Capture enable only at phase=3: three sys_clk periods after falling SCLK.
// Apply ONLY to DOUT input. Internal logic remains single-cycle at 50 MHz.
set_multicycle_path -setup -end 3 -from [get_ports {adc_dout}]
// The NEXT DOUT launch is 120 ns after this one. The enabled capture is at
// +60 ns, so hold must compare that capture to the next falling SCLK.
// Setup=3 gives a default hold capture at +40 ns. Source hold=1 advances
// launch by 120 ns; the 20 ns guard advances the check to the actual +60 ns.
set_multicycle_path -hold -start 1 -from [get_ports {adc_dout}]
set_clock_uncertainty -hold 20.000 -from [get_clocks {adc_sclk_clock}] -to [get_clocks {sys_clk}]
// DIN launches on SCLK falling and is sampled at the next rising edge.
set_output_delay -clock adc_sclk_clock -max 18.000 [get_ports {adc_din}]
set_output_delay -clock adc_sclk_clock -min -10.000 [get_ports {adc_din}]
// Asynchronous switches/encoder/reset feed explicit synchronizers.
set_false_path -from [get_ports {key_n[*] dip_n[*] enc1_a enc1_b reset_n}]
// CS changes only during reset/startup, then stays LOW in continuous mode.
// PWM/LED/UART are not sampled by external sys_clk synchronous receivers.
report_timing -setup -from [get_ports {adc_dout}] -max_paths 5
report_timing -hold -from [get_ports {adc_dout}] -max_paths 5
report_timing -setup -to [get_ports {adc_din}] -max_paths 5
report_timing -hold -to [get_ports {adc_din}] -max_paths 5
