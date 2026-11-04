# From Transcript: set J280_ROOT D:/fpga/J280_v54
#                 do $J280_ROOT/sim/modelsim/run.do
# Optional selection: set TEST_TOP tb_v54_unified
if {![info exists J280_ROOT]} {error "Set J280_ROOT to the extracted project folder first"}
set root [file normalize $J280_ROOT]
if {![info exists TEST_TOP]} {set TEST_TOP tb_v54_trajectory}
set bench [file join $root tests ${TEST_TOP}.sv]
if {![file exists $bench]} {error "Testbench not found: $bench"}
set out [file join $root build modelsim]
file mkdir $out
cd $out
if {![file isdirectory work]} {vlib work}
vmap work [file join $out work]
set sources [glob [file join $root rtl *.v]]
eval vlog -sv -work work [list +incdir+[file join $root tests]] $sources [list $bench]
vsim -voptargs=+acc work.$TEST_TOP
onfinish stop
if {[info exists RECORD_WAVES] && $RECORD_WAVES} {add wave -r /*}
run -all
catch {wave zoom full}
echo "Inspect Transcript for PASS. This bench uses synthetic inputs, not measured mechanical dynamics."
