set here [file dirname [file normalize [info script]]]
open_project [file join $here acg720_top.gprj]
set_option -top_module acg720_top
run all
