set here [file dirname [file normalize [info script]]]
open_project [file join $here acg720_io_test.gprj]
set_option -top_module acg720_io_test
run all
