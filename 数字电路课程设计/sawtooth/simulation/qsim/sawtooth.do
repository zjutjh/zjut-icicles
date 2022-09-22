onerror {exit -code 1}
vlib work
vlog -work work sawtooth.vo
vlog -work work CNT5.vwf.vt
vsim -novopt -c -t 1ps -L cycloneive_ver -L altera_ver -L altera_mf_ver -L 220model_ver -L sgate_ver -L altera_lnsim_ver work.CNT5_vlg_vec_tst -voptargs="+acc"
vcd file -direction sawtooth.msim.vcd
vcd add -internal CNT5_vlg_vec_tst/*
vcd add -internal CNT5_vlg_vec_tst/i1/*
run -all
quit -f
