onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /HDB3_vlg_tst/ena
add wave -noupdate /HDB3_vlg_tst/rst
add wave -noupdate /HDB3_vlg_tst/clk
add wave -noupdate /HDB3_vlg_tst/clk_256
add wave -noupdate /HDB3_vlg_tst/clk_16
add wave -noupdate /HDB3_vlg_tst/outm
add wave -noupdate /HDB3_vlg_tst/outv
add wave -noupdate /HDB3_vlg_tst/outb
add wave -noupdate /HDB3_vlg_tst/outP
add wave -noupdate /HDB3_vlg_tst/outN
add wave -noupdate /HDB3_vlg_tst/data_out
add wave -noupdate /HDB3_vlg_tst/outdata_P
add wave -noupdate /HDB3_vlg_tst/outdata_N
add wave -noupdate /HDB3_vlg_tst/outdata_v
add wave -noupdate /HDB3_vlg_tst/clk_recover
add wave -noupdate /HDB3_vlg_tst/finallyout
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {52 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {1048576 ns}
