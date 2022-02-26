onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -radix unsigned /sonar_FSM_testbench/distance
add wave -noupdate /sonar_FSM_testbench/clock
add wave -noupdate /sonar_FSM_testbench/reset
add wave -noupdate -radix unsigned /sonar_FSM_testbench/Flags
add wave -noupdate /sonar_FSM_testbench/Trigger
add wave -noupdate /sonar_FSM_testbench/Echo
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/clk
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/reset
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/Trigger
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/Echo
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/distance
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/Flags
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/state
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/nextstate
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/reset1
add wave -noupdate /sonar_FSM_testbench/sonar_FSM_01/overflow
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1000
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits us
update
WaveRestoreZoom {0 ps} {1024 ns}
