transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/sonar_FSM.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/full_adder5.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/full_adder14.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/sonar.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/full_adder8.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/full_adder_1_bit.sv}

vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet\ eleme/sensors_blocks {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q2/projet eleme/sensors_blocks/sonar_FSM_testbench.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  sonar_FSM_testbench

add wave *
view structure
view signals
run 10 us