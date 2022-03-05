transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020 {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020/MyARM_Pipelined.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020 {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020/MySPI.sv}
vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020 {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020/MyDE0_Nano.sv}

vlog -sv -work work +incdir+C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020 {C:/Users/Insomniaque/Desktop/UCL/Epl/M1-Q1/LELEC2531/homeworks/homework8/Quartus-Pipelined_2020/Quartus-Pipelined_2020/MyTestbench.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  MyTestbench

add wave *
view structure
view signals
run -all
