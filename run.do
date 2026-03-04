vlib work
vlog i2ctb.v +acc
vsim -debugDB i2c_tb
add schematic -full i2c_tb
add wave -r *
run -all
