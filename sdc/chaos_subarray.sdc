###############################################################################
# Created by write_sdc
# Thu Aug  4 17:05:43 2022
###############################################################################
current_design chaos_subarray
###############################################################################
# Timing Constraints
###############################################################################
create_clock -name iclk -period 25.0000 [get_ports {iclk}]
set_clock_transition 0.1500 [get_clocks {iclk}]
set_clock_uncertainty 0.2500 iclk
set_propagated_clock [get_clocks {iclk}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {hold}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {idata}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[0]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[1]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[2]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[3]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[4]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[5]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[6]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[7]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[8]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {ieast[9]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[0]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[1]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[2]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[3]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[4]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[5]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[6]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[7]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[8]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {inorth[9]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[0]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[1]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[2]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[3]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[4]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[5]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[6]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[7]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[8]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {isouth[9]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[0]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[1]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[2]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[3]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[4]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[5]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[6]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[7]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[8]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {iwest[9]}]
set_input_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {reset}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oclk}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {odata}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[0]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[1]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[2]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[3]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[4]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[5]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[6]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[7]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[8]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {oeast[9]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[0]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[1]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[2]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[3]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[4]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[5]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[6]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[7]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[8]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {onorth[9]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[0]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[1]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[2]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[3]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[4]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[5]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[6]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[7]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[8]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {osouth[9]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[0]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[1]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[2]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[3]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[4]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[5]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[6]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[7]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[8]}]
set_output_delay 5.0000 -clock [get_clocks {iclk}] -add_delay [get_ports {owest[9]}]
###############################################################################
# Environment
###############################################################################
set_load -pin_load 0.0334 [get_ports {oclk}]
set_load -pin_load 0.0334 [get_ports {odata}]
set_load -pin_load 0.0334 [get_ports {oeast[9]}]
set_load -pin_load 0.0334 [get_ports {oeast[8]}]
set_load -pin_load 0.0334 [get_ports {oeast[7]}]
set_load -pin_load 0.0334 [get_ports {oeast[6]}]
set_load -pin_load 0.0334 [get_ports {oeast[5]}]
set_load -pin_load 0.0334 [get_ports {oeast[4]}]
set_load -pin_load 0.0334 [get_ports {oeast[3]}]
set_load -pin_load 0.0334 [get_ports {oeast[2]}]
set_load -pin_load 0.0334 [get_ports {oeast[1]}]
set_load -pin_load 0.0334 [get_ports {oeast[0]}]
set_load -pin_load 0.0334 [get_ports {onorth[9]}]
set_load -pin_load 0.0334 [get_ports {onorth[8]}]
set_load -pin_load 0.0334 [get_ports {onorth[7]}]
set_load -pin_load 0.0334 [get_ports {onorth[6]}]
set_load -pin_load 0.0334 [get_ports {onorth[5]}]
set_load -pin_load 0.0334 [get_ports {onorth[4]}]
set_load -pin_load 0.0334 [get_ports {onorth[3]}]
set_load -pin_load 0.0334 [get_ports {onorth[2]}]
set_load -pin_load 0.0334 [get_ports {onorth[1]}]
set_load -pin_load 0.0334 [get_ports {onorth[0]}]
set_load -pin_load 0.0334 [get_ports {osouth[9]}]
set_load -pin_load 0.0334 [get_ports {osouth[8]}]
set_load -pin_load 0.0334 [get_ports {osouth[7]}]
set_load -pin_load 0.0334 [get_ports {osouth[6]}]
set_load -pin_load 0.0334 [get_ports {osouth[5]}]
set_load -pin_load 0.0334 [get_ports {osouth[4]}]
set_load -pin_load 0.0334 [get_ports {osouth[3]}]
set_load -pin_load 0.0334 [get_ports {osouth[2]}]
set_load -pin_load 0.0334 [get_ports {osouth[1]}]
set_load -pin_load 0.0334 [get_ports {osouth[0]}]
set_load -pin_load 0.0334 [get_ports {owest[9]}]
set_load -pin_load 0.0334 [get_ports {owest[8]}]
set_load -pin_load 0.0334 [get_ports {owest[7]}]
set_load -pin_load 0.0334 [get_ports {owest[6]}]
set_load -pin_load 0.0334 [get_ports {owest[5]}]
set_load -pin_load 0.0334 [get_ports {owest[4]}]
set_load -pin_load 0.0334 [get_ports {owest[3]}]
set_load -pin_load 0.0334 [get_ports {owest[2]}]
set_load -pin_load 0.0334 [get_ports {owest[1]}]
set_load -pin_load 0.0334 [get_ports {owest[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {hold}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iclk}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {idata}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {reset}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {ieast[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {inorth[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {isouth[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin {Y} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {iwest[0]}]
set_timing_derate -early 0.9500
set_timing_derate -late 1.0500
###############################################################################
# Design Rules
###############################################################################
set_max_fanout 12.0000 [current_design]
