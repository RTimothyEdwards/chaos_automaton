# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

set ::env(PDK) $::env(PDK)
set ::env(STD_CELL_LIBRARY) "sky130_fd_sc_hd"

set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) user_project_wrapper

set ::env(VERILOG_FILES) "\
	$::env(CARAVEL_ROOT)/verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/user_project_wrapper.v"

# set ::env(DESIGN_IS_CORE) 0

set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_NET) "counter.clk"
set ::env(CLOCK_PERIOD) "25"
# 25 ns is 40 MHz
# 100 ns is 10 MHz

# set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(PL_BASIC_PLACEMENT) 0
set ::env(PL_TARGET_DENSITY) 0.2

set ::env(SYNTH_EXTRA_MAPPING_FILE) $script_dir/yosys_mapping.v
set ::env(SYNTH_MAX_FANOUT) 12
set ::env(BASE_SDC_FILE) $script_dir/base.sdc

# Maximum layer used for routing is metal 4.
# This is because this macro will be inserted in a top level (user_project_wrapper) 
# where the PDN is planned on metal 5. So, to avoid having shorts between routes
# in this macro and the top level metal 5 stripes, we have to restrict routes to metal4.  
set ::env(RT_MAX_LAYER) {met5}


set ::env(DIODE_INSERTION_STRATEGY) 4 

set ::env(ROUTING_CORES) 16
# Number of threads to be used during routing processes

# Internal macros
set ::env(MACRO_PLACEMENT_CFG) $script_dir/macro_placement.cfg

set ::env(VERILOG_FILES_BLACKBOX) "\
        $script_dir/../../verilog/rtl/chaos_subarray.v"

set ::env(EXTRA_LEFS) "\
        $script_dir/../../lef/chaos_subarray.lef"

set ::env(EXTRA_GDS_FILES) "\
        $script_dir/../../gds/chaos_subarray.gds"

set ::env(MAGIC_ZEROIZE_ORIGIN) 0

# Area Configurations. DON'T TOUCH.
set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 2920 3520"

set ::env(RUN_CVC) 0

set ::unit 2.4
set ::env(FP_IO_VEXTEND) [expr 2*$::unit]
set ::env(FP_IO_HEXTEND) [expr 2*$::unit]
set ::env(FP_IO_VLENGTH) $::unit
set ::env(FP_IO_HLENGTH) $::unit

set ::env(FP_IO_VTHICKNESS_MULT) 4
set ::env(FP_IO_HTHICKNESS_MULT) 4

# Power & Pin Configurations. DON'T TOUCH.
set ::env(FP_PDN_CORE_RING) 1
set ::env(FP_PDN_CORE_RING_VWIDTH) 3.1
set ::env(FP_PDN_CORE_RING_HWIDTH) 3.1
set ::env(FP_PDN_CORE_RING_VOFFSET) 12.45
set ::env(FP_PDN_CORE_RING_HOFFSET) $::env(FP_PDN_CORE_RING_VOFFSET)
set ::env(FP_PDN_CORE_RING_VSPACING) 1.7
set ::env(FP_PDN_CORE_RING_HSPACING) $::env(FP_PDN_CORE_RING_VSPACING)

set ::env(FP_PDN_VWIDTH) 3.1
set ::env(FP_PDN_HWIDTH) 3.1
set ::env(FP_PDN_VSPACING) [expr 5*$::env(FP_PDN_CORE_RING_VWIDTH)]
set ::env(FP_PDN_HSPACING) [expr 5*$::env(FP_PDN_CORE_RING_HWIDTH)]

set ::env(VDD_NETS) [list {vccd1} {vccd2} {vdda1} {vdda2}]
set ::env(GND_NETS) [list {vssd1} {vssd2} {vssa1} {vssa2}]
set ::env(SYNTH_USE_PG_PINS_DEFINES) "USE_POWER_PINS"
set ::env(FP_PDN_MACRO_HOOKS) ".* vccd1 vssd1 vccd1 vssd1"

# Pin placement template
set ::env(FP_DEF_TEMPLATE) $::env(DESIGN_DIR)/user_project_wrapper.def

set ::env(PDN_CFG) $script_dir/pdn_cfg.tcl