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

set ::env(DESIGN_NAME) chaos_subarray

set ::env(VERILOG_FILES) "\
	$::env(CARAVEL_ROOT)/verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/chaos_subarray.v"

set ::env(DESIGN_IS_CORE) 0

set ::env(CLOCK_PORT) "iclk"
set ::env(CLOCK_NET) "counter.clk"
set ::env(CLOCK_PERIOD) "25"
# 25 ns is 40 MHz
# 100 ns is 10 MHz

set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 900 600"
# "0 0 3006 3596"

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(PL_BASIC_PLACEMENT) 0
set ::env(PL_TARGET_DENSITY) 0.48

set ::env(SYNTH_EXTRA_MAPPING_FILE) $script_dir/yosys_mapping.v
set ::env(SYNTH_MAX_FANOUT) 12
set ::env(BASE_SDC_FILE) $script_dir/base.sdc

# Maximum layer used for routing is metal 4.
# This is because this macro will be inserted in a top level (user_project_wrapper) 
# where the PDN is planned on metal 5. So, to avoid having shorts between routes
# in this macro and the top level metal 5 stripes, we have to restrict routes to metal4.   
set ::env(RT_MAX_LAYER) {met4}

# You can draw more power domains if you need to 
set ::env(VDD_NETS) [list {vccd1}]
set ::env(GND_NETS) [list {vssd1}]

set ::env(DIODE_INSERTION_STRATEGY) 4 
# If you're going to use multiple power domains, then disable cvc run.
set ::env(RUN_CVC) 1

# Number of threads to be used during routing processes
set ::env(ROUTING_CORES) 16 

# Designation of the subarray as a macro/core
set ::env(DESIGN_IS_CORE) 0
set ::env(FP_PDN_CORE_RING) 0
set ::env(VDD_NETS) [list {vccd1}]
set ::env(GND_NETS) [list {vssd1}]
set ::env(FP_PDN_VWIDTH) 25