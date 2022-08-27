/*
 * SPDX-FileCopyrightText: 2020 Efabless Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: Apache-2.0
 */

// This include is relative to $CARAVEL_PATH (see Makefile)
// #include "verilog/dv/caravel/defs.h"
// #include "verilog/dv/caravel/stub.c"

#include "defs.h"

// --------------------------------------------------------

#define reg_user_config_0 (*(volatile uint8_t*)0x30000000)

#define reg_user_config_l (*(volatile uint32_t*)0x30000000)
#define reg_user_config_h (*(volatile uint32_t*)0x30000004)

#define reg_user_address  (*(volatile uint32_t*)0x30000008)
#define reg_user_transfer (*(volatile uint32_t*)0x3000000c)

#define reg_user_direct (*(volatile uint32_t*)0x30000010)
#define reg_user_source (*(volatile uint32_t*)0x30000014)
#define reg_user_data0  (*(volatile uint32_t*)0x30000018)
#define reg_user_data1  (*(volatile uint32_t*)0x3000001c)
#define reg_user_data2  (*(volatile uint32_t*)0x30000020)
#define reg_user_data3  (*(volatile uint32_t*)0x30000024)
#define reg_user_data4  (*(volatile uint32_t*)0x30000028)

/* Configuration further refined to each LUT (16 bits per LUT) */
#define reg_user_config_N (*(volatile uint16_t*)0x30000000)
#define reg_user_config_S (*(volatile uint16_t*)0x30000002)
#define reg_user_config_E (*(volatile uint16_t*)0x30000004)
#define reg_user_config_W (*(volatile uint16_t*)0x30000006)

/*
 *
 * Chaos automaton test 3:
 *
 * Check cell-to-cell connections by programming all the
 * LUTs to be straight-through buffers east to west.
 *
 */

void main()
{
	uint16_t LUTdata;
	int i;

	// The upper GPIO pins are configured to be output
	// and accessble to the management SoC.
	// Used to flad the start/end of a test 
	// The lower GPIO pins are configured to be output
	// and accessible to the user project.  They show
	// the project count value, although this test is
	// designed to read the project count through the
	// logic analyzer probes.

        reg_mprj_io_31 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_30 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_29 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_28 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_27 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_26 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_25 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_24 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_23 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_22 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_21 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_20 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_19 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_18 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_17 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_16 = GPIO_MODE_MGMT_STD_OUTPUT;

        reg_mprj_io_15 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_14 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_13 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_12 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_11 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_10 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_9  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_8  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_7  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_6  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_5  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_4  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_3  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_2  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_1  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_0  = GPIO_MODE_MGMT_STD_OUTPUT;

        /* Apply configuration */
        reg_mprj_xfer = 1;
        while (reg_mprj_xfer == 1);

        /* Enable wishbone signaling in the user project */
        reg_wb_enable = 1;

	// Flag start of the test 
	reg_mprj_datal = 0xAB400000;

	// Apply input values first, so that the bits are valid
	// when the load process toggles "hold" on all cells.

	// Apply a reset to all cells (auto-clearing bit)
	reg_user_transfer = 4;

	// Apply zero bits to all inputs
 	reg_user_data0 = 0;
 	reg_user_data1 = 0;
 	reg_user_data2 = 0;
 	reg_user_data3 = 0;
 	reg_user_data4 = 0;

	// Apply address 0 (meaning shift by 1 address position each time)
	reg_user_address = 0;

	// Test part 1:

	// Write data (64 bits, buffer each direction)
	// Same data for all cells;  does not need to be inside the loop.
	reg_user_config_N = 0xff00;
	reg_user_config_S = 0xff00;
	reg_user_config_W = 0xff00;
	reg_user_config_E = 0xff00;

	for (i = 0; i < 1500; i++)
	{
	    // Cycle register to load position
	    reg_user_transfer = 1;

	    // Wait for cycle to finish
            while (reg_user_transfer != 0);

	    // Register is full---just need to apply 1 byte to trigger write.
	    reg_user_config_0 = 0x00;
	}
 
	// Cycle register to final position
	reg_user_transfer = 2;

	// Wait for cycle to finish
        while (reg_user_transfer != 0);

	// Flag end of test part 1
	reg_mprj_datal = 0xAB410000;

	// Test part 2:
	// Read bits from all outputs (should be zero)---apply to GPIO out
	reg_mprj_datal = reg_user_data0;
	reg_mprj_datal = reg_user_data1;
	reg_mprj_datal = reg_user_data2;
	reg_mprj_datal = reg_user_data3;
	reg_mprj_datal = reg_user_data4;

	// Apply one bits to all inputs (with logic analyzer)
 	reg_user_data0 = -1;
 	reg_user_data1 = -1;
 	reg_user_data2 = -1;
 	reg_user_data3 = -1;
 	reg_user_data4 = -1;

	// Read bits from all outputs (should be one)---apply to GPIO out
	reg_mprj_datal = reg_user_data0;
	reg_mprj_datal = reg_user_data1;
	reg_mprj_datal = reg_user_data2;
	reg_mprj_datal = reg_user_data3;
	reg_mprj_datal = reg_user_data4;
	
	// Flag end of test
	reg_mprj_datal = 0xAB510000;

	// Loop forever
	while (1) {};
}

