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
#include "verilog/dv/caravel/defs.h"
#include "verilog/dv/caravel/stub.c"

// --------------------------------------------------------

#define reg_user_config_l (*(volatile uint32_t*)0x30000000)
#define reg_user_config_h (*(volatile uint32_t*)0x30000004)

#define reg_user_address  (*(volatile uint32_t*)0x30000008)
#define reg_user_transfer (*(volatile uint32_t*)0x3000000c)

/* Configuration further refined to each LUT (16 bits per LUT) */
#define reg_user_config_N (*(volatile uint16_t*)0x30000000)
#define reg_user_config_S (*(volatile uint16_t*)0x30000002)
#define reg_user_config_E (*(volatile uint16_t*)0x30000004)
#define reg_user_config_W (*(volatile uint16_t*)0x30000006)

/*
 *
 * Chaos automaton test 2:
 *
 * Exercise cell LUT confguration reading and writing via the
 * shift register.
 *
 */

void main()
{
	uint16_t LUTdata;

	/* Set up the housekeeping SPI to be connected internally so	*/
	/* that external pin changes don't affect it.			*/

	reg_spimaster_config = 0xa002;	// Enable, prescaler = 2,
                                        // connect to housekeeping SPI

	// Connect the housekeeping SPI to the SPI master
	// so that the CSB line is not left floating.  This allows
	// all of the GPIO pins to be used for user functions.

	// The upper GPIO pins are configured to be output
	// and accessble to the management SoC.
	// Used to flad the start/end of a test 
	// The lower GPIO pins are configured to be output
	// and accessible to the user project.  They show
	// the project count value, although this test is
	// designed to read the project count through the
	// logic analyzer probes.
	// I/O 6 is configured for the UART Tx line

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
	// I/O 6 left out on purpose---disruptive to the simulation
        reg_mprj_io_5  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_4  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_3  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_2  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_1  = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_0  = GPIO_MODE_MGMT_STD_OUTPUT;

        reg_mprj_io_6  = GPIO_MODE_MGMT_STD_OUTPUT;

	// Set UART clock to 64 kbaud (enable before I/O configuration)
	reg_uart_clkdiv = 625;
	reg_uart_enable = 1;

        /* Apply configuration */
        reg_mprj_xfer = 1;
        while (reg_mprj_xfer == 1);

	// Flag start of the test 
	reg_mprj_datal = 0xAB400000;

	// Apply (arbitrary) address (range 0 to 199)
	reg_user_address = 50;

	// Test part 1:
	// Cycle register to load position
	reg_user_transfer = 1;

	// Wait for cycle to finish
        while (reg_user_transfer != 0);

	// Write data (64 bits, arbitrary value, written per indivitual LUT)
	reg_user_config_N = 0x1234;
	reg_user_config_S = 0x5678;
	reg_user_config_E = 0x9abc;
	reg_user_config_W = 0xdef0;
 
	// Cycle register to final position
	reg_user_transfer = 2;

	// Wait for cycle to finish
        while (reg_user_transfer != 0);

	// Flag end of test part 1
	reg_mprj_datal = 0xAB410000;

	// Test part 2:
	// Cycle register to load position
	reg_user_transfer = 1;
	
	// Wait for cycle to finish
        while (reg_user_transfer != 0);

	// Read data (64 bits, arbitrary value, written per indivitual LUT)
	LUTdata = reg_user_config_N;
	reg_mprj_datal = 0xAB420000 | LUTdata;
	LUTdata = reg_user_config_S;
	reg_mprj_datal = 0xAB420000 | LUTdata;
	LUTdata = reg_user_config_E;
	reg_mprj_datal = 0xAB420000 | LUTdata;
	LUTdata = reg_user_config_W;
	reg_mprj_datal = 0xAB420000 | LUTdata;

	// Cycle register to final position
	reg_user_transfer = 2;

	// Wait for cycle to finish
        while (reg_user_transfer != 0);

	// Flag end of test
	reg_mprj_datal = 0xAB510000;
	// The following makes the simulation very long.  Moved after end-of-test
	// is flagged so that simulation ends first.
	print("\nMonitor: Test 2 Passed\n\n");
}

