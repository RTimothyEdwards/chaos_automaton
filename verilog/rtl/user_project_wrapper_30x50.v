// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * chaos_automaton
 *
 * This chip is a pure asynchronous cellular automaton.  Each cell has
 * four inputs from N, S, E, W and generates four outputs to N, S, E, W.
 * Each output can be configured for any boolean function of the four
 * inputs (16 bits each).
 * 
 * Outputs on the periphery (or some selection thereof) are passed to the
 * chip GPIO.  Inputs may also come from the chip periphery;  choice of
 * input or output is programmable like the cell boolean function.
 * 
 * All periphery inputs and outputs may be channeled through the logic
 * analyzer to apply input to or monitor output from the array.
 * 
 * The wishbone bus may be used to program the cell functions.
 * 
 * This can be used in a loop with an evolutionary algorithm to tune the
 * chip functions to achieve a specific behavior.
 * 
 * Most of the core circuitry is straightforward.  The total number of
 * cells is parameterized, so that the largest number of cells that will
 * fit in the caravel user project space can be determined.
 *
 * Version v1:  To avoid massive amounts of wiring (e.g., 16 or 32
 * data wires + 10 address wires to every single cell), all of the
 * LUT configuration memory is stored in a (very long) serial chain
 * in a full loop.  The scan chain is 64 bits longer than the number
 * of cells and allows 64 bits to be transferred to and from the
 * wishbone bus independently of the cells.  Every cell has 64 latches
 * in addition to the 64 flops so that the scan chain can be cycled
 * without affecting ongoing operation of the automaton.
 *
 * Version v2:  The logic analyzer is replaced by a local version that
 * has the same number of bits as periphery I/O.  There are two registers
 * per signal, one for output, and one for input.  All registers update
 * simultaneously.  Every periphery input is connected to three sources,
 * XOR'd together:  A periphery output, a GPIO input, and a register.
 * Every periphery output is connected to three sinks:  A periphery
 * input, a GPIO output, and a register.  The periphery output-to-input
 * connections can be a loop-back or neighbor loop-back.
 *
 * Memory mapped address space:
 *
 *	BASE_ADR + 7 to BASE_ADR + 0:   Configuration data to read or write
 *	BASE_ADR + 11 to BASE_ADR + 8:	Core cell address for read/write
 *	BASE_ADR + 12:			Triggers
 *	BASE_ADR + 17 to BASE_ADR + 16: Per-side input configuration
 *	BASE_ADR + 18:			GPIO input and output slice selection
 *	BASE_ADR + 19:			GPIO direction
 *	BASE_ADR + ?? to BASE_ADR + 20: Operational data
 *	(BASE_ADR + 39 for 50x30 array)
 *
 * Trigger bits:
 *	bit 0:  Shift by (address) cells (64 bits).
 *	bit 1:  Finish cycle.  Return shift register to run state, toggle "hold"
 *
 * (to be done:)
 *	bit 2:  Capture data
 *	bit 3:  Apply data
 *
 * All trigger bits are self-resetting.  The trigger bit (as read) remains
 * high until the transfer has completed.  The trigger bit can be polled to
 * determine when the cycle has completed.
 *
 * The shift cycle bit can be used to load the configuration of the array
 * cell by cell.  The typical case is to set address = 1 and apply or read
 * each cell's configuration in turn.  However, it can also be used piecemeal,
 * for example, to read out a block of configurations, without having
 * to loop a full cycle for each one.  The counter tracks what the
 * current offset is, and can return to the run-state position on
 * application of bit 1, "Finish cycle".  At the end of "Finish cycle"
 * the hold bit is toggled to latch and apply any new configuration
 * data.
 *
 * Reading and writing a single cell's configuration can be accomplished
 * by a sequence of shift cycles and reads/writes.  To change the
 * configuration of a single cell:  (1) Write the cell address, (2) Apply
 * the shift cycle, (3) Write the configuration data, (4) Apply the
 * finish cycle.  To read the configuration of a single cell:  (1) Write
 * the cell address, (2) Apply the shift cycle, (3) Read the configuration
 * data, (4) Apply the finish cycle.
 *
 *
 * This version uses the chaos_subarray, which is intended to be
 * prehardened as a macro and tiled in the top level.
 *-------------------------------------------------------------
 */

// NOTE:  Uncomment the following lines for syntax checking
// `define MPRJ_IO_PADS 38
// `include "chaos_subarray.v"

/*
 *-----------------------------------------------------------------
 * User project top level 
 *-----------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter XSIZE = 30,	// Total number of cells left to right
    parameter YSIZE = 50,	// Total number of cells top to bottom
    parameter XTOP = 3,		// Number of sub-arrays left to right
    parameter YTOP = 5,		// Number of sub-arrays top to bottom
    parameter ASIZE = 11,	// Enough bits to count XSIZE * YSIZE
    parameter BASE_ADR = 32'h 3000_0000 // Wishbone base address
)(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals (unused)
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock
    input  user_clock2,

    // IRQ
    output [2:0] user_irq
);

`define IDLE	3'b000
`define START	3'b001
`define FINISH	3'b010
`define XDATAS	3'b011
`define XDATAF	3'b100
`define LOAD	3'b101

`define CONFIGL	8'h00		/* Address offset of configuration data low word */
`define CONFIGH	8'h01		/* Address offset of configuration data high word */
`define ADDRESS	8'h02		/* Address offset of cell address value */
`define XFER	8'h03		/* Address offset of transfer bits */
`define DIRECT  8'h04		/* Address offset of GPIO directions */
`define SOURCE  8'h05		/* Address offset of GPIO source selection */
`define DATATOP	8'h06		/* Address offset of start of data section */

`define MAXADDR (XSIZE * YSIZE)	/* Highest cell address plus one */

    reg clk;			/* serial clock to transfer data 	*/
    reg hold;			/* trigger to hold transferred data 	*/
    reg [2:0] xfer_state;	/* state of the data transfer		*/
    reg [1:0] xfer_ctrl;	/* Configuration transfer trigger bits	*/
    reg [63:0] config_data;	/* 64 bits to read or write configuration */
    reg local_reset;		/* Reset applied from a register	*/

    reg [ASIZE - 1:0] cell_addr;	/* Core cell to address	*/
    reg [ASIZE - 1:0] cell_offset;	/* Current offset of shift register */
    reg [ASIZE + 6:0] bit_count;	/* Full count (cell address + bits) */

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [1:0] config_sel;
    wire address_sel;
    wire xfer_sel;
    wire direct_sel;
    wire source_sel;

    // NOTE:  This should be parameterized.
    // For the 50x30 array, there are 50+50+30+30 = 160 periphery bits =
    // 5 words of 32 bits.  This is hard-coded for convenience.  If the
    // array size changes, this needs to be changed as well.  Needs to be
    // converted to a "generate" block.
    wire [4:0] data_sel;

    wire valid;
    reg ready;
    wire [3:0] iomem_we;
    wire selected;
    wire [1:0] busy;
    reg [31:0] rdata_pre;
    wire [63:0] rdata;
    reg [31:0] wbs_dat_o;
    reg [63:0] wdata;
    reg write;
    wire all_cell_reset;

    // Direction for each GPIO (32 used)
    reg [31:0] gpio_oeb;

    // Data to and from array periphery I/O
    wire [YSIZE-1: 0] data_in_east;
    wire [YSIZE-1: 0] data_in_west;
    wire [XSIZE-1: 0] data_in_north;
    wire [XSIZE-1: 0] data_in_south;

    wire [YSIZE-1: 0] data_out_east;
    wire [YSIZE-1: 0] data_out_west;
    wire [XSIZE-1: 0] data_out_north;
    wire [XSIZE-1: 0] data_out_south;

    // Latched output for wishbone read-back (to be done)
    // TBD

    // Latched input from wishbone (to do:  Make shadow register)
    wire [YSIZE-1: 0] latched_in_east;
    wire [YSIZE-1: 0] latched_in_west;
    wire [XSIZE-1: 0] latched_in_north;
    wire [XSIZE-1: 0] latched_in_south;

    // Shadow registers for wishbone input (to be done)
    // TBD

    // Register array mapping latched data to 32-bit sections for data
    // transfer through the wishbone
    reg [XSIZE*2 + YSIZE*2 - 1:0] latched_in;

    // Wire array mapping output data to 32-bit sections for data
    // transfer through the wishbone
    wire [XSIZE*2 + YSIZE*2 - 1:0] data_out;

    // Periphery output-to-input loop-back selection
    reg [2:0] north_loopback;
    reg [2:0] east_loopback;
    reg [2:0] south_loopback;
    reg [2:0] west_loopback;

// Loopback value definitions

`define INPUT_LOW	3'b000
`define INPUT_HIGH	3'b001
`define LOOPBACK	3'b010
`define NEIGHBOR_LEFT	3'b011
`define NEIGHBOR_RIGHT  3'b100

    // GPIO slicing (because there are many fewer GPIO than array outputs)
    // GPIOs can be clustered on either end or in the center of the array
    // side, or distributed along the side (1 GPIO per 5 array cells)
    reg [1:0] gpio_output_slice;
    reg [2:0] gpio_input_slice;

    // Registered GPIO directions go directly to io_oeb[37:6].  Leave the
    // lower 6 GPIO to the management processor.
    assign io_oeb = {gpio_oeb, 6'b1};

    // Wishbone address select indicators
    assign config_sel[0] = (wbs_adr_i[7:2] == `CONFIGL);
    assign config_sel[1] = (wbs_adr_i[7:2] == `CONFIGH);
    assign address_sel = (wbs_adr_i[7:2] == `ADDRESS);
    assign xfer_sel = (wbs_adr_i[7:2] == `XFER);
    assign direct_sel = (wbs_adr_i[7:2] == `DIRECT);
    assign source_sel = (wbs_adr_i[7:2] == `SOURCE);

    // Hard-coded to 5 words;  see note above
    assign data_sel[0] = (wbs_adr_i[7:2] == (`DATATOP + 0));
    assign data_sel[1] = (wbs_adr_i[7:2] == (`DATATOP + 1));
    assign data_sel[2] = (wbs_adr_i[7:2] == (`DATATOP + 2));
    assign data_sel[3] = (wbs_adr_i[7:2] == (`DATATOP + 3));
    assign data_sel[4] = (wbs_adr_i[7:2] == (`DATATOP + 4));

    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wbs_ack_o = ready;
    assign iomem_we = wbs_sel_i & {4{wbs_we_i}};

    assign all_cell_reset = wb_rst_i | local_reset;

    // IRQ
    assign user_irq = 3'b000;	// Unused

    // Instantiate the chaos cell array

    chaos_array #(
        .XSIZE(XSIZE),
        .YSIZE(YSIZE),
        .XTOP(XTOP),
        .YTOP(YTOP),
	.BASE_ADR(BASE_ADR)
    ) chaos_array_inst (
	`ifdef USE_POWER_PINS
    	     .vccd1(vccd1),
	     .vssd1(vssd1),
	`endif
        .clk(clk),
        .reset(all_cell_reset),
        .hold(hold),
        .rdata(rdata),
        .wdata(wdata),
	.write(write),
        .data_in_east(data_in_east),
        .data_in_west(data_in_west),
        .data_in_north(data_in_north),
        .data_in_south(data_in_south),
        .data_out_east(data_out_east),
        .data_out_west(data_out_west),
        .data_out_north(data_out_north),
        .data_out_south(data_out_south)
    );

    // Wire definitions mapping the GPIO to the array periphery
    wire [YSIZE-1:0] gpio_east, gpio_west;
    wire [XSIZE-1:0] gpio_north, gpio_south;

    // Wire definitions mapping the array periphery loop-back connections
    wire [YSIZE-1:0] data_muxed_east, data_muxed_west;
    wire [XSIZE-1:0] data_muxed_north, data_muxed_south;

    // Hook up array inputs (data_in_*) to an XOR'd combination of
    // (1) array outputs (data_out_*, muxed into data_muxed_*),
    // (2) the GPIO pads (muxed into gpio_*), and
    // (3) data from the wishbone bus (latched_in_*).

    assign data_in_west = latched_in_west ^ gpio_west ^ data_muxed_west;
    assign data_in_east = latched_in_east ^ gpio_east ^ data_muxed_east;
    assign data_in_south = latched_in_south ^ gpio_south ^ data_muxed_south;
    assign data_in_north = latched_in_north ^ gpio_north ^ data_muxed_north;

`define INPUT_LOW	3'b000
`define INPUT_HIGH	3'b001
`define LOOPBACK	3'b010
`define NEIGHBOR_LEFT	3'b011
`define NEIGHBOR_RIGHT  3'b100

    // Define loop-back inputs
    assign data_muxed_west =
	(west_loopback == `NEIGHBOR_LEFT) ? {data_out_west[YSIZE-2:0], 1'b0} :
	(west_loopback == `NEIGHBOR_RIGHT) ? {1'b0, data_out_west[YSIZE-1:1]} :
	(west_loopback == `LOOPBACK) ?  data_out_west :
	(west_loopback == `INPUT_HIGH) ? 'b1 : 'b0;

    assign data_muxed_east =
	(east_loopback == `NEIGHBOR_LEFT) ? {data_out_east[YSIZE-2:0], 1'b0} :
	(east_loopback == `NEIGHBOR_RIGHT) ? {1'b0, data_out_east[YSIZE-1:1]} :
	(east_loopback == `LOOPBACK) ?  data_out_east :
	(east_loopback == `INPUT_HIGH) ? 'b1 : 'b0;

    assign data_muxed_south =
	(south_loopback == `NEIGHBOR_LEFT) ? {data_out_south[XSIZE-2:0], 1'b0} :
	(south_loopback == `NEIGHBOR_RIGHT) ?  {1'b0, data_out_south[XSIZE-1:1]} :
	(south_loopback == `LOOPBACK) ? data_out_south :
	(south_loopback == `INPUT_HIGH) ? 'b1 : 'b0;

    assign data_muxed_north =
	(north_loopback == `NEIGHBOR_LEFT) ? {data_out_north[XSIZE-2:0], 1'b0} :
	(north_loopback == `NEIGHBOR_RIGHT) ?  {1'b0, data_out_north[XSIZE-1:1]} :
	(north_loopback == `LOOPBACK) ? data_out_north :
	(south_loopback == `INPUT_HIGH) ? 'b1 : 'b0;

    // Define I/O input slices
    // NOTE:  This is hard-coded.  There are 38 GPIOs.  Assigning 32 of them
    // (GPIO 37 to 6) to array inputs and outputs.  These are arranged as
    // 10 on the sides and 6 on the top and bottom.  These are further sub-
    // divided into 5 inputs and 5 outputs on the sides, and 3 inputs and
    // 3 outputs on top and bottom.  Depending on the selection, these
    // can be injected into various places around the array.

    // Another note:  It probably makes more sense to define vectors for
    // io_in_east, io_in_north, etc., and align them in the direction of
    // the arrays (high to low index is top to bottom, or right to left).

    assign gpio_east = 	// I/O 15 to 6
	(gpio_input_slice == 0) ? 50'b0 :	// No pad input
	(gpio_input_slice == 1) ?	// Distributed
		{2'b0, io_in[15], 4'b0, io_in[14], 4'b0, io_in[13],
		 4'b0, io_in[12], 4'b0, io_in[11], 4'b0, io_in[10],
		 4'b0, io_in[9],  4'b0, io_in[8],  4'b0, io_in[7],
		 4'b0, io_in[6],  2'b0} :
	(gpio_input_slice == 2) ? {40'b0, io_in[15:6]} :	// Bottom shifted
	(gpio_input_slice == 3) ? {20'b0, io_in[15:6], 20'b0} : // Centered
	{io_in[15:6], 40'b0};					// Top shifted

    assign gpio_north = 	// I/O 21 to 16
	(gpio_input_slice == 0) ? 30'b0 :	// No pad input
	(gpio_input_slice == 1) ?	// Distributed
		{2'b0, io_in[16], 4'b0, io_in[17], 4'b0, io_in[18],
		 4'b0, io_in[19], 4'b0, io_in[20], 4'b0, io_in[21], 2'b0} :
	(gpio_input_slice == 2) ?	// Right shifted
		{14'b0, io_in[16], io_in[17], io_in[18], io_in[19],
		io_in[20], io_in[21]} :
	(gpio_input_slice == 3) ?	// Centered
		{7'b0, io_in[16], io_in[17], io_in[18], io_in[19],
		io_in[20], io_in[21], 7'b0} :
	{io_in[16], io_in[17], io_in[18], io_in[19], io_in[20],
		io_in[21], 4'b0};	// Left shifted

    assign gpio_west = 	// I/O 22 to 31
	(gpio_input_slice == 0) ? 50'b0 :	// No pad input
	(gpio_input_slice == 1) ?	// Distributed
		{2'b0, io_in[22], 4'b0, io_in[23], 4'b0, io_in[24],
		 4'b0, io_in[25], 4'b0, io_in[26], 4'b0, io_in[27],
		 4'b0, io_in[28], 4'b0, io_in[29], 4'b0, io_in[30],
		 4'b0, io_in[31],  2'b0} :
	(gpio_input_slice == 2) ?	// Bottom shifted
		{40'b0, io_in[22], io_in[23], io_in[24], io_in[25],
		io_in[26], io_in[27], io_in[28], io_in[29], io_in[31],
		io_in[31]} :
	(gpio_input_slice == 3) ?	// Centered
		{20'b0, io_in[22], io_in[23], io_in[24], io_in[25],
		io_in[26], io_in[27], io_in[28], io_in[29], io_in[31],
		io_in[31], 20'b0} :
	{io_in[22], io_in[23], io_in[24], io_in[25], io_in[26],
		io_in[27], io_in[28], io_in[29], io_in[31], io_in[31],
		40'b0};					// Top shifted

    assign gpio_south = 	// I/O 32 to 37
	(gpio_input_slice == 0) ? 30'b0 :	// No pad input
	(gpio_input_slice == 1) ?	// Distributed
		{2'b0, io_in[37], 4'b0, io_in[36], 4'b0, io_in[35],
		 4'b0, io_in[34], 4'b0, io_in[33], 4'b0, io_in[32], 2'b0} :
	(gpio_input_slice == 2) ? {14'b0, io_in[37:32]} :	// Right shifted
	(gpio_input_slice == 3) ? {7'b0, io_in[37:32], 7'b0} :	// Centered
	{io_in[37:32], 14'b0};					// Left shifted

    // East side
    assign io_out[6] =
	(gpio_output_slice == 0) ? data_out_east[2] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[20] :	// Center
	(gpio_output_slice == 2) ? data_out_east[40] :	// Top
	data_out_east[0];				// Bottom
    assign io_out[7] =
	(gpio_output_slice == 0) ? data_out_east[7] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[21] :	// Center
	(gpio_output_slice == 2) ? data_out_east[41] :	// Top
	data_out_east[1];				// Bottom
    assign io_out[8] =
	(gpio_output_slice == 0) ? data_out_east[12] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[22] :	// Center
	(gpio_output_slice == 2) ? data_out_east[42] :	// Top
	data_out_east[2];				// Bottom
    assign io_out[9] =
	(gpio_output_slice == 0) ? data_out_east[17] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[23] :	// Center
	(gpio_output_slice == 2) ? data_out_east[43] :	// Top
	data_out_east[3];				// Bottom
    assign io_out[10] =
	(gpio_output_slice == 0) ? data_out_east[22] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[24] :	// Center
	(gpio_output_slice == 2) ? data_out_east[44] :	// Top
	data_out_east[4];				// Bottom
    assign io_out[11] =
	(gpio_output_slice == 0) ? data_out_east[27] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[25] :	// Center
	(gpio_output_slice == 2) ? data_out_east[45] :	// Top
	data_out_east[5];				// Bottom
    assign io_out[12] =
	(gpio_output_slice == 0) ? data_out_east[32] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[26] :	// Center
	(gpio_output_slice == 2) ? data_out_east[46] :	// Top
	data_out_east[6];				// Bottom
    assign io_out[13] =
	(gpio_output_slice == 0) ? data_out_east[37] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[27] :	// Center
	(gpio_output_slice == 2) ? data_out_east[47] :	// Top
	data_out_east[7];				// Bottom
    assign io_out[14] =
	(gpio_output_slice == 0) ? data_out_east[42] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[28] :	// Center
	(gpio_output_slice == 2) ? data_out_east[48] :	// Top
	data_out_east[8];				// Bottom
    assign io_out[15] =
	(gpio_output_slice == 0) ? data_out_east[47] :	// Distributed
	(gpio_output_slice == 1) ? data_out_east[29] :	// Center
	(gpio_output_slice == 2) ? data_out_east[49] :	// Top
	data_out_east[9];				// Bottom

    // North side
    assign io_out[16] =
	(gpio_output_slice == 0) ? data_out_north[27] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[16] :	// Center
	(gpio_output_slice == 2) ? data_out_north[29] :	// Right
	data_out_north[5];				// Left
    assign io_out[17] =
	(gpio_output_slice == 0) ? data_out_north[22] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[15] :	// Center
	(gpio_output_slice == 2) ? data_out_north[28] :	// Right
	data_out_north[4];				// Left
    assign io_out[18] =
	(gpio_output_slice == 0) ? data_out_north[17] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[14] :	// Center
	(gpio_output_slice == 2) ? data_out_north[27] :	// Right
	data_out_north[3];				// Left
    assign io_out[19] =
	(gpio_output_slice == 0) ? data_out_north[12] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[13] :	// Center
	(gpio_output_slice == 2) ? data_out_north[26] :	// Right
	data_out_north[2];				// Left
    assign io_out[20] =
	(gpio_output_slice == 0) ? data_out_north[7] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[12] :	// Center
	(gpio_output_slice == 2) ? data_out_north[25] :	// Right
	data_out_north[1];				// Left
    assign io_out[21] =
	(gpio_output_slice == 0) ? data_out_north[2] :	// Distributed
	(gpio_output_slice == 1) ? data_out_north[11] :	// Center
	(gpio_output_slice == 2) ? data_out_north[24] :	// Right
	data_out_north[0];				// Left

    // West side
    assign io_out[22] =
	(gpio_output_slice == 0) ? data_out_west[47] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[29] :	// Center
	(gpio_output_slice == 2) ? data_out_west[49] :	// Top
	data_out_west[9];				// Bottom
    assign io_out[23] =
	(gpio_output_slice == 0) ? data_out_west[42] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[28] :	// Center
	(gpio_output_slice == 2) ? data_out_west[48] :	// Top
	data_out_west[8];				// Bottom
    assign io_out[24] =
	(gpio_output_slice == 0) ? data_out_west[37] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[27] :	// Center
	(gpio_output_slice == 2) ? data_out_west[47] :	// Top
	data_out_west[7];				// Bottom
    assign io_out[25] =
	(gpio_output_slice == 0) ? data_out_west[32] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[26] :	// Center
	(gpio_output_slice == 2) ? data_out_west[46] :	// Top
	data_out_west[6];				// Bottom
    assign io_out[26] =
	(gpio_output_slice == 0) ? data_out_west[27] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[25] :	// Center
	(gpio_output_slice == 2) ? data_out_west[45] :	// Top
	data_out_west[5];				// Bottom
    assign io_out[27] =
	(gpio_output_slice == 0) ? data_out_west[22] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[24] :	// Center
	(gpio_output_slice == 2) ? data_out_west[44] :	// Top
	data_out_west[4];				// Bottom
    assign io_out[28] =
	(gpio_output_slice == 0) ? data_out_west[17] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[23] :	// Center
	(gpio_output_slice == 2) ? data_out_west[43] :	// Top
	data_out_west[3];				// Bottom
    assign io_out[29] =
	(gpio_output_slice == 0) ? data_out_west[12] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[22] :	// Center
	(gpio_output_slice == 2) ? data_out_west[42] :	// Top
	data_out_west[2];				// Bottom
    assign io_out[30] =
	(gpio_output_slice == 0) ? data_out_west[7] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[21] :	// Center
	(gpio_output_slice == 2) ? data_out_west[41] :	// Top
	data_out_west[1];				// Bottom
    assign io_out[31] =
	(gpio_output_slice == 0) ? data_out_west[2] :	// Distributed
	(gpio_output_slice == 1) ? data_out_west[20] :	// Center
	(gpio_output_slice == 2) ? data_out_west[40] :	// Top
	data_out_west[0];				// Bottom

    // South side
    assign io_out[32] =
	(gpio_output_slice == 0) ? data_out_south[2] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[11] :	// Center
	(gpio_output_slice == 2) ? data_out_south[24] :	// Right
	data_out_south[0];				// Left
    assign io_out[33] =
	(gpio_output_slice == 0) ? data_out_south[7] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[12] :	// Center
	(gpio_output_slice == 2) ? data_out_south[25] :	// Right
	data_out_south[1];				// Left
    assign io_out[34] =
	(gpio_output_slice == 0) ? data_out_south[12] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[13] :	// Center
	(gpio_output_slice == 2) ? data_out_south[26] :	// Right
	data_out_south[2];				// Left
    assign io_out[35] =
	(gpio_output_slice == 0) ? data_out_south[17] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[14] :	// Center
	(gpio_output_slice == 2) ? data_out_south[27] :	// Right
	data_out_south[3];				// Left
    assign io_out[36] =
	(gpio_output_slice == 0) ? data_out_south[22] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[15] :	// Center
	(gpio_output_slice == 2) ? data_out_south[28] :	// Right
	data_out_south[4];				// Left
    assign io_out[37] =
	(gpio_output_slice == 0) ? data_out_south[27] :	// Distributed
	(gpio_output_slice == 1) ? data_out_south[16] :	// Center
	(gpio_output_slice == 2) ? data_out_south[29] :	// Right
	data_out_south[5];				// Left

    // Map the output data from the sides to a single array that can be
    // broken up into 32 bit segments for data transfer.  

    assign data_out = {data_out_north, data_out_east, data_out_south, data_out_west};

    /* Read data (only rdata is something that was not written by the processor) */

    always @* begin
	rdata_pre = 'b0;
 	if (xfer_sel) begin
	    rdata_pre = {30'b0, busy};
	end else if (config_sel[0]) begin
	    rdata_pre = rdata[31:0];
	end else if (config_sel[1]) begin
	    rdata_pre = rdata[63:32];
	end else if (address_sel) begin
	    /* When ADDRESS is selected, pass back the existing cell	*/
	    /* count rather than what was written into cell_addr.	*/
	    rdata_pre = bit_count[ASIZE + 6: 7];
	end else if (direct_sel) begin
	    rdata_pre = gpio_oeb;
	end else if (source_sel) begin
	    rdata_pre = {9'b0, gpio_output_slice, 1'b0, gpio_input_slice,
			1'b0, north_loopback, 1'b0, east_loopback,
			1'b0, south_loopback, 1'b0, west_loopback};
	end else if (data_sel[0]) begin
	    rdata_pre = data_out[31:0];
	end else if (data_sel[1]) begin
	    rdata_pre = data_out[63:32];
	end else if (data_sel[2]) begin
	    rdata_pre = data_out[95:64];
	end else if (data_sel[3]) begin
	    rdata_pre = data_out[127:96];
	end else if (data_sel[4]) begin
	    rdata_pre = data_out[159:128];
	end
    end

    /* Read data */

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
	if (wb_rst_i) begin
	    wbs_dat_o <= 0;
	    ready <= 0;
	end else begin
	    ready <= 0;
            if (valid && !ready && (wbs_adr_i[31:8] == BASE_ADR[31:8])) begin
		ready <= 1'b1;
		wbs_dat_o <= rdata_pre;
	    end
	end
    end

    // Map the latched data from the sides to a single array that can be
    // broken up into 32 bit segments for data transfer.  

    assign latched_in_north = latched_in[2*XSIZE+2*YSIZE-1:2*XSIZE+YSIZE];
    assign latched_in_east = latched_in[2*YSIZE+XSIZE-1:YSIZE+XSIZE];
    assign latched_in_south = latched_in[YSIZE+XSIZE-1:YSIZE];
    assign latched_in_west = latched_in[YSIZE-1:0];

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
	    cell_addr <= 0;
	    gpio_oeb <= 0;
            xfer_ctrl <= 0;
            local_reset <= 0;
	    west_loopback <= 0;
	    east_loopback <= 0;
	    north_loopback <= 0;
	    south_loopback <= 0;
	    gpio_input_slice <= 0;
	    gpio_output_slice <= 0;
	    latched_in <= 0;
	    wdata <= 0;
	    write <= 1'b0;
        end else begin
	    write <= 1'b0;
            if (valid && !ready && wbs_adr_i[31:8] == BASE_ADR[31:8]) begin
                if (xfer_sel) begin
                    if (iomem_we[0]) begin
			xfer_ctrl <= wbs_dat_i[1:0];
			local_reset <= wbs_dat_i[2];
		    end
		end else if (config_sel[0]) begin
                    if (iomem_we[0]) wdata[7:0] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) wdata[15:8] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) wdata[23:16] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) wdata[31:24] <= wbs_dat_i[31:24];
		    if (|iomem_we) write <= 1'b1;
		end else if (config_sel[1]) begin
                    if (iomem_we[0]) wdata[39:32] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) wdata[47:40] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) wdata[55:48] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) wdata[63:56] <= wbs_dat_i[31:24];
		    if (|iomem_we) write <= 1'b1;
		end else if (address_sel) begin
		    // NOTE:  Assumes MAXADDR > 256 && MAXADDR < 65536
                    if (iomem_we[0]) cell_addr[7:0] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) cell_addr[ASIZE-1:8] <= wbs_dat_i[ASIZE-1:8];
		end else if (direct_sel) begin
                    if (iomem_we[0]) gpio_oeb[7:0] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) gpio_oeb[15:8] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) gpio_oeb[23:16] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) gpio_oeb[31:24] <= wbs_dat_i[31:24];
		end else if (source_sel) begin
                    if (iomem_we[0]) begin
			 west_loopback <= wbs_dat_i[2:0];
			 south_loopback <= wbs_dat_i[6:4];
		    end
                    if (iomem_we[1]) begin
			 east_loopback <= wbs_dat_i[2:0];
			 north_loopback <= wbs_dat_i[6:4];
		    end
                    if (iomem_we[2]) begin
			 gpio_input_slice <= wbs_dat_i[2:0];
			 gpio_output_slice <= wbs_dat_i[6:4];
		    end
		end else if (data_sel[0]) begin
                    if (iomem_we[0]) latched_in[7:0] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) latched_in[15:8] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) latched_in[23:16] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) latched_in[31:24] <= wbs_dat_i[31:24];
		end else if (data_sel[1]) begin
                    if (iomem_we[0]) latched_in[39:32] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) latched_in[47:40] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) latched_in[55:48] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) latched_in[63:56] <= wbs_dat_i[31:24];
		end else if (data_sel[2]) begin
                    if (iomem_we[0]) latched_in[71:64] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) latched_in[79:72] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) latched_in[87:80] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) latched_in[95:88] <= wbs_dat_i[31:24];
		end else if (data_sel[3]) begin
                    if (iomem_we[0]) latched_in[103:96] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) latched_in[111:104] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) latched_in[119:112] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) latched_in[127:120] <= wbs_dat_i[31:24];
		end else if (data_sel[4]) begin
                    if (iomem_we[0]) latched_in[135:128] <= wbs_dat_i[7:0];
                    if (iomem_we[1]) latched_in[143:136] <= wbs_dat_i[15:8];
                    if (iomem_we[2]) latched_in[151:144] <= wbs_dat_i[23:16];
                    if (iomem_we[3]) latched_in[159:152] <= wbs_dat_i[31:24];
                end
            end else begin
                xfer_ctrl <= 0;      // Immediately self-resetting
                local_reset <= 0;    // Immediately self-resetting
            end
        end
    end

    /* Transfer status */

    assign busy[0] = (xfer_state == `START || xfer_state == `XDATAS);
    assign busy[1] = (xfer_state == `FINISH || xfer_state == `XDATAF ||
			xfer_state == `LOAD);

    /* Transfer cycles */

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
	if (wb_rst_i == 1'b1) begin
	    xfer_state <= `IDLE;
	    bit_count <= 'd0;
	    cell_offset <= 'd0;
	    clk <= 1'b0;
	    hold <= 1'b1;
	end else begin
	    clk <= 1'b0;
	    hold <= 1'b1;
	    if (xfer_state == `IDLE) begin
		if (xfer_ctrl[0] == 1'b1) begin
		    xfer_state <= `START;
		end else if (xfer_ctrl[1] == 1'b1) begin
		    xfer_state <= `FINISH;
		end
	    end else if (xfer_state == `START) begin
		bit_count[ASIZE + 6:7] <= cell_addr;
		bit_count[6:0] <= 7'b1111110;
		xfer_state <= `XDATAS;
	    end else if (xfer_state == `FINISH) begin
		bit_count[ASIZE + 6:7] <= `MAXADDR - cell_offset;
		bit_count[6:0] <= 7'b1111110;
		xfer_state <= `XDATAF;
	    end else if (xfer_state == `XDATAS) begin
		clk <= ~clk;
		bit_count <= bit_count - 1;
		if (bit_count[6:0] == 0) begin
		    cell_offset <= cell_offset + 1;
		end
		if (clk == 1'b0) begin
		    if (bit_count == 0) begin
			xfer_state <= `IDLE;
		    end
		end
	    end else if (xfer_state == `XDATAF) begin
		clk <= ~clk;
		bit_count <= bit_count - 1;
		if (bit_count[6:0] == 0) begin
		    cell_offset <= cell_offset + 1;
		end
		if (clk == 1'b0) begin
		    if (bit_count == 0) begin
			xfer_state <= `LOAD;
		    end
		end
	    end else if (xfer_state == `LOAD) begin
		hold <= 1'b0;
		xfer_state <= `IDLE;
		cell_offset <= 'd0;
	    end
	end
    end
endmodule

/*
 *-----------------------------------------------------------------
 * Chaos array (XSIZE * YSIZE)
 *-----------------------------------------------------------------
 */

module chaos_array #(
    parameter XSIZE = 30,   /* Total number of cells in X */
    parameter YSIZE = 30,   /* Total number of cells in Y */
    parameter XTOP = 3,	    /* Number of sub-arrays in X */
    parameter YTOP = 3,	    /* Number of sub-arrays in Y */
    parameter BASE_ADR = 32'h3000_0000
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1, 	// User area 1 digital ground
`endif

    input clk,
    input reset,
    input hold,
    input write,
    input [63:0] wdata,
    output [63:0] rdata,
    input [YSIZE-1:0] data_in_east,	// Perimeter input
    input [YSIZE-1:0] data_in_west,
    input [XSIZE-1:0] data_in_north,
    input [XSIZE-1:0] data_in_south,
    output [YSIZE-1:0] data_out_east,	// Perimeter output
    output [YSIZE-1:0] data_out_west,
    output [XSIZE-1:0] data_out_north,
    output [XSIZE-1:0] data_out_south
);
    wire [XSIZE - 1: 0] uconn [YTOP: 0];	// Upward moving data
    wire [XSIZE - 1: 0] dconn [YTOP: 0];	// Downward moving data
    wire [YSIZE - 1: 0] rconn [XTOP: 0];	// Rightward moving data
    wire [YSIZE - 1: 0] lconn [XTOP: 0];	// Leftward moving data

    wire [YTOP - 1: 0] shiftreg [XTOP: 0];
    wire [YTOP - 1: 0] clkarray [XTOP: 0];

    wire io_data_sel;		// wishbone select data
    wire xfer_sel;		// wishbone select transfer

    assign clkarray[0][0] = clk;

    // Sub-array architecture:
    //
    //       dudu      dudu      dudu
    //       |^|^      |^|^      |^|^   
    //       v|v|      v|v|      v|v|   
    //     +------+  +------+  +------+
    //  r->|      |->|      |->|      |->r
    //  l<-|      |<-|      |<-|      |<-l
    //  r->|      |->|      |->|      |->r
    //  l<-|      |<-|      |<-|      |<-l
    //     +------+  +------+  +------+
    //       |^|^      |^|^      |^|^   
    //       v|v|      v|v|      v|v|   
    //     +------+  +------+  +------+
    //  r->|      |->|      |->|      |->r
    //  l<-|      |<-|      |<-|      |<-l
    //  r->|      |->|      |->|      |->r
    //  l<-|      |<-|      |<-|      |<-l
    //     +------+  +------+  +------+
    //       |^|^      |^|^      |^|^   
    //       v|v|      v|v|      v|v|   
    //       dudu      dudu      dudu
    //
    // Each box in the above diagram is a sub-array size 2x2.
    // The top level has XSIZE = 6, YSIZE = 4 with XTOP = 3
    // and YTOP = 2.
    //
    // The top-level inputs and outputs are the perimeter values
    // on the four edges of the top level array.
    //
    // To represent all the connections among the sub-arrays, it
    // can be seen from the above that d and u (dconn and uconn)
    // are arrays of size (XSIZE, YTOP + 1), while l and r (lconn
    // and rconn) are arrays of size (XTOP + 1, YSIZE).

    // NOTE:  For viewing internal signals in gtkwave,
    // some 2D arrays may need to be copied into 1D arrays.
    // See the original verilog for examples.

    /* The perimeter inputs and outputs connect the array to the
     * parent module.  Note that this hides all the interior data,
     * which could be an issue with understanding how the circuit
     * works.
     */

    assign data_out_north = uconn[YTOP][XSIZE - 1:0];
    assign data_out_south = dconn[0][XSIZE - 1:0];
    assign data_out_east = rconn[XTOP][YSIZE - 1:0];
    assign data_out_west = lconn[0][YSIZE - 1:0];

    assign dconn[YTOP][XSIZE - 1:0] = data_in_north;
    assign uconn[0][XSIZE - 1:0] = data_in_south;
    assign rconn[0][YSIZE - 1:0] = data_in_west;
    assign lconn[XTOP][YSIZE - 1:0] = data_in_east;

    genvar i, j;

    /* NOTE:  To see the internal cell values in gtkwave, it is necessary
     * to split out a few individual instances from the 2D array.  Loop
     * from j = 1 in 2D generate loop, then add a 1D generate loop for
     * i = N to XSIZE with j set to zero, then add individual instances for 
     * i = 0 to N - 1 with j set to zero.
     */

    /* Connected array of subarrays */
    generate
	for (j = 0; j < YTOP; j=j+1) begin: subarrayy
	    for (i = 0; i < XTOP; i=i+1) begin: subarrayx
    	        chaos_subarray #(
		    .XSIZE(XSIZE / XTOP),
		    .YSIZE(YSIZE / YTOP)
		) chaos_subarray_inst (
		    `ifdef USE_POWER_PINS
			.vccd1(vccd1),
			.vssd1(vssd1),
		    `endif
    		    .inorth(dconn[j+1][(i+1)*(XSIZE/XTOP)-1:i*(XSIZE/XTOP)]),
		    .isouth(uconn[j][(i+1)*(XSIZE/XTOP)-1:i*(XSIZE/XTOP)]),
		    .ieast(lconn[i+1][(j+1)*(YSIZE/YTOP)-1:j*(YSIZE/YTOP)]),
		    .iwest(rconn[i][(j+1)*(YSIZE/YTOP)-1:j*(YSIZE/YTOP)]),
		    .onorth(uconn[j+1][(i+1)*(XSIZE/XTOP)-1:i*(XSIZE/XTOP)]),
		    .osouth(dconn[j][(i+1)*(XSIZE/XTOP)-1:i*(XSIZE/XTOP)]),
		    .oeast(rconn[i+1][(j+1)*(YSIZE/YTOP)-1:j*(YSIZE/YTOP)]),
		    .owest(lconn[i][(j+1)*(YSIZE/YTOP)-1:j*(YSIZE/YTOP)]),
		    .reset(reset),
		    .hold(hold),
		    .iclk(clkarray[i][j]),
		    .oclk(clkarray[i+1][j]),
		    .idata(shiftreg[i][j]),
		    .odata(shiftreg[i+1][j])
    	    	);
	    end
	end

	/* NOTE:  This would work better topologically if each	*/
	/* row switched the direction of the shift register.	*/

	for (j = 0; j < YTOP - 1; j=j+1) begin: shifty
	    assign shiftreg[0][j+1] = shiftreg[XTOP][j];
	    assign clkarray[0][j+1] = clkarray[XTOP][j];
	end
    endgenerate

    /* Storage for data transfers to and from the processor.  This is	*/
    /* 64 bits, so can hold the configuration data for one core cell.	*/
   
    reg [63:0] lutdata;

    /* Wire up the lutdata registers as a shift register and connect the */
    /* ends to the array's shift register to form a loop.		*/

    always @(posedge clk or posedge write) begin
	if (write) begin
	    /* Copy data from wdata to lutdata on write */
	    lutdata <= wdata;
	end else begin
	    /* Shift data on clock when "write" is not raised */
	    lutdata[63:1] <= lutdata[62:0];
	    lutdata[0] <= shiftreg[XTOP][YTOP-1];
	end
    end

    assign shiftreg[0][0] = lutdata[63];

    assign rdata = lutdata;	/* Data to read back */

endmodule
`default_nettype wire
