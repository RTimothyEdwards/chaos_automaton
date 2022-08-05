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

// NOTE:  Remove the following line before synthesizing
// `define MPRJ_IO_PADS 38

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
 * Memory mapped address space:
 *
 *	BASE_ADR + 7 to BASE_ADR + 0:   Data to read or write
 *	BASE_ADR + 15 to BASE_ADR + 8:	Core cell address for read/write
 *	BASE_ADR + 16:			Triggers
 *
 * Trigger bits:
 *	bit 0:  Shift by (address) cells (64 bits).
 *	bit 1:  Finish cycle.  Return shift register to run state, toggle "hold"
 *
 * Both trigger bits are self-resetting.  The trigger bit (as read) remains
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

//`include "chaos_subarray.v"

/*
 *-----------------------------------------------------------------
 * User project top level 
 *-----------------------------------------------------------------
 */

module chaos_automaton #(
    parameter XSIZE = 30,	// Total number of cells left to right
    parameter YSIZE = 50,	// Total number of cells top to bottom
    parameter XTOP = 3,		// Number of sub-arrays left to right
    parameter YTOP = 5,		// Number of sub-arrays top to bottom
    parameter ASIZE = 10,	// Enough bits to count XSIZE * YSIZE
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

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);

`define IDLE	3'b000
`define START	3'b001
`define FINISH	3'b010
`define XDATAS	3'b011
`define XDATAF	3'b100
`define LOAD	3'b101

`define CONFIGL	8'h00		/* Address offset of configuration data low word */
`define CONFIGH	8'h04		/* Address offset of configuration data high word */
`define ADDRESS	8'h08		/* Address offset of cell address value */
`define XFER	8'h0c		/* Address offset of transfer bits */

`define MAXADDR (XSIZE * YSIZE)	/* Highest cell address plus one */

    reg clk;			/* serial clock to transfer data 	*/
    reg hold;			/* trigger to hold transferred data 	*/
    reg [2:0] xfer_state;	/* state of the data transfer		*/
    reg [1:0] xfer_ctrl;	/* Transfer trigger bits		*/
    reg [63:0] config_data;	/* 64 bits to read or write		*/

    reg [ASIZE - 1:0] cell_addr;	/* Core cell to address	*/
    reg [ASIZE - 1:0] cell_offset;	/* Current offset of shift register */
    reg [ASIZE + 6:0] bit_count;	/* Full count (cell address + bits) */

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [1:0] config_sel;
    wire address_sel;
    wire xfer_sel;

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

    wire [2*XSIZE + 2*YSIZE - 1: 0] data_in;

    // Wishbone address select indicators
    assign config_sel[0] = (wbs_adr_i[7:0] == `CONFIGL);
    assign config_sel[1] = (wbs_adr_i[7:0] == `CONFIGH);
    assign address_sel = (wbs_adr_i[7:0] == `ADDRESS);
    assign xfer_sel = (wbs_adr_i[7:0] == `XFER);

    assign selected = config_sel[1] || config_sel[0] || address_sel || xfer_sel;
    
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wbs_ack_o = ready;
    assign iomem_we = wbs_sel_i & {4{wbs_we_i}};

    // Chip pin output (Connects to a subset of la_data_in;
    // 9 signals each N and S, 10 signals each W and E)
    assign io_out = {la_data_out[2*YSIZE + XSIZE + 8: 2*YSIZE + XSIZE],	// north
		     la_data_out[2*YSIZE + 8: 2*YSIZE],			// south
		     la_data_out[YSIZE + 9: YSIZE],			// east
		     la_data_out[9:0]};					// west

    // Chip pin direction is assigned to la_data sub-array
    assign io_oeb = la_data_in[127:127-38] & ~la_oenb[127:127-38];

    // IRQ
    assign irq = 3'b000;	// Unused

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
        .reset(wb_rst_i),
        .hold(hold),
        .rdata(rdata),
        .wdata(wdata),
	.write(write),
        .data_in(data_in),
        .data_out(la_data_out[2*XSIZE + 2*YSIZE - 1: 0])
    );

    /* Hook up io_in (multiplexed with la_data_int based on value of la_oenb,
     * using the same subsets as used for io_out).  The expressions are more
     * complicated because the signals that are connected to the GPIO pins
     * have to be multiplexed with the logic analyzer inputs.
     */

    genvar i;

    generate
	for (i = 2*YSIZE + XSIZE + 9; i < 2*YSIZE + 2*XSIZE; i=i+1) begin
	    assign data_in[i] = la_data_in[i];
	end
	for (i = 2 * YSIZE + XSIZE; i < 2*YSIZE + XSIZE + 9; i=i+1) begin
    	    assign data_in[i] = la_oenb[i] ? io_in[i - 2*YSIZE + XSIZE + 29] : la_data_in[i];
	end
	for (i = 2 * YSIZE + 9; i < 2 * YSIZE + XSIZE; i=i+1) begin
	    assign data_in[i] = la_data_in[i];
	end
	for (i = 2 * YSIZE; i < 2 * YSIZE + 9; i=i+1) begin
    	    assign data_in[i] = la_oenb[i] ? io_in[i - 2*YSIZE + 20] : la_data_in[i];
	end
	for (i = YSIZE + 10; i < 2 * YSIZE; i=i+1) begin
	    assign data_in[i] = la_data_in[i];
	end
	for (i = YSIZE; i < YSIZE + 10; i=i+1) begin
    	    assign data_in[i] = la_oenb[i] ? io_in[i - YSIZE + 10] : la_data_in[i];
	end
	for (i = 10; i < YSIZE; i=i+1) begin
	    assign data_in[i] = la_data_in[i];
	end
	for (i = 0; i < 10; i=i+1) begin
    	    assign data_in[i] = la_oenb[i] ? io_in[i]: la_data_in[i];
	end
    endgenerate

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
	end
    end

    /* Read data */

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
	if (wb_rst_i) begin
	    wbs_dat_o <= 0;
	    ready <= 0;
	end else begin
	    ready <= 0;
            if (valid && !ready && wbs_adr_i[31:8] == BASE_ADR[31:8]) begin
		ready <= 1'b1;
		if (selected) begin
		    wbs_dat_o <= rdata_pre;
		end
	    end
	end
    end

    /* Write data */

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            xfer_ctrl <= 0;
	    wdata <= 0;
	    write <= 1'b0;
        end else begin
	    write <= 1'b0;
            if (valid && !ready && wbs_adr_i[31:8] == BASE_ADR[31:8]) begin
                if (xfer_sel) begin
                    if (iomem_we[0]) xfer_ctrl <= wbs_dat_i[1:0];
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
		    /* NOTE:  If XSIZE * YSIZE > 256, this must be adjusted */
                    if (iomem_we[0]) cell_addr <= wbs_dat_i[7:0];
                end
            end else begin
                xfer_ctrl <= 0;      // Immediately self-resetting
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
    input  [2*XSIZE + 2*YSIZE - 1:0] data_in,	// Perimeter I/O
    output [2*XSIZE + 2*YSIZE - 1:0] data_out	// Perimeter I/O
);
    wire [XSIZE - 1: 0] uconn [YTOP: 0];
    wire [XSIZE - 1: 0] dconn [YTOP: 0];
    wire [YSIZE - 1: 0] rconn [XTOP: 0];
    wire [YSIZE - 1: 0] lconn [XTOP: 0];

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
    //  l->|      |->|      |->|      |->l
    //  r<-|      |<-|      |<-|      |<-r
    //  l->|      |->|      |->|      |->l
    //  r<-|      |<-|      |<-|      |<-r
    //     +------+  +------+  +------+
    //       |^|^      |^|^      |^|^   
    //       v|v|      v|v|      v|v|   
    //     +------+  +------+  +------+
    //  l->|      |->|      |->|      |->l
    //  r<-|      |<-|      |<-|      |<-r
    //  l->|      |->|      |->|      |->l
    //  r<-|      |<-|      |<-|      |<-r
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

    /* The perimeter inputs and outputs connect to the logic analyzer */
    /* (To do:  multiplex inputs between the chip I/O and logic analyzer */

    assign data_out = {uconn[YTOP][XSIZE - 1:0], dconn[0][XSIZE - 1:0],
			  rconn[XTOP][YSIZE - 1:0], lconn[0][YSIZE - 1:0]};

    assign dconn[YTOP][XSIZE - 1:0] = data_in[2*XSIZE+2*YSIZE - 1: 2*YSIZE + XSIZE];
    assign uconn[0][XSIZE - 1:0] = data_in[2*YSIZE + XSIZE - 1:2*YSIZE];
    assign rconn[0][YSIZE - 1:0] = data_in[2*YSIZE-1:YSIZE];
    assign lconn[XTOP][YSIZE - 1:0] = data_in[YSIZE-1:0];

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
