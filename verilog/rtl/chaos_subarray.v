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
 * chaos_subarray
 *
 * This is a portion of the chaos_automaton array.  The array
 * has been broken up into smaller sub-arrays because the
 * full design runs out of memory on a relatively ample 16GB
 * machine.  However, a 10x10 array could be synthesized, so
 * the revised approach is to generate a macro out of the
 * 10x10 array, and then tile that macro in the final design.
 * For an example, the 10x10 array can be synthesized into a
 * 900x900 micron area in Sky130, so there is room for a 3x3
 * array of these macros, for a total cell count of 30x30.
 *
 * NOTE:  The programming of each cell follows the same
 * method used by the GPIO control block programming in
 * caravel with respect to clock and data:  The clock is
 * propagated from cell to cell through the array so that
 * each cell's clock has a (relatively) fixed timing relative
 * to the data, and does not require a massive clock tree.
 * The last data bit from the cell's shift register is
 * reclocked on the clock's falling edge so that the timing
 * of the clock to the following cell has a large margin.
 * Reset and hold, however, are global signals because the
 * data update needs to be as simultaneous as the synthesis
 * tools can make it.
 *
 *-------------------------------------------------------------
 */

/*
 * Chaos automaton base cell definitions:  Map directions to
 * array indexes, in clockwise order
 */

`define NORTH 3
`define EAST  2
`define SOUTH 1
`define WEST  0

/*
 *-----------------------------------------------------------------
 * Chaos base cell (four 4-input LUTs + data load circuitry)
 *
 * NOTE:  The last data bit out is clocked on the falling edge
 * of the clock so that there is no possibility of a hold
 * violation between cells.
 *-----------------------------------------------------------------
 */

module chaos_cell (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input inorth, isouth, ieast, iwest,
    output onorth, osouth, oeast, owest,
    input iclk,			/* Serial load clock (in) */
    output oclk,		/* Serial load clock (out) */
    input reset,		/* System reset */
    input hold,			/* Data latch signal */
    input idata,		/* Shift register input */
    output odata 		/* Shift register output */
);

    reg [15:0] lutfunc [3:0];	/* LUT configuration data */
    reg [15:0] lutdata [3:0];	/* Latched LUT configuration data */
    reg odata;			/* Latched shift register output */
    wire [3:0] inesw;
    wire [3:0] ieswn;
    wire [3:0] iswne;
    wire [3:0] iwnes;

    /* Gather inputs into arrays.  There is one array per direction, so	*/
    /* that the array is always oriented relative to the position of	*/
    /* the output being generated.					*/

    assign inesw = {inorth, ieast,  isouth, iwest};
    assign ieswn = {ieast,  isouth, iwest,  inorth};
    assign iswne = {isouth, iwest,  inorth, ieast};
    assign iwnes = {iwest,  inorth, ieast,  isouth};

    /* Core functions */
    /* The four LUTs define each output as a function of the four inputs */
    /* To do:  Make everything rotationally symmetric */

    /* NOTE: condition of zeroing on hold == 0 is needed to make	*/
    /* simulation run;  otherwise outputs are all X.  The system will	*/
    /* work without it.							*/

    assign onorth = (!hold) ? 0 : lutdata[`NORTH][inesw];
    assign oeast  = (!hold) ? 0 : lutdata[`EAST][ieswn];
    assign osouth = (!hold) ? 0 : lutdata[`SOUTH][iswne];
    assign owest  = (!hold) ? 0 : lutdata[`WEST][iwnes];

    /* Inferred latches from shift register */

    always @* begin
	if (!hold) begin
	    lutdata[0] = lutfunc[0];
	    lutdata[1] = lutfunc[1];
	    lutdata[2] = lutfunc[2];
	    lutdata[3] = lutfunc[3];
	end
    end

    /* Implement the shift register operation */

    always @(posedge iclk or posedge reset) begin
        if (reset == 1'b1) begin
	    lutfunc[`NORTH] <= 16'd0;
	    lutfunc[`SOUTH] <= 16'd0;
	    lutfunc[`EAST]  <= 16'd0;
	    lutfunc[`WEST]  <= 16'd0;
	end else begin
	    lutfunc[`NORTH][15:1] <= lutfunc[`NORTH][14:0];
	    lutfunc[`EAST][15:1]  <= lutfunc[`EAST][14:0];
	    lutfunc[`SOUTH][15:1] <= lutfunc[`SOUTH][14:0];
	    lutfunc[`WEST][15:1]  <= lutfunc[`WEST][14:0];

	    lutfunc[`NORTH][0] <= idata;
	    lutfunc[`EAST][0] <= lutfunc[`NORTH][15];
	    lutfunc[`SOUTH][0] <= lutfunc[`EAST][15];
	    lutfunc[`WEST][0] <= lutfunc[`SOUTH][15];
	end
    end

    always @(negedge iclk or posedge reset) begin
	if (reset == 1'b1) begin
	    odata <= 1'b0;
	end else begin
	    odata <= lutfunc[`WEST][15];
	end
    end

    /* Propagate clock */
    assign oclk = iclk;

endmodule

/*
 *-----------------------------------------------------------------
 * Chaos sub-array (XSIZE * YSIZE)
 *-----------------------------------------------------------------
 */

module chaos_subarray #(
    parameter XSIZE = 10,
    parameter YSIZE = 10
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1, 	// User area 1 digital ground
`endif
	
    input [XSIZE-1:0] inorth, isouth,
    input [YSIZE-1:0] ieast, iwest,
    output [XSIZE-1:0] onorth, osouth,
    output [YSIZE-1:0] oeast, owest,
    input iclk,			/* Serial load clock (in) */
    output oclk,		/* Serial load clock (out) */
    input reset,		/* System reset */
    input hold,			/* Data latch signal */
    input idata,		/* Shift register input */
    output odata 		/* Shift register output */
);

    wire [XSIZE - 1: 0] uconn [YSIZE: 0];
    wire [XSIZE - 1: 0] dconn [YSIZE: 0];
    wire [YSIZE - 1: 0] rconn [XSIZE: 0];
    wire [YSIZE - 1: 0] lconn [XSIZE: 0];

    /* The shift register (data) and clock wind through the cells */
    /* to maintain consistent timing.  Hold and reset are applied */
    /* simultaneously.						  */

    wire [YSIZE - 1: 0] shiftreg [XSIZE: 0];
    wire [YSIZE - 1: 0] clkarray [XSIZE: 0];

    genvar i, j;

    /* Connected array of cells */
    generate
	for (j = 0; j < YSIZE; j=j+1) begin: celly
	    for (i = 0; i < XSIZE; i=i+1) begin: cellx
    	        chaos_cell chaos_cell_inst (
		    `ifdef USE_POWER_PINS
			.vccd1(vccd1),
			.vssd1(vssd1),
		    `endif
    		    .inorth(dconn[j+1][i]),
		    .isouth(uconn[j][i]),
		    .ieast(lconn[i+1][j]),
		    .iwest(rconn[i][j]),
		    .onorth(uconn[j+1][i]),
		    .osouth(dconn[j][i]),
		    .oeast(rconn[i+1][j]),
		    .owest(lconn[i][j]),
		    .iclk(clkarray[i][j]),
		    .oclk(clkarray[i+1][j]),
		    .reset(reset),
		    .hold(hold),
		    .idata(shiftreg[i][j]),
		    .odata(shiftreg[i+1][j])
    	    	);
	    end
	end

	/* NOTE:  This would work better topologically if each	*/
	/* row switched the direction of clock and data.	*/

	for (j = 0; j < YSIZE - 1; j=j+1) begin: shifty
	    assign shiftreg[0][j+1] = shiftreg[XSIZE][j];
	    assign clkarray[0][j+1] = clkarray[XSIZE][j];
	end

	/* Connect the endpoints of the array to the inputs and outputs of the module */

	for (j = 0; j < YSIZE; j=j+1) begin: conny
	    assign rconn[0][j] = iwest[j];
	    assign lconn[XSIZE][j] = ieast[j];
	    assign oeast[j] = rconn[XSIZE][j];
	    assign owest[j] = lconn[0][j];
	end

	for (i = 0; i < XSIZE; i=i+1) begin: connx
	    assign uconn[0][i] = isouth[i];
	    assign dconn[YSIZE][i] = inorth[i];
	    assign onorth[i] = uconn[YSIZE][i];
	    assign osouth[i] = dconn[0][i];
	end

    endgenerate

    /* Connect the shift register endpoints to the input and output of the module */
    assign shiftreg[0][0] = idata;
    assign odata = shiftreg[XSIZE][YSIZE-1];

    /* Do the same to the clock array endpoints */
    assign clkarray[0][0] = iclk;
    assign oclk = clkarray[XSIZE][YSIZE-1];

    /* Propagate clock */
    assign oclk = iclk;

endmodule
`default_nettype wire
