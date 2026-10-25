// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The integer register file: D0-D7 and A0-A7 (UM 2.2.1), sixteen 32-bit
// registers, A7 being whichever stack pointer the status register selects (the
// others are held beside it, M8), and four temporaries, T0-T3 (16-19), that
// multi-micro-operation instructions pass values in. Flip-flops rather than
// LUT RAM: five reads (two for <ea> calculate, three for execute -- the third
// for a 64-bit dividend's high half) and
// three writes a clock, and a reset value for every register (CLAUDE.md) -- the
// part leaves them undefined at reset (UM 8), and zero is as good a value as
// any other.
//
// Register numbers are the operation word's: 0-7 the data registers, 8-15 the
// address registers, 16-19 the temporaries. Reads are combinational; a write is
// visible from the next clock. Three write ports, for the three things one
// micro-operation can write (doc/microarch.md): its result (port 0, byte or
// word or long), its second result (port 2: EXG, LINK, UNLK, MUL.L, DIV.L),
// and its address register's update (port 1: (An)+, -(An)). When two name the
// same register the earlier wins -- 0, then 2, then 1: PRM 4 keeps the
// operand read where the update and the result would both go (MOVEA (An)+,An;
// UNLK A7), and LINK A7 keeps the displacement's sum over the push's update.

module rd68041_regfile (
    input  logic        clk,
    input  logic        rst_n,

    input  logic  [4:0] ra, rb, rc, rd, re,
    output logic [31:0] qa, qb, qc, qd, qe,
    output logic [31:0] q15,      // A7, the active stack pointer: the stack swap's (M8)
    output logic [31:0] o0, o1, o2, // what each write port's register holds before the write

    input  logic        we0,
    input  logic  [4:0] wa0,
    input  logic [31:0] wd0,
    input  logic  [3:0] wm0,      // byte enables: a byte or word result leaves the rest (PRM 4)
    input  logic        we1,
    input  logic  [4:0] wa1,
    input  logic [31:0] wd1,
    input  logic        we2,
    input  logic  [4:0] wa2,
    input  logic [31:0] wd2
);

  logic [31:0] r [0:19];

  for (genvar i = 0; i < 20; i++) begin : g_reg
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        r[i] <= 32'h0000_0000;
      end else if (we0 && wa0 == i[4:0]) begin
        if (wm0[3]) r[i][31:24] <= wd0[31:24];
        if (wm0[2]) r[i][23:16] <= wd0[23:16];
        if (wm0[1]) r[i][15:8]  <= wd0[15:8];
        if (wm0[0]) r[i][7:0]   <= wd0[7:0];
      end else if (we2 && wa2 == i[4:0]) begin
        r[i] <= wd2;
      end else if (we1 && wa1 == i[4:0]) begin
        r[i] <= wd1;
      end
    end
  end

  // A number past T3 reads zero.
  assign qa = (ra < 5'd20) ? r[ra] : 32'h0;
  assign qb = (rb < 5'd20) ? r[rb] : 32'h0;
  assign qc = (rc < 5'd20) ? r[rc] : 32'h0;
  assign qd = (rd < 5'd20) ? r[rd] : 32'h0;
  assign qe = (re < 5'd20) ? r[re] : 32'h0;
  assign q15 = r[15];
  assign o0  = (wa0 < 5'd20) ? r[wa0] : 32'h0;
  assign o1  = (wa1 < 5'd20) ? r[wa1] : 32'h0;
  assign o2  = (wa2 < 5'd20) ? r[wa2] : 32'h0;

endmodule
