// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The integer register file: D0-D7 and A0-A7 (UM 2.2.1), sixteen 32-bit
// registers, A7 being whichever stack pointer the status register selects (the
// others are held beside it, M8), and four temporaries, T0-T3 (16-19), that
// multi-micro-operation instructions pass values in. Eight reads (two for <ea>
// calculate, three for execute -- the third for a 64-bit dividend's high half
// -- and what each write port's register holds, for the undo log) and three
// writes a clock.
//
// LUT RAM, not flip-flops (M15, the area review's A2): one bank per write port,
// each read from every read port, and a live-value table that says, per
// register and lane, which bank was written last. Port 0's lanes are [7:0],
// [15:8] and [31:16]: a byte, word or long result (wm0's two high bits move
// together). The table is flip-flops with a reset value, 3, that reads zero --
// the part leaves the registers undefined at reset (UM 8), and zero is as good
// a value as any other -- so nothing the RAM powered up holding can be read
// (CLAUDE.md, tools/reset_audit.py). A7 is also kept in flip-flops, q15, for
// the stack swap (M8), not as a ninth read port.
//
// Register numbers are the operation word's: 0-7 the data registers, 8-15 the
// address registers, 16-19 the temporaries; a number past T3 reads zero. Reads
// are combinational; a write is visible from the next clock. Three write ports,
// for the three things one micro-operation can write (doc/microarch.md): its
// result (port 0, byte or word or long), its second result (port 2: EXG, LINK,
// UNLK, MUL.L, DIV.L), and its address register's update (port 1: (An)+,
// -(An)). When two name the same register the earlier wins -- 0, then 2, then
// 1: PRM 4 keeps the operand read where the update and the result would both
// go (MOVEA (An)+,An; UNLK A7), and LINK A7 keeps the displacement's sum over
// the push's update. A port 0 write of a byte or word leaves the rest of the
// register as it was, the others' writes included.

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

  // --- the banks ----------------------------------------------------------------
  logic  [7:0] b0l [0:31];        // port 0: [7:0]
  logic  [7:0] b0m [0:31];        // ... [15:8]
  logic [15:0] b0h [0:31];        // ... [31:16]
  logic [31:0] b1  [0:31];
  logic [31:0] b2  [0:31];

  always_ff @(posedge clk) begin
    if (we0 && wm0[0]) b0l[wa0] <= wd0[7:0];
    if (we0 && wm0[1]) b0m[wa0] <= wd0[15:8];
    if (we0 && wm0[2]) b0h[wa0] <= wd0[31:16];
    if (we1)           b1[wa1]  <= wd1;
    if (we2)           b2[wa2]  <= wd2;
  end

  // --- the live-value table: per register and lane, 0-2 the bank, 3 zero -------
  logic [1:0] lvt [0:19][0:2];

  for (genvar i = 0; i < 20; i++) begin : g_lvt
    for (genvar l = 0; l < 3; l++) begin : g_lane
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          lvt[i][l] <= 2'd3;
        end else if (we0 && wa0 == i[4:0]) begin
          if (wm0[l]) lvt[i][l] <= 2'd0;
        end else if (we2 && wa2 == i[4:0]) begin
          lvt[i][l] <= 2'd2;
        end else if (we1 && wa1 == i[4:0]) begin
          lvt[i][l] <= 2'd1;
        end
      end
    end
  end

  // --- the reads: each port's bank values, chosen by its lanes' table entries -----
  logic [4:0]  pa [0:7];
  logic [31:0] pq [0:7];
  assign pa[0] = ra;  assign pa[1] = rb;  assign pa[2] = rc;  assign pa[3] = rd;
  assign pa[4] = re;  assign pa[5] = wa0; assign pa[6] = wa1; assign pa[7] = wa2;

  for (genvar p = 0; p < 8; p++) begin : g_rd
    wire  [4:0] a  = pa[p];
    wire        in = (a < 5'd20);
    wire [31:0] v1 = b1[a];
    wire [31:0] v2 = b2[a];
    wire  [1:0] sl = in ? lvt[a[4:0]][0] : 2'd3;
    wire  [1:0] sm = in ? lvt[a[4:0]][1] : 2'd3;
    wire  [1:0] sh = in ? lvt[a[4:0]][2] : 2'd3;
    assign pq[p][7:0]   = (sl == 2'd0) ? b0l[a] : (sl == 2'd1) ? v1[7:0]   : (sl == 2'd2) ? v2[7:0]   : 8'h0;
    assign pq[p][15:8]  = (sm == 2'd0) ? b0m[a] : (sm == 2'd1) ? v1[15:8]  : (sm == 2'd2) ? v2[15:8]  : 8'h0;
    assign pq[p][31:16] = (sh == 2'd0) ? b0h[a] : (sh == 2'd1) ? v1[31:16] : (sh == 2'd2) ? v2[31:16] : 16'h0;
  end

  assign qa = pq[0];
  assign qb = pq[1];
  assign qc = pq[2];
  assign qd = pq[3];
  assign qe = pq[4];
  assign o0 = pq[5];
  assign o1 = pq[6];
  assign o2 = pq[7];

  // --- A7 in flip-flops, written as the table says --------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q15 <= 32'h0000_0000;
    end else if (we0 && wa0 == 5'd15) begin
      if (wm0[0]) q15[7:0]   <= wd0[7:0];
      if (wm0[1]) q15[15:8]  <= wd0[15:8];
      if (wm0[2]) q15[31:16] <= wd0[31:16];
    end else if (we2 && wa2 == 5'd15) begin
      q15 <= wd2;
    end else if (we1 && wa1 == 5'd15) begin
      q15 <= wd1;
    end
  end

  logic unused_ok;
  assign unused_ok = wm0[3];

endmodule
