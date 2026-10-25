// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The storage of one 4 KB cache, instruction or data: four ways of 64 sets of
// 16-byte lines (UM 4.1), and the lookups on it. No policy lives here -- what is
// filled, pushed or invalidated, and when, is the cache controller's (M9); this
// is the arrays and the comparators, with the ports doc/microarch.md gives them:
//
//   data   one rd68041_bram per way, 128 words of 64 bits: a half-line per word
//          (UM 4.4.3, "physical address bit 3 selects either the upper or lower
//          half-line"). Port A is the core's read; port B the fills, pushes,
//          snoops and stores. Both registered: the address goes in on the clock
//          before the data comes out.
//   tags   PA31-PA10 (UM 4.1: "an address tag consisting of the upper 22 bits of
//          the physical address"), LUT RAM read combinationally at three set
//          indices at once: the core's (port 0), the snooper's (port 1), and the
//          controller's (port 2: pushes, CPUSH and CINV by set).
//   dirty  four bits per line, one per long word (UM 4.1, Figure 4-2), read with
//          the tags. Built only for the data cache (DIRTY = 1).
//   valid  a flip-flop per line, reset (doc/microarch.md: the part's reset does
//          not invalidate; a register with no reset value is not allowed here),
//          and cleared all at once for CINVA/CPUSHA.
//
// The core's and the snooper's ports compare the four tags of their set with a
// physical tag they are given and say which way hits.

module rd68041_cache_arr #(
    parameter bit DIRTY = 1'b1
) (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          crst,

    // Port 0: the core's lookup -------------------------------------------
    input  logic   [5:0]  p0_set,
    input  logic  [31:10] p0_ptag,
    output logic          p0_hit,
    output logic   [1:0]  p0_way,
    output logic   [3:0]  p0_dirty,
    output logic   [3:0]  p0_valid,     // the four ways' valid bits, for replacement

    // Port 1: the snooper's -----------------------------------------------
    input  logic   [5:0]  p1_set,
    input  logic  [31:10] p1_ptag,
    output logic          p1_hit,
    output logic   [1:0]  p1_way,
    output logic   [3:0]  p1_dirty,

    // Port 2: by set and way, for the controller --------------------------
    input  logic   [5:0]  p2_set,
    input  logic   [1:0]  p2_way,
    output logic  [31:10] p2_tag,
    output logic          p2_valid,
    output logic   [3:0]  p2_dirty,

    // Tag, valid and dirty writes -------------------------------------------
    input  logic          t_we,         // write the tag and set or clear valid
    input  logic   [5:0]  t_set,
    input  logic   [1:0]  t_way,
    input  logic  [31:10] t_tag,
    input  logic          t_valid,
    input  logic          d_we,         // write the dirty bits
    input  logic   [5:0]  d_set,
    input  logic   [1:0]  d_way,
    input  logic   [3:0]  d_bits,
    input  logic          inv_all,      // every line invalid (CINVA, CPUSHA's end)

    // Data, port A: the core's read, at {set, A3} --------------------------
    input  logic   [6:0]  a_addr,
    output logic [255:0]  a_rdata,      // way w in [64w+63:64w]

    // Data, port B: read or write one way, at {set, A3} ----------------------
    input  logic   [6:0]  b_addr,
    input  logic   [1:0]  b_way,
    input  logic   [7:0]  b_we,         // byte enables, 7 the most significant
    input  logic  [63:0]  b_wdata,
    output logic [255:0]  b_rdata
);

  // --- data ------------------------------------------------------------------
  for (genvar w = 0; w < 4; w++) begin : g_data
    rd68041_bram #(.DEPTH(128)) u_ram (
        .clk     (clk),
        .rst_n   (rst_n),
        .a_addr  (a_addr),
        .a_we    (8'h00),
        .a_wdata (64'h0),
        .a_rdata (a_rdata[w*64 +: 64]),
        .b_addr  (b_addr),
        .b_we    ((b_way == w[1:0]) ? b_we : 8'h00),
        .b_wdata (b_wdata),
        .b_rdata (b_rdata[w*64 +: 64])
    );
  end

  // --- tags and dirty bits: LUT RAM, one write port, three read ports ------
  logic [21:0] t0 [0:3], t1 [0:3], t2 [0:3];
  logic  [3:0] dy0 [0:3], dy1 [0:3], dy2 [0:3];
  logic [63:0] v_q [0:3];

  for (genvar w = 0; w < 4; w++) begin : g_tag
    logic [21:0] tag_ram [0:63];
    always_ff @(posedge clk) begin
      if (t_we && t_way == w[1:0]) tag_ram[t_set] <= t_tag;
    end
    assign t0[w] = tag_ram[p0_set];
    assign t1[w] = tag_ram[p1_set];
    assign t2[w] = tag_ram[p2_set];

    if (DIRTY) begin : g_dirty
      logic [3:0] dirty_ram [0:63];
      always_ff @(posedge clk) begin
        if (d_we && d_way == w[1:0]) dirty_ram[d_set] <= d_bits;
      end
      assign dy0[w] = dirty_ram[p0_set];
      assign dy1[w] = dirty_ram[p1_set];
      assign dy2[w] = dirty_ram[p2_set];
    end else begin : g_nodirty
      assign dy0[w] = 4'b0000;
      assign dy1[w] = 4'b0000;
      assign dy2[w] = 4'b0000;
    end

    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        v_q[w] <= '0;
      end else if (crst || inv_all) begin
        v_q[w] <= '0;
      end else if (t_we && t_way == w[1:0]) begin
        v_q[w][t_set] <= t_valid;
      end
    end
  end

  // --- the comparators -------------------------------------------------------
  logic [3:0] h0, h1;
  for (genvar w = 0; w < 4; w++) begin : g_cmp
    assign h0[w] = v_q[w][p0_set] && (t0[w] == p0_ptag);
    assign h1[w] = v_q[w][p1_set] && (t1[w] == p1_ptag);
    assign p0_valid[w] = v_q[w][p0_set];
  end

  // A line is in one way at most, so the hits are one-hot or none.
  function automatic logic [1:0] enc(input logic [3:0] h);
    case (h)
      4'b0010: enc = 2'd1;
      4'b0100: enc = 2'd2;
      4'b1000: enc = 2'd3;
      default: enc = 2'd0;
    endcase
  endfunction

  assign p0_hit   = |h0;
  assign p0_way   = enc(h0);
  assign p0_dirty = dy0[p0_way];
  assign p1_hit   = |h1;
  assign p1_way   = enc(h1);
  assign p1_dirty = dy1[p1_way];
  assign p2_tag   = t2[p2_way];
  assign p2_valid = v_q[p2_way][p2_set];
  assign p2_dirty = dy2[p2_way];

  // Without dirty bits (the instruction cache), their write port goes unread.
  logic unused_ok;
  assign unused_ok = &{1'b0, DIRTY ? 1'b0 : &{d_we, d_set, d_way, d_bits}};

endmodule
