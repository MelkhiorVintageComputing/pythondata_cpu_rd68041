// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The floating-point unit's internal number and the data path's operations
// (doc/fpu.md).

package rd68041_fpu_pkg;

  // An operand's class (and the frame's tags, UM 9.8: STAG/DTAG 000-101).
  localparam logic [2:0] C_NORM  = 3'd0;
  localparam logic [2:0] C_ZERO  = 3'd1;
  localparam logic [2:0] C_INF   = 3'd2;
  localparam logic [2:0] C_NAN   = 3'd3;
  localparam logic [2:0] C_DNRMX = 3'd4;   // an extended denormal or unnormal
  localparam logic [2:0] C_DNRMS = 3'd5;   // a single or double denormal

  // An unpacked number: the exponent biased by 16383 and signed, wide enough
  // for any intermediate result; the mantissa with its explicit integer bit.
  typedef struct packed {
    logic [2:0]  cls;
    logic        snan;     // a NaN, signalling
    logic        sign;
    logic [17:0] exp;
    logic [63:0] man;
  } fpn_t;
  localparam int FPN_W = 3 + 1 + 1 + 18 + 64;

  // The data path's operations.
  localparam logic [3:0] D_MOVE = 4'd0;    // round the source (FMOVE, FABS, FNEG in)
  localparam logic [3:0] D_ADD  = 4'd1;    // dest + src
  localparam logic [3:0] D_SUB  = 4'd2;    // dest - src
  localparam logic [3:0] D_MUL  = 4'd3;
  localparam logic [3:0] D_DIV  = 4'd4;    // dest / src
  localparam logic [3:0] D_SQRT = 4'd5;    // of the source
  localparam logic [3:0] D_SMUL = 4'd6;    // FSGLMUL
  localparam logic [3:0] D_SDIV = 4'd7;    // FSGLDIV
  localparam logic [3:0] D_INT  = 4'd8;    // the source to an integer (FMOVE to B, W, L)
  localparam logic [3:0] D_CMP  = 4'd9;    // dest - src, for the condition codes only

  // Rounding precisions (FPCR PREC, UM Table 9-1) and modes.
  localparam logic [1:0] P_X = 2'd0;
  localparam logic [1:0] P_S = 2'd1;
  localparam logic [1:0] P_D = 2'd2;
  localparam logic [1:0] R_N = 2'd0;
  localparam logic [1:0] R_Z = 2'd1;
  localparam logic [1:0] R_M = 2'd2;
  localparam logic [1:0] R_P = 2'd3;

  // The extended bias.
  localparam logic [17:0] BIAS = 18'd16383;

  // The FPU's micro-operations (doc/fpu.md, "Micro-operations"): fk in the uop.
  localparam logic [3:0] FK_NONE = 4'd0;
  localparam logic [3:0] FK_IN   = 4'd1;   // an operand's long word fidx into the staging
  localparam logic [3:0] FK_OP   = 4'd2;   // the general operation of fcmd (its last long in B)
  localparam logic [3:0] FK_OUT  = 4'd3;   // FMOVE out: long word fidx of the result
  localparam logic [3:0] FK_CRW  = 4'd4;   // FPCR, FPSR or FPIAR (fidx 2, 1, 0) from B
  localparam logic [3:0] FK_CRR  = 4'd5;   // ... read
  localparam logic [3:0] FK_RGW  = 4'd6;   // FMOVEM in: long fidx of FP register freg
  localparam logic [3:0] FK_RGR  = 4'd7;   // FMOVEM out: ... read
  localparam logic [3:0] FK_COND = 4'd8;   // the predicate fcmd[5:0] (FBcc, FScc, FDBcc, FTRAPcc)
  localparam logic [3:0] FK_SAVE = 4'd9;   // FSAVE: long word fidx of the frame
  localparam logic [3:0] FK_RST  = 4'd10;  // FRESTORE: long word fidx, from B
  localparam logic [3:0] FK_REND = 4'd11;  // FRESTORE: the frame is in, its state taken

  // The state FSAVE describes (UM 9.8).
  localparam logic [1:0] ST_NULL = 2'd0;
  localparam logic [1:0] ST_IDLE = 2'd1;
  localparam logic [1:0] ST_UNIM = 2'd2;
  localparam logic [1:0] ST_BUSY = 2'd3;

  // ---------------------------------------------------------------------------
  // The memory formats (UM 9.3, Tables 9-3 to 9-5), in and out
  // ---------------------------------------------------------------------------
  // (Each function builds its result in one assignment: yosys sees a latch in
  // a struct assigned whole and then a field on some paths.)
  function automatic fpn_t fp_mk(input logic [2:0] c, input logic sg, input logic [17:0] e,
                                 input logic [63:0] m);
    fp_mk = {c, 1'b0, sg, e, m};
  endfunction
  function automatic fpn_t fp_nan(input logic sg, input logic [63:0] m, input logic sn);
    fp_nan = {C_NAN, sn, sg, 18'h7FFF, m};
  endfunction

  // Extended: sign, 15-bit exponent, 64-bit mantissa with its integer bit. An
  // exponent of 0 with the integer bit set is a normal number (Table 9-5); with
  // it clear and a nonzero mantissa, a denormal; a nonzero exponent with the
  // integer bit clear, an unnormal -- both unsupported (UM 9.6.2).
  function automatic fpn_t unpack_x(input logic [79:0] v);
    logic        sg;
    logic [14:0] e;
    logic [63:0] m;
    sg = v[79]; e = v[78:64]; m = v[63:0];
    if (e == 15'h7FFF) begin
      if (m[62:0] == 63'h0) unpack_x = fp_mk(C_INF, sg, 18'h7FFF, m);
      else                  unpack_x = fp_nan(sg, m, !m[62]);
    end else if (m == 64'h0) begin
      // A zero, or an unnormal zero (UM 9.4: "converted to a normalized zero").
      unpack_x = fp_mk((e == 15'h0) ? C_ZERO : C_DNRMX, sg, {3'b000, e}, m);
    end else if (!m[63]) begin
      unpack_x = fp_mk(C_DNRMX, sg, {3'b000, e}, m);
    end else begin
      unpack_x = fp_mk(C_NORM, sg, {3'b000, e}, m);
    end
  endfunction

  // Single and double: the fraction behind an implicit integer bit.
  function automatic fpn_t unpack_s(input logic [31:0] v);
    logic        sg;
    logic [7:0]  e;
    logic [22:0] f;
    sg = v[31]; e = v[30:23]; f = v[22:0];
    if (e == 8'hFF) begin
      if (f == 23'h0) unpack_s = fp_mk(C_INF, sg, 18'h7FFF, 64'h0);
      else            unpack_s = fp_nan(sg, {1'b1, f, 40'h0}, !f[22]);
    end else if (e == 8'h0) begin
      // A denormal, kept in UM Figure 9-9's form for the frame.
      unpack_s = fp_mk((f == 23'h0) ? C_ZERO : C_DNRMS, sg, 18'h0, {1'b0, f, 40'h0});
    end else begin
      unpack_s = fp_mk(C_NORM, sg, {10'h0, e} + 18'd16256, {1'b1, f, 40'h0});
    end
  endfunction
  function automatic fpn_t unpack_d(input logic [63:0] v);
    logic        sg;
    logic [10:0] e;
    logic [51:0] f;
    sg = v[63]; e = v[62:52]; f = v[51:0];
    if (e == 11'h7FF) begin
      if (f == 52'h0) unpack_d = fp_mk(C_INF, sg, 18'h7FFF, 64'h0);
      else            unpack_d = fp_nan(sg, {1'b1, f, 11'h0}, !f[51]);
    end else if (e == 11'h0) begin
      unpack_d = fp_mk((f == 52'h0) ? C_ZERO : C_DNRMS, sg, 18'h0, {1'b0, f, 11'h0});
    end else begin
      unpack_d = fp_mk(C_NORM, sg, {7'h0, e} + 18'd15360, {1'b1, f, 11'h0});
    end
  endfunction

  // An integer, already sign-extended to 32 bits: exact in extended.
  function automatic logic [5:0] lzc32(input logic [31:0] v);
    lzc32 = 6'd32;
    for (int k = 0; k < 32; k++) if (v[k]) lzc32 = 6'(31 - k);
  endfunction
  function automatic fpn_t unpack_i(input logic [31:0] v);
    logic [31:0] mag;
    logic [5:0]  lz;
    mag = v[31] ? (32'h0 - v) : v;
    lz  = lzc32(mag);
    if (mag == 32'h0) unpack_i = fp_mk(C_ZERO, 1'b0, 18'h0, 64'h0);
    else unpack_i = fp_mk(C_NORM, v[31], 18'd16414 - {12'h0, lz}, {mag << lz, 32'h0});
  endfunction

  // Out: the number as the register (extended) holds it.
  function automatic logic [79:0] pack_x(input fpn_t x);
    case (x.cls)
      C_ZERO:  pack_x = {x.sign, 15'h0, 64'h0};
      C_INF:   pack_x = {x.sign, 15'h7FFF, 64'h0};
      C_NAN:   pack_x = {x.sign, 15'h7FFF, x.man};
      default: pack_x = {x.sign, x.exp[14:0], x.man};
    endcase
  endfunction
  // ... and as a single or double, rounded to it already and in its range.
  function automatic logic [31:0] pack_s(input fpn_t x);
    logic [17:0] e;
    e = x.exp - 18'd16256;
    case (x.cls)
      C_ZERO:  pack_s = {x.sign, 31'h0};
      C_INF:   pack_s = {x.sign, 8'hFF, 23'h0};
      C_NAN:   pack_s = {x.sign, 8'hFF, x.man[62:40]};
      default: pack_s = {x.sign, e[7:0], x.man[62:40]};
    endcase
  endfunction
  function automatic logic [63:0] pack_d(input fpn_t x);
    logic [17:0] e;
    e = x.exp - 18'd15360;
    case (x.cls)
      C_ZERO:  pack_d = {x.sign, 63'h0};
      C_INF:   pack_d = {x.sign, 11'h7FF, 52'h0};
      C_NAN:   pack_d = {x.sign, 11'h7FF, x.man[62:11]};
      default: pack_d = {x.sign, e[10:0], x.man[62:11]};
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // The general operations' opmodes (opclasses 000 and 010, FMOVECR apart):
  // Table 9-10's, unimplemented (vector 11, format $2); and every opmode PRM 5
  // has, those and the ones built (rd68041_fpu). Any other is an F-line
  // illegal instruction. The decoder of the MC68LC040 asks the second too: a
  // legal one is format $4 there, an illegal one F-line (UM A.5.1).
  function automatic logic unimp_op(input logic [6:0] m);
    case (m)
      7'h01, 7'h02, 7'h03, 7'h06, 7'h08, 7'h09, 7'h0A, 7'h0C, 7'h0D, 7'h0E, 7'h0F,
      7'h10, 7'h11, 7'h12, 7'h14, 7'h15, 7'h16, 7'h19, 7'h1C, 7'h1D, 7'h1E, 7'h1F,
      7'h21, 7'h25, 7'h26, 7'h30, 7'h31, 7'h32, 7'h33, 7'h34, 7'h35, 7'h36, 7'h37:
        unimp_op = 1'b1;
      default: unimp_op = 1'b0;
    endcase
  endfunction
  function automatic logic legal_op(input logic [6:0] m);
    case (m)
      7'h00, 7'h40, 7'h44, 7'h04, 7'h41, 7'h45, 7'h18, 7'h58, 7'h5C, 7'h1A, 7'h5A, 7'h5E,
      7'h20, 7'h60, 7'h64, 7'h22, 7'h62, 7'h66, 7'h23, 7'h63, 7'h67, 7'h28, 7'h68, 7'h6C,
      7'h24, 7'h27, 7'h38, 7'h3A:
        legal_op = 1'b1;
      default: legal_op = unimp_op(m);
    endcase
  endfunction

endpackage
