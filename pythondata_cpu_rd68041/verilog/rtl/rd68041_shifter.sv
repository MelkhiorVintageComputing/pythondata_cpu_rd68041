// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- the shifts and rotates: ASL, ASR, LSL, LSR, ROL, ROR, ROXL, ROXR.
//
// PRM 4 gives each of the eight its own page, and the condition codes are where
// they differ. The table below is the specification; everything in this module
// is one line of it.
//
//          result                      X          C                    V
//   ASL    shifted left, zero filled   last out   last out (0 if n=0)  sign ever changed
//   ASR    shifted right, sign filled  last out   last out (0 if n=0)  0
//   LSL    shifted left, zero filled   last out   last out (0 if n=0)  0
//   LSR    shifted right, zero filled  last out   last out (0 if n=0)  0
//   ROL    rotated left                --         last out (0 if n=0)  0
//   ROR    rotated right               --         last out (0 if n=0)  0
//   ROXL   rotated left through X      last out   last out (X if n=0)  0
//   ROXR   rotated right through X     last out   last out (X if n=0)  0
//
// A count of zero leaves X alone for every one of them, and clears C for all
// but the two that rotate through X, where C becomes X. That is the row most
// easily got wrong, and a shift by a register can be zero at run time.
//
// The counts: PRM 4 takes the register forms modulo 64, and ROXL and ROXR
// modulo the operand size PLUS ONE, because X is part of the ring.
//
// Purely combinational. A 68020 takes a clock or two per bit and this does not;
// doc/timing-divergences.md records the difference.

module rd68041_shifter (
    input  logic [31:0] op,
    input  logic  [5:0] count,     // 0..63, already reduced by the caller
    input  logic  [1:0] size,      // 0 byte, 1 word, 2 long
    input  logic  [1:0] kind,      // 0 arithmetic, 1 logical, 2 through X, 3 rotate
    input  logic        left,
    input  logic        x_in,

    output logic [31:0] res,
    output logic        c_out,
    output logic        v_out,
    output logic        x_out,
    output logic        x_write    // whether X is written at all
);

  // The operand width, and the operand in it.
  logic [5:0]  w;
  logic [31:0] opw, ones;
  always_comb begin
    unique case (size)
      2'd0:    w = 6'd8;
      2'd1:    w = 6'd16;
      default: w = 6'd32;
    endcase
  end
  assign ones = (w == 6'd32) ? 32'hFFFF_FFFF : ((32'd1 << w) - 32'd1);
  assign opw  = op & ones;

  // Every selection of "the bit at the top of the operand" goes through a case
  // on the size rather than a variable bit-select. iverilog does not support a
  // variable select inside an always_comb and warns that ALL bits will be
  // included -- a note, not an error, and a wrong answer. doc/coding-standard.md
  // has the rule.
  // Written as a three-input mux rather than a function taking the whole word,
  // because a function that uses three of its argument's thirty-two bits is an
  // unused-bits warning Verilator makes an error.
  logic sign, rot_top;
  always_comb begin
    unique case (size)
      2'd0:    sign = opw[7];
      2'd1:    sign = opw[15];
      default: sign = opw[31];
    endcase
  end

  // Sign extended to the full width, which is what an arithmetic right shift
  // needs and what nothing else may see.
  logic [31:0] op_sx;
  assign op_sx = sign ? (op | ~ones) : opw;

  // --------------------------------------------------------------------
  // The shifts. Sixty-four bits wide so that a count larger than the operand
  // still has somewhere to put the bit that left last.
  // --------------------------------------------------------------------
  logic [63:0] lsh;
  logic [31:0] rsh_l, rsh_a;
  assign lsh   = {32'd0, opw} << count;
  assign rsh_l = opw   >> count;
  assign rsh_a = $signed(op_sx) >>> count;

  // The last bit out. Shifting left by n, it is bit w-n of the operand, which
  // is bit w of the widened result; shifting right by n it is bit n-1, which is
  // bit 0 of a shift by n-1. A count of zero has no last bit and is handled
  // where C is assembled.
  logic c_left, c_right_l, c_right_a;
  always_comb begin
    unique case (size)
      2'd0:    c_left = lsh[8];
      2'd1:    c_left = lsh[16];
      default: c_left = lsh[32];
    endcase
  end
  logic [31:0] last_l, last_a;
  assign last_l = (count == 6'd0) ? 32'd0 : (opw >> (count - 6'd1));
  assign last_a = (count == 6'd0) ? 32'd0
                                  : 32'($signed(op_sx) >>> (count - 6'd1));
  assign c_right_l = (count == 6'd0) ? 1'b0 : last_l[0];
  assign c_right_a = (count == 6'd0) ? 1'b0 : last_a[0];

  // --------------------------------------------------------------------
  // The rotates. Modulo the width, and modulo the width plus one when X is in
  // the ring.
  // --------------------------------------------------------------------
  logic [5:0]  rot_n;
  logic [5:0]  rox_n;
  logic [63:0] rot_wide, rox_wide;
  logic [32:0] ext;

  // The width is a power of two, so the rotate count reduces with a mask. The
  // width PLUS ONE is not, so that one reduces with a ladder of subtractions --
  // three of them at most, and a great deal cheaper than the divider a `%` by a
  // non-constant would infer.
  logic [5:0] m9, m17, m33;
  always_comb begin
    m9 = count;
    if (m9 >= 6'd36) m9 = m9 - 6'd36;
    if (m9 >= 6'd18) m9 = m9 - 6'd18;
    if (m9 >= 6'd9)  m9 = m9 - 6'd9;
    m17 = count;
    if (m17 >= 6'd34) m17 = m17 - 6'd34;
    if (m17 >= 6'd17) m17 = m17 - 6'd17;
    m33 = (count >= 6'd33) ? (count - 6'd33) : count;
  end

  assign rot_n = count & (w - 6'd1);
  always_comb begin
    unique case (size)
      2'd0:    rox_n = m9;
      2'd1:    rox_n = m17;
      default: rox_n = m33;
    endcase
  end

  // Both rings are rotated LEFT, and a right rotation is a left one by the
  // complement. Writing two rotators would have been the obvious way and would
  // have cost twice the logic; not writing the complement at all was the bug,
  // and it made ROR and ROXR rotate the wrong way while ROL and ROXL passed.
  logic [5:0] rot_amt, rox_amt;
  assign rot_amt = left ? rot_n
                        : ((rot_n == 6'd0) ? 6'd0 : (w - rot_n));
  assign rox_amt = left ? rox_n
                        : ((rox_n == 6'd0) ? 6'd0 : ((w + 6'd1) - rox_n));

  assign rot_wide = ({32'd0, opw} << rot_amt) | {32'd0, (opw >> (w - rot_amt))};
  assign ext      = {1'b0, opw} | ({32'd0, x_in} << w);
  assign rox_wide = ({31'd0, ext} << rox_amt)
                  | {31'd0, (ext >> ((w + 6'd1) - rox_amt))};

  // (x >> w) with a count equal to the width is a shift by zero in Verilog's
  // arithmetic only if the count is reduced first, which rot_n and rox_n are.
  logic [31:0] rot_res, rox_res;
  always_comb begin
    unique case (size)
      2'd0:    rot_top = rot_res[7];
      2'd1:    rot_top = rot_res[15];
      default: rot_top = rot_res[31];
    endcase
  end
  logic        rox_bit;
  assign rot_res = rot_wide[31:0] & ones;
  assign rox_res = rox_wide[31:0] & ones;
  always_comb begin
    unique case (size)
      2'd0:    rox_bit = rox_wide[8];
      2'd1:    rox_bit = rox_wide[16];
      default: rox_bit = rox_wide[32];
    endcase
  end

  // --------------------------------------------------------------------
  // The overflow bit, which only ASL has.
  //
  // PRM 4: "V is set if the most significant bit is changed at any time during
  // the shift operation". Equivalently the top count+1 bits of the operand are
  // not all the same -- and once the count reaches the width, any non-zero
  // operand has had its sign bit change.
  // --------------------------------------------------------------------
  logic [31:0] top_mask, top_bits;
  logic        asl_v;
  always_comb begin
    if (count >= w) begin
      top_mask = ones;
      top_bits = opw;
      asl_v    = (opw != 32'd0);
    end else begin
      top_mask = ones & ~((32'd1 << (w - count - 6'd1)) - 32'd1);  // the top count+1 bits
      top_bits = opw & top_mask;
      asl_v    = (top_bits != 32'd0) && (top_bits != top_mask);
    end
  end

  // --------------------------------------------------------------------
  // And the selection.
  // --------------------------------------------------------------------
  always_comb begin
    res     = 32'd0;
    c_out   = 1'b0;
    v_out   = 1'b0;
    x_out   = x_in;
    x_write = (count != 6'd0);
    unique case (kind)
      2'd0: begin                                   // ASL and ASR
        res   = left ? (lsh[31:0] & ones) : (rsh_a & ones);
        c_out = (count == 6'd0) ? 1'b0 : (left ? c_left : c_right_a);
        v_out = left ? asl_v : 1'b0;
        x_out = c_out;
      end
      2'd1: begin                                   // LSL and LSR
        res   = left ? (lsh[31:0] & ones) : (rsh_l & ones);
        c_out = (count == 6'd0) ? 1'b0 : (left ? c_left : c_right_l);
        x_out = c_out;
      end
      2'd2: begin                                   // ROXL and ROXR
        res   = rox_res;
        // A count of zero leaves the ring alone and puts X into C, which is the
        // one place a zero count is not simply "nothing happened".
        c_out = (rox_n == 6'd0) ? x_in : rox_bit;
        x_out = c_out;
        x_write = (count != 6'd0);
      end
      default: begin                                // ROL and ROR
        res     = rot_res;
        c_out   = (count == 6'd0) ? 1'b0
                                  : (left ? rot_res[0] : rot_top);
        x_write = 1'b0;                             // X is never touched
      end
    endcase
  end

  // The wide intermediates exist so that a count larger than the operand still
  // has somewhere to put the bit that left last; the bits above that are not an
  // answer to anything.
  logic unused_shifter;
  assign unused_shifter = &{1'b1, lsh[63:33], rot_wide[63:32], rox_wide[63:33],
                            last_l[31:1], last_a[31:1]};

endmodule
