// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- the divider: 64 by 32, which every form of DIVU and DIVS reduces to.
//
// PRM 4 gives DIVS and DIVU four shapes each:
//
//     DIVx.W  <ea>,Dn      32 / 16 -> 16 quotient in the low word,
//                                     16 remainder in the high word
//     DIVx.L  <ea>,Dq      32 / 32 -> 32 quotient, remainder discarded
//     DIVx.L  <ea>,Dr:Dq   64 / 32 -> 32 quotient and 32 remainder
//     DIVxL.L <ea>,Dr:Dq   32 / 32 -> 32 quotient and 32 remainder
//
// All four are this one unit: the caller widens its dividend to 64 bits and its
// divisor to 32, and checks afterwards that the quotient fits where it is going.
//
// Signed division is done on magnitudes and the signs put back. PRM 4: "the sign
// of the remainder is the same as the sign of the dividend", which is the rule
// C99 chose too and not the one a floor division would give.
//
// Restoring division, one quotient bit per clock, 32 clocks. The sequencer
// stalls on `busy` exactly as it stalls on a bus cycle, so the instruction
// timing falls out of the structure rather than being designed --
// doc/timing-divergences.md measures it against UM20 section 8.

module rd68041_divider (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        start,      // one clock, with the operands valid
    input  logic [63:0] dividend,
    input  logic [31:0] divisor,
    input  logic        is_signed,

    output logic        busy,
    output logic [31:0] quotient,
    output logic [31:0] remainder,
    output logic        div_zero,   // the divisor was zero: PRM 4 says trap
    output logic        overflow,   // the quotient does not fit in 32 bits
    // The magnitude of the quotient and the sign it is to carry, so that the
    // caller can ask whether it fits where it is GOING: the word forms have
    // sixteen bits to put it in, and a signed long has thirty-one plus a sign.
    output logic [31:0] q_mag,
    output logic        q_neg
);

  // The signs to put back.
  logic        sgn_num, sgn_den;

  assign sgn_num = is_signed & dividend[63];
  assign sgn_den = is_signed & divisor[31];

  logic [63:0] num_abs;
  logic [31:0] den_abs;
  assign num_abs = sgn_num ? (~dividend + 64'd1) : dividend;
  assign den_abs = sgn_den ? (~divisor  + 32'd1) : divisor;

  logic [32:0] acc;       // the running remainder, one bit wider than the divisor
  logic [63:0] rem_num;   // what is left of the dividend to shift in
  logic [31:0] quo;
  logic  [5:0] iter;
  logic        run;
  logic        sq, sr;    // the signs the result is to carry
  logic [31:0] den_q;
  logic        ovf_q, dz_q;

  logic [32:0] acc_next;
  logic [32:0] acc_sub;
  assign acc_next = {acc[31:0], rem_num[63]};
  assign acc_sub  = acc_next - {1'b0, den_q};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc     <= '0;
      rem_num <= '0;
      quo     <= '0;
      iter    <= '0;
      run     <= 1'b0;
      sq      <= 1'b0;
      sr      <= 1'b0;
      den_q   <= '0;
      ovf_q   <= 1'b0;
      dz_q    <= 1'b0;
    end else if (start) begin
      den_q   <= den_abs;
      sq      <= sgn_num ^ sgn_den;
      sr      <= sgn_num;
      dz_q    <= (divisor == 32'd0);
      // The quotient needs more than thirty-two bits exactly when the top half
      // of the dividend is already at least the divisor. Checked here rather
      // than after, because on overflow PRM 4 leaves the operands alone and the
      // thirty-two clocks would be spent for nothing.
      ovf_q   <= (divisor != 32'd0) && (num_abs[63:32] >= den_abs);
      acc     <= {1'b0, num_abs[63:32]};
      rem_num <= {num_abs[31:0], 32'd0};
      quo     <= '0;
      iter    <= 6'd32;
      run     <= (divisor != 32'd0)
                 && !((num_abs[63:32] >= den_abs));
    end else if (run) begin
      if (acc_sub[32] == 1'b0) begin      // no borrow: the divisor went in
        acc <= acc_sub;
        quo <= {quo[30:0], 1'b1};
      end else begin
        acc <= acc_next;
        quo <= {quo[30:0], 1'b0};
      end
      rem_num <= {rem_num[62:0], 1'b0};
      iter    <= iter - 6'd1;
      if (iter == 6'd1) run <= 1'b0;
    end
  end

  assign busy      = run;
  assign div_zero  = dz_q;
  assign overflow  = ovf_q;
  assign q_mag     = quo;
  assign q_neg     = sq;
  assign quotient  = sq ? (~quo + 32'd1) : quo;
  assign remainder = sr ? (~acc[31:0] + 32'd1) : acc[31:0];

  logic unused_div;
  assign unused_div = &{1'b1, acc[32]};

endmodule
