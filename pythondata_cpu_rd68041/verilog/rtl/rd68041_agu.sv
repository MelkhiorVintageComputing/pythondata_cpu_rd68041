// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The effective-address adder of the <ea> calculate stage: base + scaled index
// + displacement, the one sum every addressing mode reduces to (PRM 2.2):
//
//   (An), (An)+, -(An)          base = An (or An - size), no index, d = 0
//   (d16,An), (d16,PC)          d = the sign-extended displacement
//   (d8,An,Xn), (bd,An,Xn)      index = Xn.W sign-extended or Xn.L, times 1/2/4/8
//   ([bd,An,Xn],od) ...         two passes: the pre- or post-indexed sum, then od
//   (xxx).W, (xxx).L, #imm      base = 0
//
// Any operand may be suppressed by the caller (zero). Combinational: the stage
// registers the result, which is also the data cache's RAM address
// (doc/microarch.md).

module rd68041_agu (
    input  logic [31:0] base,
    input  logic [31:0] index,      // already the register's value
    input  logic        index_w,    // Xn.W: sign-extend its low word
    input  logic  [1:0] scale,      // 0-3: times 1, 2, 4, 8
    input  logic [31:0] disp,       // already sign-extended
    output logic [31:0] ea
);

  logic [31:0] x;
  always_comb begin
    x = index_w ? {{16{index[15]}}, index[15:0]} : index;
    x = x << scale;
  end

  assign ea = base + x + disp;

endmodule
