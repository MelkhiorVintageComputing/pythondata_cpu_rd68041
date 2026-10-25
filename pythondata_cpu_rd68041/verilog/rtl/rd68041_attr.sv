// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// An access's attributes from the transparent translation registers (UM
// 3.1.3, 3.4): one instance in each memory unit, its two TTRs (ITT0/1 or
// DTT0/1). Combinational.
//
// A TTR matches when it is enabled, A31-A24 equal its base outside its mask,
// and its S field takes the access's mode (00 user only, 01 supervisor only,
// 1x either); TT0 wins when both do. "The TTRs operate independently of the
// E-bit in the TCR" (UM 3.1.3), so they apply with translation off too. An
// access no TTR matches with translation off has the defaults (UM 3.1.2):
// cachable write-through, not write protected, UPA zero; with translation on,
// the ATC's entry supplies them (rd68041_dmu, rd68041_ifu).

module rd68041_attr (
    input  logic [31:24] addr,
    input  logic         sup,
    input  logic [31:0]  ttr0,
    input  logic [31:0]  ttr1,
    output logic  [1:0]  cm,      // 00 write-through, 01 copyback, 10 serialised, 11 inhibited
    output logic  [1:0]  upa,     // U1, U0 for UPA1-UPA0
    output logic         wp,      // write protected
    output logic         tt       // a TTR matched
);

  function automatic logic hit(input logic [31:13] t, input logic [31:24] a, input logic s);
    hit = t[15] && (((a ^ t[31:24]) & ~t[23:16]) == 8'h00) &&
          (t[14] || (t[13] == s));
  endfunction

  wire h0 = hit(ttr0[31:13], addr, sup);
  wire h1 = hit(ttr1[31:13], addr, sup);

  always_comb begin
    if (h0) begin
      cm = ttr0[6:5]; upa = ttr0[9:8]; wp = ttr0[2];
    end else if (h1) begin
      cm = ttr1[6:5]; upa = ttr1[9:8]; wp = ttr1[2];
    end else begin
      cm = 2'b00; upa = 2'b00; wp = 1'b0;
    end
  end
  assign tt = h0 || h1;

  // Bits 12-10, 7, 4, 3, 1 and 0 of a TTR read as zero (UM 3.1.3).
  logic unused_ok;
  assign unused_ok = &{1'b0, ttr0[12:10], ttr0[7], ttr0[4:3], ttr0[1:0],
                       ttr1[12:10], ttr1[7], ttr1[4:3], ttr1[1:0]};

endmodule
