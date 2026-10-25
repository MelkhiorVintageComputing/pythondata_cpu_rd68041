// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The processor reset: RSTI synchronised, the 128-clock internal hold, and the
// modes sampled at reset.
//
// UM 7.10: "RSTI is internally synchronized for two BCLKS before being used",
// and "once RSTI negates, the processor is internally held in reset for another
// 128 clock cycles. During the reset period, all signals that can be, are
// three-stated, and the rest are driven to their inactive state." `crst` is that
// internal reset: asserted from the clock the synchronised RSTI is asserted to
// the 128th clock after it negates.
//
// UM 7.10: "the levels on CDIS, MDIS, and IPL2-IPL0 are used to selectively
// enable the special modes of operation when RSTI is negated", and
// specifications 53 and 54 hold them from 20 ns before RSTI negates to 2 ns
// after. They are captured on every clock while the first synchroniser rank
// still reads RSTI asserted, so the value kept is the one on the pins at the
// edge on which RSTI was first seen negated -- up to a clock after it negated,
// outside 54's window. No rising-edge capture can be inside a window timed from
// an asynchronous edge and narrower than a clock (doc/ac-timing.md). Only the
// normal modes are built, so the levels are only reported, and nothing else
// reads them (doc/divergences.md, "The mode selects are recorded, not met").

module rd68041_rstctl (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rsti_n_i,
    input  logic       cdis_n_i,
    input  logic       mdis_n_i,
    input  logic [2:0] ipl_n_i,
    output logic       crst,        // the internal reset, active high
    output logic       mode_mux,    // CDIS low at reset: multiplexed bus mode asked for
    output logic       mode_dle,    // MDIS low at reset: DLE mode asked for
    output logic [2:0] mode_ipl     // the IPL levels at reset (output buffer selection)
);

  logic rank0, rank1;
  logic [7:0] hold;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rank0    <= 1'b0;
      rank1    <= 1'b0;
      hold     <= 8'd127;
      crst     <= 1'b1;
      mode_mux <= 1'b0;
      mode_dle <= 1'b0;
      mode_ipl <= 3'b000;
    end else begin
      rank0 <= rsti_n_i;
      rank1 <= rank0;
      if (!rank0) begin
        mode_mux <= !cdis_n_i;
        mode_dle <= !mdis_n_i;
        mode_ipl <= ipl_n_i;
      end
      // 127 here and the clearing edge below make the 128 clocks: crst is
      // negated by the 128th rising edge on which rank1 reads RSTI negated.
      if (!rank1) begin
        hold <= 8'd127;
        crst <= 1'b1;
      end else if (hold != 8'd0) begin
        hold <= hold - 8'd1;
        crst <= 1'b1;
      end else begin
        crst <= 1'b0;
      end
    end
  end

endmodule
