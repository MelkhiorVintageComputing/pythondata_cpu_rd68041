// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- two-rank input synchroniser. Reused from RD68031, moved to the
// rising edge.
//
// UM 5.7.2: RSTI "is an asynchronous input that is internally synchronized to the
// next rising edge of the BCLK signal". The MC68040's bus is referenced to BCLK's
// rising edge throughout (UM 7, UM 11), so both ranks are positive-edge clocked;
// RD68031's were on the falling edge, as the MC68030's sample window is (UM30 7.1).
//
// This is for the inputs that may change at any time relative to a bus cycle:
// RSTI, IPL2-IPL0 and the reset-sampled configuration. The bus control inputs --
// TA, TEA, TBI, TCI, BG, BB, SC1-SC0 -- are synchronous to BCLK (UM 7) and are
// sampled directly by the bus unit, with no rank of latency added.

module rd68041_sync #(
    parameter int WIDTH = 1,
    // Explicit, because there is no power-on state: an active-low input resets to
    // its negated value so that nothing looks asserted while rst_n is held. Sized
    // by WIDTH rather than fixed at 32 bits, or every instantiation is a
    // width-expansion warning.
    parameter logic [WIDTH-1:0] RESET_VAL = '1
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);

  logic [WIDTH-1:0] rank0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rank0 <= RESET_VAL;
      q     <= RESET_VAL;
    end else begin
      rank0 <= d;
      q     <= rank0;
    end
  end

endmodule
