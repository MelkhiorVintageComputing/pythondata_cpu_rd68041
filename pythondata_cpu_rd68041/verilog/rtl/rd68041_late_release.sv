// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// An output enable that turns on at a rising edge of BCLK and turns off half a
// clock after one: for BB, TA and TIP, the three pins the MC68040 negates and
// then releases.
//
// UM 7.1: "the TIP, TA, and BB signals ... transition between logic levels during
// T4 but transition from a driven state to a high-impedance state during T1", and
// Figure 7-32: "the processor negates BB and TIP before three-stating these". The
// numbers are specification 39, "BCLK to BB, TA, TIP High Impedance": 19 to 28 ns
// at 25 MHz, 14 to 23 at 33, 11.5 to 22 at 40 -- about half a period after the
// rising edge, while specification 38 releases every other bus signal 9 to 18 ns
// after it. A rising-edge enable cannot meet 39: released at the edge it is early,
// released at the next it is a whole period late and fights the next master,
// which may drive BB from that edge (Figure 7-32, C9).
//
// So the pin's enable is `on_q | hold_q`: on_q is the rising-edge register the
// bus unit decides, and hold_q a falling-edge copy of it. When on_q drops at a
// rising edge, hold_q still holds the 1 it took at the falling edge before, and
// keeps the pin driven -- at the level the bus unit set at that same rising
// edge, negated -- until the falling edge that follows. When on_q rises, the OR
// rises with it. Only one input of the OR changes at any edge, so it does not
// glitch.
//
// This is the one falling-edge register in rtl/ (doc/coding-standard.md, "One
// edge only"). It feeds nothing but a pin's output enable; no path inside the
// core starts or ends on the falling edge.

module rd68041_late_release (
    input  logic clk,
    input  logic rst_n,
    input  logic on_d,    // drive the pin for the whole of the next clock
    output logic oe       // to the pin
);

  logic on_q;
  logic hold_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      on_q <= 1'b0;
    end else begin
      on_q <= on_d;
    end
  end

  always_ff @(negedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hold_q <= 1'b0;
    end else begin
      hold_q <= on_q;
    end
  end

  assign oe = on_q | hold_q;

endmodule
