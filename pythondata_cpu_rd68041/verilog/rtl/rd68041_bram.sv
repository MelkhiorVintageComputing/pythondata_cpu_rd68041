// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// A true dual-port RAM of 64-bit words with byte write enables and a registered read on each
// port: the block RAM every FPGA has, written so that Vivado, Quartus and yosys
// infer one (doc/coding-standard.md).
//
// The memory itself is not reset and is not a register: whatever it holds at
// power-up is never observed, because each structure built on it gates every
// read with valid bits that are reset (doc/microarch.md, "Storage").
//
// The read data registers ARE registers, so they are reset -- synchronously,
// from rst_n, because that is the reset a block RAM's output register has
// (RSTRAM on a 7-series RAMB36, sclr on an M10K): an asynchronous one would
// push the register out of the RAM and the read with it. It needs the clock to
// run while rst_n is held, which it does on every target this design has.
//
// Each port reads the word at the address it is given on a rising edge, as it
// was before any write on that edge ("read first"); a write on one port to the
// address the other reads on the same edge leaves the other's read undefined,
// and no user of this module does it.

module rd68041_bram #(
    parameter int DEPTH = 128
) (
    input  logic                      clk,
    input  logic                      rst_n,

    input  logic [$clog2(DEPTH)-1:0]  a_addr,
    input  logic [7:0]                a_we,     // per byte, 7 the most significant
    input  logic [63:0]               a_wdata,
    output logic [63:0]               a_rdata,

    input  logic [$clog2(DEPTH)-1:0]  b_addr,
    input  logic [7:0]                b_we,
    input  logic [63:0]               b_wdata,
    output logic [63:0]               b_rdata
);

  logic [63:0] mem [0:DEPTH-1];

  // One process per port, each with its read and its writes: the shape Vivado
  // and Quartus infer a true dual-port block RAM from (measured at M4: a write
  // per byte lane in a process of its own made Vivado build the RAM from
  // registers, "RAM has multiple writes via different ports in same
  // process"). The byte lanes are written out rather than looped over: yosys
  // makes thirty-two flip-flops of a process loop's own index outside an
  // asynchronous reset branch (doc/coding-standard.md), so the word is fixed at
  // eight bytes.
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      a_rdata <= '0;
    end else begin
      a_rdata <= mem[a_addr];
    end
      if (a_we[0]) mem[a_addr][7:0] <= a_wdata[7:0];
      if (a_we[1]) mem[a_addr][15:8] <= a_wdata[15:8];
      if (a_we[2]) mem[a_addr][23:16] <= a_wdata[23:16];
      if (a_we[3]) mem[a_addr][31:24] <= a_wdata[31:24];
      if (a_we[4]) mem[a_addr][39:32] <= a_wdata[39:32];
      if (a_we[5]) mem[a_addr][47:40] <= a_wdata[47:40];
      if (a_we[6]) mem[a_addr][55:48] <= a_wdata[55:48];
      if (a_we[7]) mem[a_addr][63:56] <= a_wdata[63:56];
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      b_rdata <= '0;
    end else begin
      b_rdata <= mem[b_addr];
    end
      if (b_we[0]) mem[b_addr][7:0] <= b_wdata[7:0];
      if (b_we[1]) mem[b_addr][15:8] <= b_wdata[15:8];
      if (b_we[2]) mem[b_addr][23:16] <= b_wdata[23:16];
      if (b_we[3]) mem[b_addr][31:24] <= b_wdata[31:24];
      if (b_we[4]) mem[b_addr][39:32] <= b_wdata[39:32];
      if (b_we[5]) mem[b_addr][47:40] <= b_wdata[47:40];
      if (b_we[6]) mem[b_addr][55:48] <= b_wdata[55:48];
      if (b_we[7]) mem[b_addr][63:56] <= b_wdata[63:56];
  end

endmodule
