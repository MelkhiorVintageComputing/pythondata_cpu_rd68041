// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The exception sequencer: once the core has taken an exception at write-back,
// changed the SR and worked out where the frame goes (rd68041_core.sv), the
// micro-operations that build the frame and fetch the vector (UM 8.1, Figure
// 8-1), issued in place of decode's, one a clock, into <ea> calculate.
//
//   an interrupt's acknowledge cycle first (UM 8.2.9, 7.5.1): the vector
//     offset into T0 (OP_IACK);
//   the frame, a long word a micro-operation at its absolute address (UM 8.4:
//     "it uses long-word operand transfers wherever possible"; the order is
//     not the part's, which "does not necessarily read or write the stack
//     frame data in sequential order"), $0, $2, $3, $4 or $7
//     (doc/checkpoint.md);
//   for an interrupt taken with M set, the throwaway frame ($1) on the
//     interrupt stack (UM 8.2.9);
//   the vector, read from VBR + 4 x the vector number and jumped to (BR_MEM).
//
// Every one is marked `xs`: a fault on any of them is a double bus fault. The
// last is the one marked `last`; the core counts the sequence done when it is
// past write-back.

module rd68041_exc (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          crst,

    input  logic          start,       // one clock: the parameters below are taken
    input  logic   [7:0]  vec,         // the vector (an interrupt's comes from its acknowledge)
    input  logic          intr,        // an interrupt: acknowledge it first
    input  logic   [2:0]  level,       // ... at this level
    input  logic   [3:0]  fmt,         // the frame's format: $0, $2, $3, $4 or $7
    input  logic  [15:0]  sr_old,      // the SR the frame holds
    input  logic  [15:0]  sr_new,      // the SR the throwaway frame holds
    input  logic  [31:0]  pc,          // the frame's PC
    input  logic  [31:0]  addr,        // $2: ADDR; $3, $4: EA; $7: FA
    input  logic  [31:0]  pcf,         // $4: the PC of the faulted instruction
    input  logic  [15:0]  ssw,         // $7: the special status word
    input  logic [383:0]  tail,        // $7 of a write's fault: long words 3-14 (rd68041_dmu),
    input  logic          use_tail,    // ... held there until the frame is written
    input  logic  [31:0]  base,        // the frame's address
    input  logic          two,         // a throwaway frame too ...
    input  logic  [31:0]  base2,       // ... here
    input  logic  [31:0]  vbr,

    output logic          valid,       // a micro-operation is offered ...
    output logic [rd68041_pkg::UOP_W-1:0] uop,
    input  logic          take         // ... and <ea> calculate took it
);

  // The slots, in issue order; a sequence uses those its parameters enable.
  localparam logic [4:0] S_IACK = 5'd0;     // the acknowledge
  localparam logic [4:0] S_F0   = 5'd1;     // the frame's long words 0-14: 1-15
  localparam logic [4:0] S_T0   = 5'd16;    // the throwaway frame's two
  localparam logic [4:0] S_T1   = 5'd17;
  localparam logic [4:0] S_VEC  = 5'd18;    // the vector

  logic        busy;
  logic  [4:0] slot;
  logic  [7:0] x_vec;
  logic        x_intr, x_two;
  logic  [2:0] x_level;
  logic  [3:0] x_fmt;
  logic [15:0] x_sr_old, x_sr_new, x_ssw;
  logic        x_tail;
  logic [31:0] x_pc, x_addr, x_pcf, x_base, x_base2, x_vbr;

  // The frame's length in long words.
  function automatic logic [4:0] longs(input logic [3:0] f);
    longs = (f == 4'h7) ? 5'd15 : (f == 4'h4) ? 5'd4 : (f == 4'h2 || f == 4'h3) ? 5'd3 : 5'd2;
  endfunction

  function automatic logic used(input logic [4:0] s, input logic i, input logic t,
                                input logic [3:0] f);
    if (s == S_IACK)                  used = i;
    else if (s < S_T0)                used = (s - S_F0) < longs(f);
    else if (s == S_T0 || s == S_T1)  used = t;
    else                              used = 1'b1;
  endfunction

  // The slot after s that this sequence uses (S_VEC always is).
  function automatic logic [4:0] after(input logic [4:0] s, input logic i, input logic t,
                                       input logic [3:0] f);
    after = S_VEC;
    for (int k = int'(S_VEC) - 1; k >= 0; k--)
      if (5'(k) > s && used(5'(k), i, t, f)) after = 5'(k);
  endfunction

  // The frame's long word n (doc/checkpoint.md): SR and the PC's high half;
  // the PC's low half and the format/vector-offset word; then $2's ADDR, $3's
  // EA, $4's EA and the faulted instruction's PC (UM A.5.2), or $7's EA, SSW
  // and WB3S, WB2S and WB1S, FA, and the write-backs and push data -- empty
  // but for a write's fault, whose long words 3-14 the data memory unit holds
  // (tail).
  function automatic logic [31:0] frame_lw(input logic [3:0] n, input logic [3:0] f,
                                           input logic [15:0] srv, input logic [31:0] p,
                                           input logic [7:0] v, input logic [31:0] a,
                                           input logic [15:0] w, input logic [31:0] pf);
    case (n)
      4'd0:    frame_lw = {srv, p[31:16]};
      4'd1:    frame_lw = {p[15:0], f, 2'b00, v, 2'b00};
      4'd2:    frame_lw = (f == 4'h2 || f == 4'h3 || f == 4'h4) ? a : 32'h0;
      4'd3:    frame_lw = (f == 4'h4) ? pf : {w, 16'h0};
      4'd5:    frame_lw = a;
      default: frame_lw = 32'h0;
    endcase
  endfunction

  // Worked out here, not in the clocked process: yosys keeps a function's loop
  // variable there as a register (doc/coding-standard.md).
  logic [4:0] next_slot;
  assign next_slot = after(slot, x_intr, x_two, x_fmt);

  rd68041_pkg::uop_t u;
  always_comb begin
    logic [3:0] n;
    n = 4'(slot - S_F0);
    u = '0;
    u.valid   = busy;
    u.xs      = 1'b1;
    u.pc      = x_pc;
    u.ea_en   = 1'b1;
    u.ea_base = rd68041_pkg::B_NONE;
    u.sz      = rd68041_pkg::SZ_L;
    u.a_sel   = rd68041_pkg::S_IMM;
    u.b_sel   = rd68041_pkg::S_IMM;
    u.op      = rd68041_pkg::OP_PASSB;
    u.cc      = rd68041_pkg::CC_NONE;
    u.wr_sz   = rd68041_pkg::SZ_L;
    case (slot)
      S_IACK: begin
        // UM 7.5.1.1: a read at $FFFFFFFF with TT 11 and TM the level.
        u.ea_disp = 32'hFFFF_FFFF;
        u.ld = 1'b1; u.sz = rd68041_pkg::SZ_B; u.ack = 1'b1;
        u.fn = {1'b0, x_level};
        u.b_sel = rd68041_pkg::S_MEM;
        u.op = rd68041_pkg::OP_IACK;
        u.wr = 1'b1; u.wr_reg = 5'd16;
      end
      S_T0, S_T1: begin
        u.ea_disp = x_base2 + ((slot == S_T1) ? 32'd4 : 32'd0);
        u.st = 1'b1;
        u.imm = frame_lw((slot == S_T1) ? 4'd1 : 4'd0, 4'h1, x_sr_new, x_pc, x_vec, 32'h0, 16'h0, 32'h0);
      end
      S_VEC: begin
        // UM 8.1: the vector at VBR + 4 x its number, read as data in the
        // supervisor space; the handler's address, jumped to.
        u.last = 1'b1;
        u.ld = 1'b1;
        u.b_sel = rd68041_pkg::S_MEM;
        u.br = rd68041_pkg::BR_MEM;
        if (x_intr) begin
          u.ea_base = rd68041_pkg::B_REG; u.ea_reg = 5'd16;
          u.ea_disp = x_vbr;
        end else begin
          u.ea_disp = x_vbr + {22'h0, x_vec, 2'b00};
        end
      end
      default: begin
        u.ea_disp = x_base + {25'h0, n, 2'b00};
        u.st = 1'b1;
        u.imm = frame_lw(n, x_fmt, x_sr_old, x_pc, x_vec, x_addr, x_ssw, x_pcf);
        if (x_tail && n >= 4'd3) u.imm = tail[32*(int'(n) - 3) +: 32];
      end
    endcase
    // An interrupt's vector offset is T0's: the format word is ORed with it.
    if (x_intr && (slot == S_T1 || slot == S_F0 + 5'd1)) begin
      u.op = rd68041_pkg::OP_OR;
      u.b_sel = rd68041_pkg::S_REG; u.b_reg = 5'd16;
    end
  end
  assign uop   = u;
  assign valid = busy;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy     <= 1'b0;
      slot     <= '0;
      x_vec    <= '0;
      x_intr   <= 1'b0;
      x_two    <= 1'b0;
      x_level  <= '0;
      x_fmt    <= '0;
      x_sr_old <= '0;
      x_sr_new <= '0;
      x_ssw    <= '0;
      x_tail   <= 1'b0;
      x_pc     <= '0;
      x_addr   <= '0;
      x_pcf    <= '0;
      x_base   <= '0;
      x_base2  <= '0;
      x_vbr    <= '0;
    end else if (crst) begin
      busy     <= 1'b0;
    end else if (start) begin
      busy     <= 1'b1;
      slot     <= intr ? S_IACK : S_F0;
      x_vec    <= intr ? 8'd0 : vec;
      x_intr   <= intr;
      x_two    <= two;
      x_level  <= level;
      x_fmt    <= fmt;
      x_sr_old <= sr_old;
      x_sr_new <= sr_new;
      x_ssw    <= ssw;
      x_tail   <= use_tail;
      x_pc     <= pc;
      x_addr   <= addr;
      x_pcf    <= pcf;
      x_base   <= base;
      x_base2  <= base2;
      x_vbr    <= vbr;
    end else if (busy && take) begin
      if (slot == S_VEC) busy <= 1'b0;
      else               slot <= next_slot;
    end
  end

endmodule
