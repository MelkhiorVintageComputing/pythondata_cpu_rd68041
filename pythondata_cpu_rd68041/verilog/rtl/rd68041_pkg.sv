// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// Architectural constants transcribed from the manual, each with its citation.
// Written down once, here; a module that needs one refers to it by its full scope
// (doc/coding-standard.md: there is no portable `import`).

package rd68041_pkg;

  // UM 5.3.1, Table 5-2: transfer type, TT1-TT0.
  localparam logic [1:0] TT_NORMAL = 2'b00;
  localparam logic [1:0] TT_MOVE16 = 2'b01;
  localparam logic [1:0] TT_ALTFC  = 2'b10;  // alternate logical function code
  localparam logic [1:0] TT_ACK    = 2'b11;  // interrupt and breakpoint acknowledge

  // UM 5.3.2, Table 5-3: transfer modifier, TM2-TM0, for normal and MOVE16
  // accesses. MOVE16 uses only the two data encodings.
  localparam logic [2:0] TM_PUSH      = 3'b000;  // data cache push
  localparam logic [2:0] TM_USER_DATA = 3'b001;
  localparam logic [2:0] TM_USER_CODE = 3'b010;
  localparam logic [2:0] TM_TBL_DATA  = 3'b011;  // MMU table search, data
  localparam logic [2:0] TM_TBL_CODE  = 3'b100;  // MMU table search, code
  localparam logic [2:0] TM_SUP_DATA  = 3'b101;
  localparam logic [2:0] TM_SUP_CODE  = 3'b110;

  // UM 7: transfer size, SIZ1-SIZ0. There is no dynamic bus sizing on the
  // MC68040; a line is four long words, by burst or, with TBI, one at a time.
  localparam logic [1:0] SIZ_LONG = 2'b00;
  localparam logic [1:0] SIZ_BYTE = 2'b01;
  localparam logic [1:0] SIZ_WORD = 2'b10;
  localparam logic [1:0] SIZ_LINE = 2'b11;

  // UM 5.9.1, Table 5-6: processor status, PST3-PST0. Stubbed (doc/pinout.md);
  // the encodings are kept for when it is not.
  localparam logic [3:0] PST_USER_RUN = 4'h0;  // user, start/continue current instruction
  localparam logic [3:0] PST_SUP_RUN  = 4'h8;  // supervisor, start/continue

  // ===========================================================================
  // The micro-operation: what decode issues and every later stage carries
  // (doc/microarch.md, "Micro-operations"). One a clock; an instruction is one
  // or more, the last with `last` set.
  // ===========================================================================

  // Where EA's base comes from.
  localparam logic [1:0] B_NONE = 2'd0;   // zero: absolute, immediate, register direct
  localparam logic [1:0] B_AN   = 2'd1;   // address register ea_an
  localparam logic [1:0] B_PC   = 2'd2;   // the instruction's address (disp folded, PRM 2.2.11)
  localparam logic [1:0] B_REG  = 2'd3;   // any register, ea_reg (a temporary holding an address)

  // An update in EA.
  localparam logic [1:0] U_NONE = 2'd0;
  localparam logic [1:0] U_POST = 2'd1;   // (An)+: the address is An, An becomes An + step
  localparam logic [1:0] U_PRE  = 2'd2;   // -(An): An becomes An - step, which is the address

  // An EX operand.
  localparam logic [1:0] S_REG  = 2'd0;
  localparam logic [1:0] S_IMM  = 2'd1;
  localparam logic [1:0] S_MEM  = 2'd2;   // the operand OF loaded
  localparam logic [1:0] S_EA   = 2'd3;   // the effective address itself

  // Operation sizes, as everywhere below.
  localparam logic [1:0] SZ_B = 2'd0;
  localparam logic [1:0] SZ_W = 2'd1;
  localparam logic [1:0] SZ_L = 2'd2;

  // EX operations: the result from A (the register or immediate side) and B
  // (the memory, register or immediate side).
  localparam logic [5:0] OP_PASSB = 6'd0;   // MOVE, MOVEA, LEA, MOVEQ: B
  localparam logic [5:0] OP_ADD = 6'd1;   // B + A
  localparam logic [5:0] OP_SUB = 6'd2;   // B - A
  localparam logic [5:0] OP_AND = 6'd3;
  localparam logic [5:0] OP_OR = 6'd4;
  localparam logic [5:0] OP_EOR = 6'd5;
  localparam logic [5:0] OP_CMP = 6'd6;   // B - A, flags only
  localparam logic [5:0] OP_EXTW = 6'd7;   // B's byte sign-extended to a word
  localparam logic [5:0] OP_EXTL = 6'd8;   // B's word sign-extended to a long
  localparam logic [5:0] OP_EXTBL = 6'd9;   // B's byte sign-extended to a long
  localparam logic [5:0] OP_SWAP = 6'd10;
  localparam logic [5:0] OP_CLR = 6'd11;  // zero
  localparam logic [5:0] OP_PASSA = 6'd12;  // A (a store of a register)
  localparam logic [5:0] OP_ADDA = 6'd13;  // B + A at 32 bits, no flags (ADDA, SUBQ to An...)
  localparam logic [5:0] OP_SUBA = 6'd14;
  localparam logic [5:0] OP_CCR = 6'd15;  // ANDI/ORI/EORI to CCR: CCR op= the immediate
  localparam logic [5:0] OP_SHIFT = 6'd16;  // fn: {left, kind}; B shifted by A (rd68041_shifter)
  localparam logic [5:0] OP_NEG = 6'd17;  // 0 - B
  localparam logic [5:0] OP_NEGX = 6'd18;  // 0 - B - X
  localparam logic [5:0] OP_NOT = 6'd19;
  localparam logic [5:0] OP_ADDX = 6'd20;  // B + A + X
  localparam logic [5:0] OP_SUBX = 6'd21;  // B - A - X
  localparam logic [5:0] OP_SCC = 6'd22;  // $FF if cond, else $00
  localparam logic [5:0] OP_DBCC = 6'd23;  // B.W - 1, and the branch unless cond or -1
  localparam logic [5:0] OP_BIT = 6'd24;  // fn: 0 BTST 1 BCHG 2 BCLR 3 BSET; bit A of B
  localparam logic [5:0] OP_MUL = 6'd25;  // fn: {long, signed, 64-bit}
  localparam logic [5:0] OP_DIV = 6'd26;  // fn: {long, signed, 64-bit dividend}
  localparam logic [5:0] OP_EXG = 6'd27;  // result A, second result B
  localparam logic [5:0] OP_TOCCR = 6'd28;  // the CCR from A's low five bits
  localparam logic [5:0] OP_BCD = 6'd29;  // fn: 0 ABCD, 1 SBCD, 2 NBCD
  localparam logic [5:0] OP_LINK = 6'd30;  // result the address, second result the address + imm
  localparam logic [5:0] OP_MOVEP = 6'd31;  // fn: {store, -, byte}: B's byte fn[1:0] in or out
  localparam logic [5:0] OP_PACK  = 6'd32;  // fn[0]: memory form, {B, A} bytes; + imm, packed
  localparam logic [5:0] OP_UNPK  = 6'd33;  // B's byte unpacked to a word, + imm
  localparam logic [5:0] OP_CAS   = 6'd34;  // compare A (Dc) with the operand read; Du (C) or it back
  localparam logic [5:0] OP_TAS   = 6'd35;  // the byte read with bit 7 set
  localparam logic [5:0] OP_CMP2  = 6'd36;  // the register (C) against A (lower) and B (upper)
  localparam logic [5:0] OP_BFPRM = 6'd37;  // a bit field's width (B) and offset (A); the byte address
  localparam logic [5:0] OP_BFPK  = 6'd38;  // ... with the fifth byte read
  localparam logic [5:0] OP_BF    = 6'd39;  // fn: {register, type}: the operation (rd68041_bitfield)
  localparam logic [5:0] OP_SR    = 6'd40;  // fn: 0 AND, 1 OR, 2 EOR, 3 MOVE: the SR with A (M8)
  localparam logic [5:0] OP_RTEF  = 6'd41;  // RTE's format word (B): the stack past the frame, or vector 14
  localparam logic [5:0] OP_RTE   = 6'd42;  // RTE's last: the SR from A, the PC from B, C the format word
  localparam logic [5:0] OP_IACK  = 6'd43;  // the vector offset an interrupt acknowledge returned (fn: level)

  // How the condition codes are set (PRM 3.4, Table 3-18).
  localparam logic [2:0] CC_NONE  = 3'd0;
  localparam logic [2:0] CC_LOGIC = 3'd1;   // N, Z from the result; V, C cleared; X kept
  localparam logic [2:0] CC_ADD   = 3'd2;   // X N Z V C of an add
  localparam logic [2:0] CC_SUB   = 3'd3;   // X N Z V C of a subtract
  localparam logic [2:0] CC_CMP   = 3'd4;   // N Z V C of a subtract, X kept
  localparam logic [2:0] CC_SET   = 3'd5;   // the CCR operations

  // Control registers, for MOVEC, MOVE USP and MOVE from SR (UM 2.2, 3.1, 4.2).
  localparam logic [3:0] CR_SFC  = 4'd0;
  localparam logic [3:0] CR_DFC  = 4'd1;
  localparam logic [3:0] CR_CACR = 4'd2;
  localparam logic [3:0] CR_TC   = 4'd3;
  localparam logic [3:0] CR_ITT0 = 4'd4;
  localparam logic [3:0] CR_ITT1 = 4'd5;
  localparam logic [3:0] CR_DTT0 = 4'd6;
  localparam logic [3:0] CR_DTT1 = 4'd7;
  localparam logic [3:0] CR_USP  = 4'd8;
  localparam logic [3:0] CR_VBR  = 4'd9;
  localparam logic [3:0] CR_MSP  = 4'd10;
  localparam logic [3:0] CR_ISP  = 4'd11;
  localparam logic [3:0] CR_MMUSR = 4'd12;
  localparam logic [3:0] CR_URP  = 4'd13;
  localparam logic [3:0] CR_SRP  = 4'd14;
  localparam logic [3:0] CR_SR   = 4'd15;    // read only here: MOVE from SR

  // A control transfer, resolved in EX.
  localparam logic [1:0] BR_NONE = 2'd0;
  localparam logic [1:0] BR_DISP = 2'd1;    // to pc + 2 + imm, if cond holds (Bcc, BRA, BSR)
  localparam logic [1:0] BR_EA   = 2'd2;    // to B: the effective address (JMP), or a temporary (JSR)
  localparam logic [1:0] BR_MEM  = 2'd3;    // to the long word OF loaded (RTS)

  typedef struct packed {
    logic        valid;
    logic        last;       // the instruction's last micro-operation
    logic        halt;       // stop the pipeline here (an instruction not built yet, M6)
    logic [31:0] pc;         // the instruction's address
    logic  [4:0] len;        // the instruction's length in bytes
    // <ea> calculate
    logic        ea_en;
    logic  [1:0] ea_base;
    logic  [2:0] ea_an;
    logic  [4:0] ea_reg;     // B_REG's register
    logic        ea_xen;     // an index register
    logic  [3:0] ea_xn;
    logic        ea_xl;      // the index is long (else its low word, sign-extended)
    logic  [1:0] ea_scale;
    logic [31:0] ea_disp;
    logic  [1:0] ea_upd;
    logic  [3:0] ea_step;    // (An)+, -(An): 1 2 4, or 12 (FMOVE.P out, at once)
    // <ea> fetch and write-back
    logic        ld;
    logic        st;
    logic  [1:0] sz;
    // execute
    logic  [1:0] a_sel;
    logic  [4:0] a_reg;      // 0-7 D, 8-15 A, 16-19 temporaries
    logic  [1:0] b_sel;
    logic  [4:0] b_reg;
    logic        a_sxw;      // A is a word, sign-extended to a long first (ADDA.W, CMPA.W)
    logic        b_sxw;      // B likewise (MOVEA.W)
    logic [31:0] imm;
    logic  [5:0] op;
    logic  [2:0] cc;
    logic  [3:0] cond;       // Bcc's condition (PRM Table 3-19); 0 = true
    logic  [1:0] br;
    logic [31:0] tgt;        // BR_DISP's target, computed in decode
    logic  [3:0] fn;         // the operation's variant (OP_SHIFT, OP_BIT, OP_MUL...)
    logic        trap;       // a conditional trap (TRAPcc, TRAPV, CHK): taken if EX says
    logic        crr;        // B is control register cr
    logic        crw;        // the result goes to control register cr
    logic  [3:0] cr;
    logic        lk;         // a locked transfer (CAS, CAS2, TAS; UM 7.4.5)
    logic        lke;        // ... the sequence's last
    logic        alt;        // MOVES: the space SFC or DFC names (UM 7.4, TT = 10)
    logic        cx;         // only if cond holds: else no store, no write, no CCR (CAS2)
    logic  [1:0] bfa;        // a bit field's first (1) or second (2) access: sized at EA
                             // from T3 to the bytes the field covers, the second dropped
                             // if there is none (rd68041_core.sv, bf_bytes)
    // exceptions and the status register (M8; doc/microarch.md, "Exceptions")
    logic  [7:0] xvec;       // the exception this micro-operation takes at W, 0 for none;
                             // with it, imm is the fault address and tgt[15:0] the SSW
    logic        xnext;      // ... after it completes: the frame's PC is the next instruction's
    logic        xs;         // part of exception processing (rd68041_exc): a fault is a
                             // double bus fault
    logic        ser;        // serialising: nothing younger issues until it is past W
    logic        srw;        // the result is the new SR (with ser)
    logic        stp;        // STOP: then wait for an interrupt
    logic        rsto;       // RESET: assert RSTO
    logic        ack;        // the read is an acknowledge cycle (TT 11), TM fn[2:0]; a
                             // transfer error is an answer, not a fault
    logic        cof;        // a change of flow for tracing (UM 8.2.6), taken or not
    logic        cmo;        // CINV, CPUSH (with ser): fn {caches, scope}, cond[0] push,
                             // the result the line's or page's address (M9)
    logic        m16;        // MOVE16: the read or write of a line, through the data
                             // memory unit's own buffer (M9)
    // the FPU (M11; rd68041_fpu, doc/fpu.md)
    logic  [3:0] fpk;        // the FP micro-operation (rd68041_fpu_pkg::FK_*), 0 none
    logic        fchk;       // the FP instruction's first: a pending exception is taken
    logic  [4:0] fidx;       // the long word it moves
    logic  [2:0] freg;       // FMOVEM's FP register
    logic [15:0] fcmd;       // the command word, or the predicate (FBcc...) in 5-0
    // the register result
    logic        wr;
    logic  [4:0] wr_reg;
    logic  [1:0] wr_sz;      // the size written: an address register is always long
    logic        wr2;        // a second register result, long (EXG, LINK, UNLK, MUL.L, DIV.L)
    logic  [4:0] wr2_reg;
  } uop_t;

  // An <ea>'s fields of a micro-operation, as rd68041_decode works them out
  // once and copies them (all but ea_step, which goes with the operand's size).
  typedef struct packed {
    logic        en;
    logic  [1:0] base;
    logic  [2:0] an;
    logic  [4:0] rg;
    logic        xen;
    logic  [3:0] xn;
    logic        xl;
    logic  [1:0] scale;
    logic [31:0] disp;
    logic  [1:0] upd;
    logic        bad;        // a reserved encoding (PRM Table 2-2): halt
  } ea_t;

  // Its width, written out: yosys has no $bits of a type. 40 (valid, last, halt,
  // pc, len) + 57 (the <ea>) + 4 (ld, st, sz) + 112 (execute) + 16 (exceptions)
  // + 2 (cache maintenance, MOVE16) + 29 (the FPU) + 14 (the results).
  // sim/tb/core_*_tb check it against $bits.
  localparam int UOP_W = 274;

  // The status register's implemented bits (UM 2.2.1): T1 T0 S M, I2-I0, the CCR.
  localparam logic [15:0] SR_MASK = 16'hF71F;

endpackage
