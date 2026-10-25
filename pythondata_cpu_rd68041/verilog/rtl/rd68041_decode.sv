// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The decode stage's decoder: an operation word and the extension words after
// it, in, one micro-operation out (rd68041_pkg::uop_t, doc/microarch.md).
// Combinational.
//
// The instruction is decoded once it is all in the queue: `need` is how many
// extension words it has, and the decode stage waits until the queue holds
// them. Most instructions are one micro-operation. Where one has more -- a
// second memory address, a push beside its own address, a list of registers --
// they are issued one a clock: `more` says another follows, `seq` how many
// went before, and `pop` how many extension words this one takes (the
// operation word is the decode stage's to take, on the first). The decode stage
// holds the operation word, the instruction's address and the prelude flags
// (`ind_out`, returned as `ind_in`) for the rest.
//
// An <ea> with memory indirection (PRM 2.2.8-2.2.10) is two addresses: a
// prelude reads the pointer into T3, and the instruction then addresses from
// T3. The prelude comes first, before the instruction's own micro-operations,
// or -- for MOVE's destination -- before the micro-operation that needs it.
// It is a plain read of the program's data space whatever the instruction
// is: MOVES's space (SFC, DFC) and its sign extension to An, a lock, a bit
// field's sizing, an SR write or an exception are the operand's, not the
// pointer's (DEC_IND; doc/bugs-found.md M12, M13).
//
//   MOVE <mem>,<mem>   load the source into T0; store T0 at the destination
//   PEA <ea>           T0 = <ea>; push T0
//   JSR <ea>           T0 = <ea>; push the return address and jump to T0
//   ADDX, SUBX -(Ay),-(Ax)   load -(Ay) into T0; the operation on -(Ax)
//   CMPM (Ay)+,(Ax)+   load (Ay)+ into T0; compare (Ax)+ with it
//   RTR                the CCR from (A7)+; then RTS
//   ABCD, SBCD -(Ay),-(Ax)   as ADDX
//   PACK, UNPK -(Ax),-(Ay)   two reads and a write, or a read and two writes
//   CMP2, CHK2         the lower bound into T0; the upper, compared
//   CAS2               both operands read under LOCK into T0 and T1, the two
//                      compares, the two updates or the write-back, each
//                      conditional on the compares (`cx`), and Dc1, Dc2
//   MOVEP              one byte a micro-operation, two or four
//   MOVEM              one register a micro-operation, after T0 = <ea> for the
//                      control modes, or T0 = An - size for -(An) when An is
//                      in the list (PRM 4: the MC68020 on writes "the initial
//                      register value decremented by the size")
//
// MOVEM's registers and MOVEP's bytes are counted by the decode stage: it
// passes the next register (mv_reg), whether it is the last (mv_last), and how
// many have gone (mv_k), and is told when one is used (mv_used).
//
// An instruction that takes an exception before it executes -- an illegal
// one, the A-line, the F-line, one privileged in user mode (UM 8.2.4, 8.2.5) --
// is one micro-operation that carries the vector (`xvec`) and does nothing
// else; TRAP #n carries its vector with `xnext`, the frame's PC being the next
// instruction's. The core takes them at write-back (rd68041_core.sv). What is
// not built yet -- the F-line's floating-point instructions (M11) -- is a
// micro-operation with `halt`, which stops the pipeline when it reaches write-back.
//
// The status register's writers (MOVE to SR, ANDI, ORI and EORI to SR, RTE,
// STOP) and NOP and RESET are serialising (`ser`): the core lets nothing after
// them issue until they are past write-back, and fetches again from the next
// instruction (doc/microarch.md, "Exceptions").

module rd68041_decode #(
    parameter int FPU = 1                // 0: the MC68LC040
) (
    input  logic [15:0]  op,           // the operation word
    input  logic [175:0] w,            // extension words: w[0] in 175:160, then w[1]...
    input  logic  [4:0]  seq,          // micro-operations of this instruction issued already
    input  logic  [1:0]  ind_in,       // the preludes issued (bit 0 the source's or op[5:0]'s)
    input  logic  [4:0]  len_in,       // the instruction's length, as the first one said
    input  logic [31:0]  pc,           // the instruction's address
    input  logic  [3:0]  mv_reg,       // MOVEM: the next register (0-7 D, 8-15 A)
    input  logic         mv_last,      // ... and it is the list's last
    input  logic  [4:0]  mv_k,         // MOVEM, MOVEP: transfers already issued
    input  logic         sup,          // SR's S: supervisor (the privileged instructions)
    input  logic         msp_on,       // SR's M: A7 is the master stack pointer, not the ISP
    input  logic         mv_none,      // FMOVEM: the list is empty
    input  logic  [4:0]  fp_svn,       // FSAVE: the frame's long words (the FPU's state)
    input  logic  [4:0]  fp_rsn,       // FRESTORE: those of the frame its first long named
    output logic         qwait,        // this one waits for the pipeline behind decode to empty
    output logic [rd68041_pkg::UOP_W-1:0] uop,
    output logic  [3:0]  need,         // extension words the instruction has (seq 0)
    output logic  [3:0]  pop,          // extension words this micro-operation takes
    output logic         more,         // another micro-operation follows
    output logic         mv_used,      // this one transfers mv_reg (or MOVEP's byte)
    output logic  [1:0]  ind_out       // seq 0: the preludes this instruction has
);

  // ---------------------------------------------------------------------------
  // Extension words, numbered from w[0].
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] ew(input logic [175:0] v, input int k);
    ew = v[175 - 16*k -: 16];
  endfunction
  // Its low byte, and the word sign-extended (a select on a function's result
  // is not portable: yosys and iverilog refuse it).
  function automatic logic [7:0] ewb(input logic [175:0] v, input int k);
    ewb = v[167 - 16*k -: 8];
  endfunction
  function automatic logic [31:0] ews(input logic [175:0] v, input int k);
    ews = {{16{v[175 - 16*k]}}, v[175 - 16*k -: 16]};
  endfunction
  function automatic logic [31:0] ewl(input logic [175:0] v, input int k);
    ewl = {v[175 - 16*k -: 16], v[159 - 16*k -: 16]};
  endfunction

  // An indexed mode with the full extension-word format (PRM 2.2.6-2.2.10):
  // mode 6, or mode 7 register 3, whose first extension word has bit 8 set.
  function automatic logic is_full(input logic [2:0] m, input logic [2:0] r,
                                   input logic [15:0] x);
    is_full = ((m == 3'd6) || (m == 3'd7 && r == 3'd3)) && x[8];
  endfunction
  // Its words: the base displacement's (BD SIZE, bits 5-4: 01 none, 10 a word,
  // 11 a long) and the outer displacement's (I/IS, bits 1-0, the same).
  function automatic logic [3:0] bd_words(input logic [15:0] x);
    bd_words = (x[5:4] == 2'b10) ? 4'd1 : (x[5:4] == 2'b11) ? 4'd2 : 4'd0;
  endfunction
  function automatic logic [3:0] od_words(input logic [15:0] x);
    od_words = (x[2:0] == 3'd0 || x[2:0] == 3'd4) ? 4'd0 :
               (x[1:0] == 2'b10) ? 4'd1 : (x[1:0] == 2'b11) ? 4'd2 : 4'd0;
  endfunction
  // Memory indirect: I/IS not zero (PRM Table 2-2).
  function automatic logic is_ind(input logic [2:0] m, input logic [2:0] r,
                                  input logic [15:0] x);
    is_ind = is_full(m, r, x) && (x[2:0] != 3'd0);
  endfunction
  // ... and post-indexed (I/IS 101-111 with the index not suppressed).
  function automatic logic is_post(input logic [15:0] x);
    is_post = !x[6] && x[2];
  endfunction
  // The encodings PRM Table 2-2 reserves.
  function automatic logic full_bad(input logic [15:0] x);
    full_bad = x[3] || (x[5:4] == 2'b00) || (x[2:0] == 3'd4) || (x[6] && x[2]);
  endfunction

  // How many extension words an <ea> has (PRM 2.2), for an operand of `sz`,
  // its first extension word x.
  function automatic logic [3:0] ea_len(input logic [2:0] m, input logic [2:0] r,
                                        input logic [1:0] sz, input logic [15:0] x);
    if (is_full(m, r, x)) begin
      ea_len = 4'd1 + bd_words(x) + od_words(x);
    end else begin
      case (m)
        3'd5, 3'd6: ea_len = 4'd1;
        3'd7: case (r)
                3'd0, 3'd2, 3'd3: ea_len = 4'd1;
                3'd1:             ea_len = 4'd2;
                3'd4:             ea_len = (sz == rd68041_pkg::SZ_L) ? 4'd2 : 4'd1;
                default:          ea_len = 4'd0;
              endcase
        default: ea_len = 4'd0;
      endcase
    end
  endfunction

  // The immediate an <ea> of mode 7.4 carries, at extension word k.
  function automatic logic [31:0] imm_of(input logic [175:0] v, input int k,
                                         input logic [1:0] sz);
    case (sz)
      rd68041_pkg::SZ_B: imm_of = {24'h0, ewb(v, k)};
      rd68041_pkg::SZ_W: imm_of = {16'h0, ew(v, k)};
      default:           imm_of = ewl(v, k);
    endcase
  endfunction

  // PRM 2.2: the classes of <ea> an instruction allows.
  function automatic logic is_imm(input logic [2:0] m, input logic [2:0] r);
    is_imm = (m == 3'd7) && (r == 3'd4);
  endfunction
  function automatic logic ok_mode(input logic [2:0] m, input logic [2:0] r);
    ok_mode = (m != 3'd7) || (r <= 3'd4);
  endfunction
  // Data alterable: not An, not PC-relative, not immediate.
  function automatic logic dalt(input logic [2:0] m, input logic [2:0] r);
    dalt = (m != 3'd1) && ((m != 3'd7) || (r <= 3'd1));
  endfunction
  // Memory alterable.
  function automatic logic malt(input logic [2:0] m, input logic [2:0] r);
    malt = (m >= 3'd2) && ((m != 3'd7) || (r <= 3'd1));
  endfunction
  // Control: (An), (d16,An), (d8,An,Xn), abs, PC-relative.
  function automatic logic ctrl(input logic [2:0] m, input logic [2:0] r);
    ctrl = (m == 3'd2) || (m == 3'd5) || (m == 3'd6) || ((m == 3'd7) && (r <= 3'd3));
  endfunction

  // One <ea>'s fields, its extension words at w[k]. With the full format and
  // memory indirection, `pre` asks for the first of the two addresses -- where
  // the pointer is read, into T3 -- and otherwise this is the second: T3 plus
  // the post-index and the outer displacement (PRM 2.2.8-2.2.10).
  //
  // Called only in the four always_comb blocks below, never inside the decode
  // case: yosys inlines a function at every call, and proc_mux's time grows
  // with the temporaries times the branches (seventy calls there took it
  // twenty minutes).
  function automatic rd68041_pkg::ea_t ea_of(input logic [175:0] v,
                                             input logic [2:0] m, input logic [2:0] r,
                                             input int k, input bit pre);
    rd68041_pkg::ea_t o;
    logic [15:0] x;
    logic [31:0] bd, od;
    int          kb, ko;
    o = '0;
    x = ew(v, k);
    o.an = r;
    if (is_full(m, r, x)) begin
      kb = k + 1;
      ko = k + 1 + int'(bd_words(x));
      bd = (x[5:4] == 2'b10) ? ews(v, kb) : (x[5:4] == 2'b11) ? ewl(v, kb) : 32'h0;
      od = (od_words(x) == 4'd1) ? ews(v, ko) : (od_words(x) == 4'd2) ? ewl(v, ko) : 32'h0;
      o.en    = 1'b1;
      o.xn    = x[15:12];
      o.xl    = x[11];
      o.scale = x[10:9];
      o.bad   = full_bad(x);
      if (x[2:0] != 3'd0 && !pre) begin
        // The second address: T3, the post-index, the outer displacement.
        o.base = rd68041_pkg::B_REG;
        o.rg   = 5'd19;
        o.xen  = is_post(x);
        o.disp = od;
      end else begin
        // The base (An or the PC, unless BS suppresses it), the index (unless
        // IS does, or it comes after the indirection) and the base
        // displacement. PRM 2.2.11: the PC is the first extension word's.
        o.xen = !x[6] && !(x[2:0] != 3'd0 && is_post(x));
        if (x[7]) begin
          o.base = rd68041_pkg::B_NONE;
          o.disp = bd;
        end else if (m == 3'd7) begin
          o.base = rd68041_pkg::B_PC;
          o.disp = bd + 32'(2 + 2*k);
        end else begin
          o.base = rd68041_pkg::B_AN;
          o.disp = bd;
        end
      end
    end else begin
      case (m)
        3'd2: begin o.en = 1'b1; o.base = rd68041_pkg::B_AN; end
        3'd3: begin o.en = 1'b1; o.base = rd68041_pkg::B_AN; o.upd = rd68041_pkg::U_POST; end
        3'd4: begin o.en = 1'b1; o.base = rd68041_pkg::B_AN; o.upd = rd68041_pkg::U_PRE; end
        3'd5: begin o.en = 1'b1; o.base = rd68041_pkg::B_AN; o.disp = {{16{x[15]}}, x}; end
        3'd6: begin
          o.en    = 1'b1;
          o.base  = rd68041_pkg::B_AN;
          o.xen   = 1'b1;
          o.xn    = x[15:12];
          o.xl    = x[11];
          o.scale = x[10:9];
          o.disp  = {{24{x[7]}}, x[7:0]};
        end
        3'd7: begin
          o.en = (r != 3'd4);
          case (r)
            3'd0: o.disp = {{16{x[15]}}, x};
            3'd1: o.disp = ewl(v, k);
            3'd2: begin o.base = rd68041_pkg::B_PC; o.disp = {{16{x[15]}}, x} + 32'(2 + 2*k); end
            3'd3: begin
              o.base  = rd68041_pkg::B_PC;
              o.xen   = 1'b1;
              o.xn    = x[15:12];
              o.xl    = x[11];
              o.scale = x[10:9];
              o.disp  = {{24{x[7]}}, x[7:0]} + 32'(2 + 2*k);
            end
            default: ;
          endcase
        end
        default: ;
      endcase
    end
    ea_of = o;
  endfunction

  // The modes an instruction names itself, none with more than the one word
  // x0: (An), (An)+, -(An), (d16,An). Cheap enough to call in the case.
  function automatic rd68041_pkg::ea_t ea_fix(input logic [2:0] m, input logic [2:0] r,
                                              input logic [15:0] x0);
    rd68041_pkg::ea_t o;
    o      = '0;
    o.en   = 1'b1;
    o.base = rd68041_pkg::B_AN;
    o.an   = r;
    o.upd  = (m == 3'd3) ? rd68041_pkg::U_POST :
             (m == 3'd4) ? rd68041_pkg::U_PRE  : rd68041_pkg::U_NONE;
    o.disp = (m == 3'd5) ? {{16{x0[15]}}, x0} : 32'h0;
    ea_fix = o;
  endfunction

  // An (An)+ or -(An)'s step for an operand of sz: 2 for a byte on A7.
  function automatic logic [3:0] ea_step_of(input logic [1:0] sz, input logic [2:0] r);
    ea_step_of = (sz == rd68041_pkg::SZ_L) ? 4'd4 :
                 ((sz == rd68041_pkg::SZ_W) || (r == 3'd7)) ? 4'd2 : 4'd1;
  endfunction

  logic bad;
  logic fline;                    // an F-line encoding no instruction has: vector 11

  // Shorthands for the fields the operation words share.
  logic [2:0] sm, sr, dm, dr;     // <ea> mode and register; bits 8-6 and 11-9
  logic [1:0] sz;
  logic [3:0] n_src, n_dst;
  logic [4:0] dn, an_r;           // Dn and An at bits 11-9, as register numbers
  logic [4:0] dy, ay;             // Dn and An at bits 2-0
  logic [4:0] k;                  // the transfer this is (MOVEM, MOVEP)
  logic [4:0] sq;                 // the micro-operation, not counting a prelude
  logic       stp;                // ... not the instruction's first
  logic [3:0] eak;                // where op[5:0]'s <ea> extension words start
  logic       uses_ea;            // op[5:0] is an <ea> of this instruction
  logic       pre_s;              // a prelude was issued for op[5:0]'s <ea>
  logic       priv;               // privileged (UM 8.2.5): in user mode, vector 8

  // ---------------------------------------------------------------------------
  // The <ea>s, worked out once: op[5:0]'s (the source, or the only one) with
  // its words at eak, and MOVE's destination, op[11:6], at kd; each as the
  // operand's address and as the prelude's pointer. The decode case below
  // copies them (DEC_EA, DEC_IND, DEC_SRC).
  // ---------------------------------------------------------------------------
  logic [3:0]        kd;          // where MOVE's destination's words start
  logic [1:0]        msz;         // MOVE's size
  rd68041_pkg::ea_t  ea_s, ea_sp, ea_d, ea_dp;
  logic [31:0]       imm_sb, imm_sw, imm_sl;   // op[5:0] as an immediate, by size
  wire  [15:0]       w0;                  // the first extension word
  assign w0 = w[175:160];

  // Where op[5:0]'s extension words start: after an immediate, a bit number,
  // a second operation word. MOVE's source is at w[0]; its destination after
  // the source's words, or, memory to memory, at w[0] again once the source
  // took them (the phases, below).
  always_comb begin
    logic [2:0] m0;
    m0  = op[5:3];
    eak = 4'd0;
    case (op[15:12])
      4'b0000: begin
        if (op[8] && m0 == 3'd1)       eak = 4'd0;                    // MOVEP
        else if (op[8])                eak = 4'd0;                    // BTST Dn,<ea>...
        else if (op[11:8] == 4'b1000)  eak = 4'd1;                    // BTST #n,<ea>...
        else if (op[7:6] == 2'b11 || op[11:8] == 4'b1110) eak = 4'd1; // CAS, CMP2, MOVES
        else                           eak = (op[7:6] == 2'b10) ? 4'd2 : 4'd1; // the immediates
      end
      4'b0100: if (op[11:7] == 5'b11000 || op[11:7] == 5'b10001 || op[11:7] == 5'b11001)
                 eak = 4'd1;                                          // MUL.L, DIV.L, MOVEM
      4'b1110: if (op[7:6] == 2'b11 && op[11]) eak = 4'd1;            // the bit fields
      // The FP general instructions and FScc: after the command word.
      4'b1111: if (op[11:9] == 3'b001 && op[8:7] == 2'b00) eak = 4'd1;
      default: ;
    endcase
  end
  // MOVE's destination, after the source's words; then op[5:0] and op[11:6]
  // as addressing modes, and op[5:0] as an immediate. Continuous assignments:
  // iverilog wakes an always_comb on the variables of the functions it calls,
  // and blocks calling ea_of with different arguments woke each other forever.
  assign msz    = (op[13:12] == 2'b01) ? rd68041_pkg::SZ_B :
                  (op[13:12] == 2'b11) ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
  assign kd     = ((op[5:3] >= 3'd2) && !is_imm(op[5:3], op[2:0]) && (op[8:6] >= 3'd2))
                  ? 4'd0 : ea_len(op[5:3], op[2:0], msz, w[175:160]);
  assign ea_s   = ea_of(w, op[5:3], op[2:0], int'(eak), 1'b0);
  assign ea_sp  = ea_of(w, op[5:3], op[2:0], int'(eak), 1'b1);
  assign ea_d   = ea_of(w, op[8:6], op[11:9], int'(kd), 1'b0);
  assign ea_dp  = ea_of(w, op[8:6], op[11:9], int'(kd), 1'b1);
  assign imm_sb = imm_of(w, int'(eak), rd68041_pkg::SZ_B);
  assign imm_sw = imm_of(w, int'(eak), rd68041_pkg::SZ_W);
  assign imm_sl = imm_of(w, int'(eak), rd68041_pkg::SZ_L);

  // The data register of a 1-bit-wide field, as a register number.
  function automatic logic [4:0] dreg(input logic [2:0] r);
    dreg = {2'b00, r};
  endfunction
  function automatic logic [4:0] areg(input logic [2:0] r);
    areg = {2'b01, r};
  endfunction

  // MOVEC's control register field (PRM 6, MOVEC; UM 2.2.2, 3.1, 4.2): its
  // index, or 4'hF for one the MC68040 does not have (an illegal instruction).
  function automatic logic [3:0] cr_of(input logic [11:0] c);
    case (c)
      12'h000: cr_of = rd68041_pkg::CR_SFC;
      12'h001: cr_of = rd68041_pkg::CR_DFC;
      12'h002: cr_of = rd68041_pkg::CR_CACR;
      12'h003: cr_of = rd68041_pkg::CR_TC;
      12'h004: cr_of = rd68041_pkg::CR_ITT0;
      12'h005: cr_of = rd68041_pkg::CR_ITT1;
      12'h006: cr_of = rd68041_pkg::CR_DTT0;
      12'h007: cr_of = rd68041_pkg::CR_DTT1;
      12'h800: cr_of = rd68041_pkg::CR_USP;
      12'h801: cr_of = rd68041_pkg::CR_VBR;
      12'h803: cr_of = rd68041_pkg::CR_MSP;
      12'h804: cr_of = rd68041_pkg::CR_ISP;
      12'h805: cr_of = rd68041_pkg::CR_MMUSR;
      12'h806: cr_of = rd68041_pkg::CR_URP;
      12'h807: cr_of = rd68041_pkg::CR_SRP;
      default: cr_of = 4'hF;
    endcase
  endfunction

  rd68041_pkg::ea_t  ea_tmp;      // the DEC_* macros' scratch
  logic [1:0]        src_sel;

  // Into u, in the decode case, through one temporary (so the case inlines
  // nothing large):
  //   DEC_EA(e, sz)       an <ea>'s fields, for an operand of sz
  //   DEC_IND(e)          the prelude of memory indirection: the pointer, a
  //                       long word at e, into T3
  //   DEC_SRC(sz, to_a)   op[5:0]'s operand on side A or B: a register, the
  //                       immediate, or the memory operand (and a load)
`define DEC_EA(E, SZ) begin \
    ea_tmp     = (E); \
    u.ea_en    = ea_tmp.en; \
    u.ea_base  = ea_tmp.base; \
    u.ea_an    = ea_tmp.an; \
    u.ea_reg   = ea_tmp.rg; \
    u.ea_xen   = ea_tmp.xen; \
    u.ea_xn    = ea_tmp.xn; \
    u.ea_xl    = ea_tmp.xl; \
    u.ea_scale = ea_tmp.scale; \
    u.ea_disp  = ea_tmp.disp; \
    u.ea_upd   = ea_tmp.upd; \
    u.ea_step  = ea_step_of((SZ), ea_tmp.an); \
    if (ea_tmp.bad) u.halt = 1'b1; \
  end
`define DEC_IND(E) begin \
    `DEC_EA(E, rd68041_pkg::SZ_L) \
    u.ld     = 1'b1; \
    u.sz     = rd68041_pkg::SZ_L; \
    u.b_sel  = rd68041_pkg::S_MEM; \
    u.op     = rd68041_pkg::OP_PASSB; \
    u.cc     = rd68041_pkg::CC_NONE; \
    u.br     = rd68041_pkg::BR_NONE; \
    u.st     = 1'b0; \
    u.wr     = 1'b1; \
    u.wr_reg = 5'd19; \
    u.wr_sz  = rd68041_pkg::SZ_L; \
    u.wr2    = 1'b0; \
    u.crw    = 1'b0; \
    u.trap   = 1'b0; \
    u.last   = 1'b0; \
    u.xvec   = 8'd0; \
    u.xnext  = 1'b0; \
    u.fpk    = rd68041_fpu_pkg::FK_NONE; \
    u.fchk   = 1'b0; \
    u.lk     = 1'b0; \
    u.lke    = 1'b0; \
    u.alt    = 1'b0; \
    u.cx     = 1'b0; \
    u.bfa    = 2'd0; \
    u.ser    = 1'b0; \
    u.srw    = 1'b0; \
    u.stp    = 1'b0; \
    u.rsto   = 1'b0; \
    u.ack    = 1'b0; \
    u.cof    = 1'b0; \
    u.cmo    = 1'b0; \
    u.m16    = 1'b0; \
    u.crr    = 1'b0; \
    u.a_sxw  = 1'b0; \
    u.b_sxw  = 1'b0; \
  end
`define DEC_SRC(SZ, TO_A) begin \
    if (sm <= 3'd1) begin \
      src_sel = rd68041_pkg::S_REG; \
    end else if (is_imm(sm, sr)) begin \
      src_sel = rd68041_pkg::S_IMM; \
      u.imm = ((SZ) == rd68041_pkg::SZ_B) ? imm_sb : ((SZ) == rd68041_pkg::SZ_W) ? imm_sw : imm_sl; \
    end else begin \
      src_sel = rd68041_pkg::S_MEM; \
      u.ld = 1'b1; \
      `DEC_EA(ea_s, SZ) \
    end \
    if (TO_A) begin u.a_sel = src_sel; u.a_reg = {1'b0, (sm == 3'd1), sr}; end \
    else      begin u.b_sel = src_sel; u.b_reg = {1'b0, (sm == 3'd1), sr}; end \
  end

  rd68041_pkg::uop_t u;

  always_comb begin
    u        = '0;
    ea_tmp   = '0;
    src_sel  = '0;
    u.valid  = 1'b1;
    u.last   = 1'b1;
    u.pc     = pc;
    u.a_sel  = rd68041_pkg::S_REG;
    u.b_sel  = rd68041_pkg::S_REG;
    u.op     = rd68041_pkg::OP_PASSB;
    u.cc     = rd68041_pkg::CC_NONE;
    u.br     = rd68041_pkg::BR_NONE;
    u.wr_sz  = rd68041_pkg::SZ_L;
    u.sz     = rd68041_pkg::SZ_L;   // SWAP, MOVEQ, EXG...: long unless the case says
    bad      = 1'b0;
    fline    = 1'b0;
    qwait    = 1'b0;
    priv     = 1'b0;
    need     =  4'd0;
    pop      =  4'd0;
    more     = 1'b0;
    mv_used  = 1'b0;
    ind_out  = 2'b00;
    sm = op[5:3];  sr = op[2:0];
    dm = op[8:6];  dr = op[11:9];
    dn = dreg(op[11:9]);  an_r = areg(op[11:9]);
    dy = dreg(op[2:0]);   ay = areg(op[2:0]);
    sz = rd68041_pkg::SZ_L;
    n_src =  4'd0;  n_dst =  4'd0;
    // The indirection prelude, if any, comes first; the instruction's own
    // micro-operations are counted after it (doc/microarch.md).
    pre_s = (seq == 5'd0) ? 1'b0 : ind_in[0];
    sq  = seq - {4'h0, pre_s};
    stp = (sq != 5'd0);
    k   = stp ? mv_k : 5'd0;
    // Whether op[5:0] is an <ea> at all, for the prelude below (eak, above,
    // where its words start). MOVE decides its own.
    uses_ea = 1'b1;
    case (op[15:12])
      4'b0000: if (op[8] && sm == 3'd1) uses_ea = 1'b0;               // MOVEP
      4'b0100: if (op[11:4] == 8'hE7)   uses_ea = 1'b0;               // RTE, RTS, MOVEC...
      4'b0101: if (op[7:6] == 2'b11 && (sm == 3'd1 || sm == 3'd7 && sr >= 3'd2))
                 uses_ea = 1'b0;                                      // DBcc, TRAPcc
      4'b0110, 4'b0111, 4'b1010: uses_ea = 1'b0;
      // The FP instructions with an <ea> in op[5:0] (not FBcc, FDBcc, FTRAPcc,
      // nor a register-to-register operation or FMOVECR).
      4'b1111: uses_ea = (op[11:9] == 3'b001) &&
                         ((op[8:6] == 3'b000 && w0[15:13] != 3'b000 &&
                           !(w0[15:13] == 3'b010 && w0[12:10] == 3'b111)) ||
                          (op[8:6] == 3'b001 && !(sm == 3'd1 || (sm == 3'd7 && sr >= 3'd2))) ||
                          op[8:6] == 3'b100 || op[8:6] == 3'b101);
      4'b1110: if (op[7:6] != 2'b11) uses_ea = 1'b0;                  // the register shifts
      default: ;
    endcase

    case (op[15:12])
      // =======================================================================
      // 0000: bit operations, MOVEP, the immediates, CAS, CMP2/CHK2, MOVES
      // =======================================================================
      4'b0000: begin
        if (op[8] && sm == 3'd1) begin
          // MOVEP (PRM 4): bytes at (d16,Ay), +2, +4, +6, one a micro-op.
          logic [2:0] nb;
          nb   = op[6] ? 3'd4 : 3'd2;
          need =  4'd1;
          u.len = 5'd4;
          pop  = (k == 5'(nb - 3'd1)) ? 4'd1 : 4'd0;
          more = (k != 5'(nb - 3'd1));
          u.last = !more;
          mv_used = 1'b1;
          `DEC_EA(ea_fix(3'd5, sr, w0), rd68041_pkg::SZ_B)
          u.ea_disp = u.ea_disp + {26'h0, k, 1'b0};
          u.sz = rd68041_pkg::SZ_B;
          u.op = rd68041_pkg::OP_MOVEP;
          u.b_reg = dn;
          // Byte k of the transfer is the register's byte nb-1-k, high first.
          u.fn = {op[7], 1'b0, 2'(nb - 3'd1 - k[2:0])};
          if (op[7]) begin
            u.st = 1'b1;
          end else begin
            u.ld = 1'b1;
            u.a_sel = rd68041_pkg::S_MEM;
            u.wr = 1'b1; u.wr_reg = dn;
          end
        end else if (op[8] || op[11:8] == 4'b1000) begin
          // BTST, BCHG, BCLR, BSET (PRM 4): the bit number from Dn or from an
          // immediate word; a long word in Dn, a byte in memory.
          logic st_form;
          st_form = !op[8];
          n_src   = st_form ? 4'd1 : 4'd0;
          sz      = (sm == 3'd0) ? rd68041_pkg::SZ_L : rd68041_pkg::SZ_B;
          n_dst   = ea_len(sm, sr, sz, ew(w, int'(n_src)));
          need    = n_src + n_dst;
          pop     = need;
          u.len   = 5'(2 + 2*need);
          u.sz    = sz;
          u.op    = rd68041_pkg::OP_BIT;
          u.fn    = {2'b00, op[7:6]};
          if (st_form) begin
            u.a_sel = rd68041_pkg::S_IMM; u.imm = {24'h0, ewb(w, 0)};
          end else begin
            u.a_reg = dn;
          end
          if (sm == 3'd1 || !ok_mode(sm, sr)) bad = 1'b1;
          // Only BTST reads an immediate or a PC-relative operand.
          if (op[7:6] != 2'b00 && !dalt(sm, sr)) bad = 1'b1;
          if (st_form && is_imm(sm, sr)) bad = 1'b1;
          `DEC_SRC(sz, 1'b0)
          if (op[7:6] != 2'b00) begin
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; end
            else            u.st = 1'b1;
          end
        end else if (op[7:6] == 2'b11 && !op[11]) begin
          // CMP2, CHK2 (PRM 4): Rn against the bounds pair at <ea>, the lower
          // first. The lower into T0; then the upper, compared.
          logic [15:0] x;
          x  = ew(w, 0);
          sz = op[10:9];
          if (sz == 2'b11 || !ctrl(sm, sr) || x[10:0] != 11'h0) bad = 1'b1;
          need  = 4'd1 + ea_len(sm, sr, sz, ew(w, 1));
          u.len = 5'(2 + 2*need);
          u.sz  = sz;
          `DEC_EA(ea_s, sz)
          u.ld = 1'b1;
          if (!stp) begin
            u.b_sel = rd68041_pkg::S_MEM;
            u.wr = 1'b1; u.wr_reg = 5'd16;
            u.last = 1'b0; more = 1'b1;
          end else begin
            u.ea_disp = u.ea_disp + ((sz == rd68041_pkg::SZ_B) ? 32'd1 :
                                     (sz == rd68041_pkg::SZ_W) ? 32'd2 : 32'd4);
            u.a_reg = 5'd16;
            u.b_sel = rd68041_pkg::S_MEM;
            u.wr2_reg = {1'b0, x[15:12]};          // Rn, the third operand
            u.op = rd68041_pkg::OP_CMP2;
            u.trap = x[11];                        // CHK2
            pop = need;
          end
        end else if (op[7:6] == 2'b11 && sm == 3'd7 && sr == 3'd4) begin
          // CAS2 (PRM 4): both read under LOCK, compared, then either both
          // updates written or the second operand written back (UM 7.4.5),
          // and the compare registers loaded.
          logic [15:0] x1, x2;
          logic [4:0]  rn1, rn2;
          x1 = ew(w, 0); x2 = ew(w, 1);
          rn1 = {1'b0, x1[15:12]}; rn2 = {1'b0, x2[15:12]};
          sz  = (op[10:9] == 2'b10) ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
          if (op[10:9] == 2'b00 || op[10:9] == 2'b01) bad = 1'b1;
          u.cof = 1'b1;
          need  = 4'd2;
          u.len = 5'd6;
          u.sz  = sz;
          u.wr_sz = sz;
          more  = (sq != 5'd8);
          u.last = !more;
          case (sq)
            5'd0, 5'd1: begin                      // the two operands, locked
              u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_REG;
              u.ea_reg = (sq == 5'd0) ? rn1 : rn2;
              u.ld = 1'b1; u.lk = 1'b1;
              u.b_sel = rd68041_pkg::S_MEM;
              u.wr = 1'b1; u.wr_reg = (sq == 5'd0) ? 5'd16 : 5'd17;
            end
            5'd2: begin                            // compare the first
              u.a_reg = dreg(x1[2:0]); u.b_reg = 5'd16;
              u.op = rd68041_pkg::OP_CMP; u.cc = rd68041_pkg::CC_CMP;
            end
            5'd3: begin                            // ... and, if equal, the second
              u.a_reg = dreg(x2[2:0]); u.b_reg = 5'd17;
              u.op = rd68041_pkg::OP_CMP; u.cc = rd68041_pkg::CC_CMP;
              u.cx = 1'b1; u.cond = 4'h7;
            end
            5'd4, 5'd5: begin                      // equal: the updates, the second first
              u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_REG;
              u.ea_reg = (sq == 5'd4) ? rn2 : rn1;
              u.b_reg  = (sq == 5'd4) ? dreg(x2[8:6]) : dreg(x1[8:6]);
              u.st = 1'b1; u.lk = 1'b1; u.lke = (sq == 5'd5);
              u.cx = 1'b1; u.cond = 4'h7;
            end
            5'd6: begin                            // not: the second written back
              u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_REG; u.ea_reg = rn2;
              u.b_reg = 5'd17;
              u.st = 1'b1; u.lk = 1'b1; u.lke = 1'b1;
              u.cx = 1'b1; u.cond = 4'h6;
            end
            default: begin                         // ... and Dc1, Dc2 loaded
              u.b_reg = (sq == 5'd7) ? 5'd16 : 5'd17;
              u.wr = 1'b1; u.wr_reg = (sq == 5'd7) ? dreg(x1[2:0]) : dreg(x2[2:0]);
              u.cx = 1'b1; u.cond = 4'h6;
              if (sq == 5'd8) pop = 4'd2;
            end
          endcase
        end else if (op[7:6] == 2'b11) begin
          // CAS Dc,Du,<ea> (PRM 4): read under LOCK, compared with Dc; Du
          // written if equal, else the operand written back (UM 7.4.5) and Dc
          // loaded with it.
          logic [15:0] x;
          x  = ew(w, 0);
          sz = (op[10:9] == 2'b01) ? rd68041_pkg::SZ_B :
               (op[10:9] == 2'b10) ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
          if (op[10:9] == 2'b00 || !malt(sm, sr) || x[15:9] != 7'h0 || x[5:3] != 3'h0)
            bad = 1'b1;
          need  = 4'd1 + ea_len(sm, sr, sz, ew(w, 1)); pop = need;
          u.len = 5'(2 + 2*need);
          u.sz  = sz;
          `DEC_EA(ea_s, sz)
          u.cof = 1'b1;
          u.ld = 1'b1; u.st = 1'b1; u.lk = 1'b1; u.lke = 1'b1;
          u.a_reg = dreg(x[2:0]);
          u.b_sel = rd68041_pkg::S_MEM;
          u.wr2_reg = dreg(x[8:6]);                // Du, the third operand
          u.op = rd68041_pkg::OP_CAS; u.cc = rd68041_pkg::CC_CMP;
          u.wr = 1'b1; u.wr_reg = dreg(x[2:0]); u.wr_sz = sz;
        end else if (op[11:8] == 4'b1110) begin
          // MOVES (PRM 6): the source or destination in the space SFC or
          // DFC names (UM 7.4: an alternate function code access).
          logic [15:0] x;
          x  = ew(w, 0);
          sz = op[7:6];
          // Bits 10-0 of the extension word are reserved; the part ignores
          // them (the cputest corpus, doc/oracles.md).
          if (!malt(sm, sr)) bad = 1'b1;
          need  = 4'd1 + ea_len(sm, sr, sz, ew(w, 1)); pop = need;
          u.len = 5'(2 + 2*need);
          u.sz  = sz;
          `DEC_EA(ea_s, sz)
          priv = 1'b1; u.cof = 1'b1;
          u.alt = 1'b1;
          if (x[11]) begin
            u.st = 1'b1;
            u.b_reg = {1'b0, x[15:12]};
          end else begin
            u.ld = 1'b1;
            u.b_sel = rd68041_pkg::S_MEM;
            u.wr = 1'b1; u.wr_reg = {1'b0, x[15:12]};
            if (x[15]) begin
              // To An: the whole register, the operand sign-extended.
              u.b_sxw = (sz == rd68041_pkg::SZ_W);
              if (sz == rd68041_pkg::SZ_B) u.op = rd68041_pkg::OP_EXTBL;
            end else begin
              u.wr_sz = sz;
            end
          end
        end else if (op[11:9] == 3'd0 || op[11:9] == 3'd1 || op[11:9] == 3'd2 ||
                     op[11:9] == 3'd3 || op[11:9] == 3'd5 || op[11:9] == 3'd6) begin
          sz = op[7:6];
          if (sm == 3'd7 && sr == 3'd4) begin
            // To CCR (byte), and to SR (word, privileged; serialising).
            if (sz == rd68041_pkg::SZ_B &&
                (op[11:9] == 3'd0 || op[11:9] == 3'd1 || op[11:9] == 3'd5)) begin
              need =  4'd1; pop =  4'd1;
              u.len = 5'd4;
              u.a_sel = rd68041_pkg::S_IMM; u.imm = {24'h0, ewb(w, 0)};
              u.op = rd68041_pkg::OP_CCR;
              u.cc = rd68041_pkg::CC_SET;
              u.cond = (op[11:9] == 3'd0) ? 4'd1 : (op[11:9] == 3'd1) ? 4'd0 : 4'd2;
            end else if (sz == rd68041_pkg::SZ_W &&
                         (op[11:9] == 3'd0 || op[11:9] == 3'd1 || op[11:9] == 3'd5)) begin
              need =  4'd1; pop =  4'd1;
              u.len = 5'd4;
              priv = 1'b1;
              u.a_sel = rd68041_pkg::S_IMM; u.imm = {16'h0, ew(w, 0)};
              u.op = rd68041_pkg::OP_SR;
              u.fn = (op[11:9] == 3'd0) ? 4'd1 : (op[11:9] == 3'd1) ? 4'd0 : 4'd2;
              u.srw = 1'b1; u.ser = 1'b1; u.cof = 1'b1;
            end else begin
              bad = 1'b1;
            end
          end else begin
            n_src = (sz == rd68041_pkg::SZ_L) ? 4'd2 : 4'd1;
            n_dst = ea_len(sm, sr, sz, ew(w, int'(n_src)));
            need  = n_src + n_dst;
            pop   = need;
            u.len = 5'(2 + 2*need);
            u.sz  = sz;
            u.a_sel = rd68041_pkg::S_IMM; u.imm = imm_of(w, 0, sz);
            case (op[11:9])
              3'd0: begin u.op = rd68041_pkg::OP_OR;  u.cc = rd68041_pkg::CC_LOGIC; end
              3'd1: begin u.op = rd68041_pkg::OP_AND; u.cc = rd68041_pkg::CC_LOGIC; end
              3'd2: begin u.op = rd68041_pkg::OP_SUB; u.cc = rd68041_pkg::CC_SUB;   end
              3'd3: begin u.op = rd68041_pkg::OP_ADD; u.cc = rd68041_pkg::CC_ADD;   end
              3'd5: begin u.op = rd68041_pkg::OP_EOR; u.cc = rd68041_pkg::CC_LOGIC; end
              default: begin u.op = rd68041_pkg::OP_CMP; u.cc = rd68041_pkg::CC_CMP; end
            endcase
            if (op[11:9] == 3'd6) begin
              if (sm == 3'd1 || is_imm(sm, sr) || !ok_mode(sm, sr)) bad = 1'b1;
              `DEC_SRC(sz, 1'b0)
            end else if (!dalt(sm, sr)) begin
              bad = 1'b1;
            end else if (sm == 3'd0) begin
              u.b_reg = dy;
              u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = sz;
            end else begin
              `DEC_SRC(sz, 1'b0)
              u.st = 1'b1;
            end
          end
        end else begin
          bad = 1'b1;
        end
      end

      // =======================================================================
      // 0001, 0010, 0011: MOVE, MOVEA
      // =======================================================================
      4'b0001, 4'b0010, 4'b0011: begin
        // The phases (doc/microarch.md): [the source's prelude] [the source
        // into T0] [the destination's prelude] the move. ind_in says which
        // preludes were issued; ph counts the phases from 0.
        logic       mm;
        logic [4:0] ph;
        uses_ea = 1'b0;                          // MOVE handles its own preludes
        sz = (op[13:12] == 2'b01) ? rd68041_pkg::SZ_B :
             (op[13:12] == 2'b11) ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
        mm    = (sm >= 3'd2) && !is_imm(sm, sr) && (dm >= 3'd2);
        n_src = ea_len(sm, sr, sz, ew(w, 0));
        // After the first micro-operation of a memory-to-memory move the
        // source's words are gone and the destination's are at w[0] (kd): its
        // length is read there, not past them -- from the next instruction
        // (doc/bugs-found.md M13).
        n_dst = ea_len(dm, dr, sz, ew(w, (mm && seq != 5'd0) ? 0 : int'(n_src)));
        need  = n_src + n_dst;
        u.sz  = sz;
        u.len = 5'(2 + 2*need);
        if (!ok_mode(sm, sr) || (sm == 3'd1 && sz == rd68041_pkg::SZ_B)) bad = 1'b1;
        if (dm >= 3'd2 && !dalt(dm, dr)) bad = 1'b1;
        if (dm == 3'd1 && sz == rd68041_pkg::SZ_B) bad = 1'b1;
        if (seq == 5'd0) begin
          ind_out[0] = (sm >= 3'd2) && is_ind(sm, sr, ew(w, 0));
          ind_out[1] = (dm >= 3'd2) && is_ind(dm, dr, ew(w, int'(n_src)));
        end
        // Map the micro-operation to its phase: 0 source prelude, 1 source to
        // T0, 2 destination prelude, 3 the move.
        begin
          logic si, di;
          si = (seq == 5'd0) ? ind_out[0] : ind_in[0];
          di = (seq == 5'd0) ? ind_out[1] : ind_in[1];
          ph = 5'd3;
          if (mm) begin
            case (seq)
              5'd0: ph = si ? 5'd0 : 5'd1;
              5'd1: ph = si ? 5'd1 : (di ? 5'd2 : 5'd3);
              5'd2: ph = (si && di) ? 5'd2 : 5'd3;
              default: ph = 5'd3;
            endcase
          end else if (dm >= 3'd2) begin
            ph = (seq == 5'd0 && di) ? 5'd2 : 5'd3;
          end else begin
            ph = (seq == 5'd0 && si) ? 5'd0 : 5'd3;
          end
        end
        case (ph)
          5'd0: begin
            `DEC_IND(ea_sp)
            more = 1'b1;
          end
          5'd2: begin
            // Before it, with memory to memory, the source took its words.
            `DEC_IND(ea_dp)
            more = 1'b1;
          end
          5'd1: begin
            `DEC_SRC(sz, 1'b0)
            u.last = 1'b0; more = 1'b1;
            u.wr = 1'b1; u.wr_reg = 5'd16;
            pop = n_src;
          end
          default: begin
            if (dm == 3'd1) begin
              // MOVEA: no byte form; the word is sign-extended (PRM 4).
              `DEC_SRC(sz, 1'b0)
              u.b_sxw = (sz == rd68041_pkg::SZ_W);
              u.wr = 1'b1; u.wr_reg = an_r;
              pop = n_src;
            end else if (dm == 3'd0) begin
              `DEC_SRC(sz, 1'b0)
              u.cc = rd68041_pkg::CC_LOGIC;
              u.wr = 1'b1; u.wr_reg = dn; u.wr_sz = sz;
              pop = n_src;
            end else if (mm) begin
              // T0 to the destination, its extension words now at w[0].
              u.b_reg = 5'd16;
              `DEC_EA(ea_d, sz)
              u.st = 1'b1;
              u.cc = rd68041_pkg::CC_LOGIC;
              pop = n_dst;
            end else begin
              `DEC_SRC(sz, 1'b0)
              `DEC_EA(ea_d, sz)
              u.st = 1'b1;
              u.cc = rd68041_pkg::CC_LOGIC;
              pop = n_src + n_dst;
            end
          end
        endcase
      end

      // =======================================================================
      // 0100: the miscellaneous group
      // =======================================================================
      4'b0100: begin
        priority casez (op[11:0])
          12'b0000_11??_????: begin                // MOVE from SR
            if (!dalt(sm, sr)) bad = 1'b1;
            priv = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_W;
            u.crr = 1'b1; u.cr = rd68041_pkg::CR_SR;
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_W; end
            else begin `DEC_EA(ea_s, rd68041_pkg::SZ_W) u.st = 1'b1; end
          end
          12'b0010_11??_????: begin                // MOVE from CCR
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_W;
            u.crr = 1'b1; u.cr = rd68041_pkg::CR_SR;
            u.op = rd68041_pkg::OP_AND; u.a_sel = rd68041_pkg::S_IMM; u.imm = 32'h1F;
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_W; end
            else begin `DEC_EA(ea_s, rd68041_pkg::SZ_W) u.st = 1'b1; end
          end
          12'b0100_11??_????: begin                // MOVE to CCR
            if (sm == 3'd1 || !ok_mode(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_W;
            `DEC_SRC(rd68041_pkg::SZ_W, 1'b1)
            u.op = rd68041_pkg::OP_TOCCR;
            u.cc = rd68041_pkg::CC_SET; u.cond = 4'd3;
          end
          12'b0110_11??_????: begin                // MOVE to SR
            if (sm == 3'd1 || !ok_mode(sm, sr)) bad = 1'b1;
            priv = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_W;
            `DEC_SRC(rd68041_pkg::SZ_W, 1'b1)
            u.op = rd68041_pkg::OP_SR; u.fn = 4'd3;
            u.srw = 1'b1; u.ser = 1'b1; u.cof = 1'b1;
          end
          12'b0010_????_????: begin                // CLR <ea>
            sz = op[7:6];
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = sz;
            u.op = rd68041_pkg::OP_CLR; u.cc = rd68041_pkg::CC_LOGIC;
            if (sm == 3'd0) begin
              u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = sz;
            end else begin
              // PRM 4: the MC68020 on does not read the operand first.
              `DEC_EA(ea_s, sz)
              u.st = 1'b1;
            end
          end
          12'b0000_????_????,                      // NEGX
          12'b0100_????_????,                      // NEG
          12'b0110_????_????: begin                // NOT
            sz = op[7:6];
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = sz;
            case (op[10:9])
              2'b00:   begin u.op = rd68041_pkg::OP_NEGX; u.cc = rd68041_pkg::CC_SUB; end
              2'b10:   begin u.op = rd68041_pkg::OP_NEG;  u.cc = rd68041_pkg::CC_SUB; end
              default: begin u.op = rd68041_pkg::OP_NOT;  u.cc = rd68041_pkg::CC_LOGIC; end
            endcase
            `DEC_SRC(sz, 1'b0)
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = sz; end
            else            u.st = 1'b1;
          end
          12'b1001_1100_0???: begin                // EXTB.L (before LEA, whose encoding it shares)
            u.len = 5'd2;
            u.b_reg = dy;
            u.op = rd68041_pkg::OP_EXTBL;
            u.cc = rd68041_pkg::CC_LOGIC;
            u.wr = 1'b1; u.wr_reg = dy;
          end
          12'b???1_11??_????: begin                // LEA <ea>,An
            if (!ctrl(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            `DEC_EA(ea_s, rd68041_pkg::SZ_L)
            u.b_sel = rd68041_pkg::S_EA;
            u.wr = 1'b1; u.wr_reg = an_r;
          end
          12'b???1_10??_????,                      // CHK.W
          12'b???1_00??_????: begin                // CHK.L
            sz = op[7] ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
            if (sm == 3'd1 || !ok_mode(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = sz;
            `DEC_SRC(sz, 1'b1)
            u.b_reg = dn;
            u.op = rd68041_pkg::OP_CMP;
            u.trap = 1'b1;
          end
          12'b1000_0000_1???: begin                // LINK.L An,#d32
            need =  4'd2; pop =  4'd2;
            u.len = 5'd6;
            `DEC_EA(ea_fix(3'd4, 3'd7, w0), rd68041_pkg::SZ_L)
            u.a_reg = ay;
            u.imm = {ew(w, 0), ew(w, 1)};
            u.op = rd68041_pkg::OP_LINK;
            u.st = 1'b1; u.sz = rd68041_pkg::SZ_L;
            u.wr = (op[2:0] != 3'd7); u.wr_reg = ay;
            u.wr2 = 1'b1; u.wr2_reg = 5'd15;
          end
          12'b1000_00??_????: begin                // NBCD <ea>
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_B, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_B;
            u.op = rd68041_pkg::OP_BCD; u.fn = 4'd2;
            `DEC_SRC(rd68041_pkg::SZ_B, 1'b0)
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_B; end
            else            u.st = 1'b1;
          end
          12'b1000_0100_0???: begin                // SWAP Dn
            u.len = 5'd2;
            u.b_reg = dy;
            u.op = rd68041_pkg::OP_SWAP; u.cc = rd68041_pkg::CC_LOGIC;
            u.wr = 1'b1; u.wr_reg = dy;
          end
          12'b1000_0100_1???: begin                // BKPT #n
            // UM 8.2.8, 7.5.2: a breakpoint acknowledge cycle (TT 11, TM 0,
            // address 0, a byte); whether TA or TEA ends it, an illegal
            // instruction exception.
            u.len = 5'd2;
            u.ea_en = 1'b1;
            u.ld = 1'b1; u.sz = rd68041_pkg::SZ_B; u.ack = 1'b1; u.fn = 4'd0;
            u.xvec = 8'd4;
          end
          12'b1000_01??_????: begin                // PEA <ea>
            if (!ctrl(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 0));
            u.len = 5'(2 + 2*need);
            if (!stp) begin
              `DEC_EA(ea_s, rd68041_pkg::SZ_L)
              pop = need;
              u.b_sel = rd68041_pkg::S_EA;
              u.wr = 1'b1; u.wr_reg = 5'd16;
              u.last = 1'b0; more = 1'b1;
            end else begin
              `DEC_EA(ea_fix(3'd4, 3'd7, w0), rd68041_pkg::SZ_L)
              u.b_reg = 5'd16;
              u.st = 1'b1; u.sz = rd68041_pkg::SZ_L;
            end
          end
          12'b1000_1?00_0???: begin                // EXT.W, EXT.L
            u.len = 5'd2;
            u.b_reg = dy;
            u.op = op[6] ? rd68041_pkg::OP_EXTL : rd68041_pkg::OP_EXTW;
            u.sz = op[6] ? rd68041_pkg::SZ_L : rd68041_pkg::SZ_W;
            u.cc = rd68041_pkg::CC_LOGIC;
            u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = u.sz;
          end
          12'b1?00_1???_????: begin                // MOVEM
            // op[10]: to registers; op[6]: long. The mask is w[0].
            logic ld_form, pre, post, prel;
            logic [15:0] mask0;
            ld_form = op[10];
            mask0 = ew(w, 0);
            pre  = (sm == 3'd4);
            post = (sm == 3'd3);
            sz   = op[6] ? rd68041_pkg::SZ_L : rd68041_pkg::SZ_W;
            if (ld_form ? !(ctrl(sm, sr) || post) : !((ctrl(sm, sr) && malt(sm, sr)) || pre))
              bad = 1'b1;
            need  =  4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1));
            u.len = 5'(2 + 2*need);
            u.sz  = sz;
            // A prelude for the control modes (T0 = <ea>), and for -(An) when
            // An is in the list (T0 = An - size, what PRM 4 says is written
            // for it). The list's bit for An under -(An), whose mask is
            // reversed, is 7 - n. Every extension word goes with the first
            // micro-operation; the decode stage keeps the mask from there.
            prel = !(pre || post) || (pre && mask0[4'(7 - sr)]);
            if (!stp) pop = need;
            if (!stp && mask0 == 16'h0000) begin
              // No registers: nothing moves, and An is left alone.
            end else if (!stp && prel) begin
              if (pre) begin
                `DEC_EA(ea_fix(3'd5, sr, w0), rd68041_pkg::SZ_L)
                u.ea_disp = (sz == rd68041_pkg::SZ_L) ? -32'd4 : -32'd2;
              end else begin
                `DEC_EA(ea_s, rd68041_pkg::SZ_L)
              end
              u.b_sel = rd68041_pkg::S_EA;
              u.wr = 1'b1; u.wr_reg = 5'd16;
              u.last = 1'b0; more = 1'b1;
            end else begin
              // One register.
              mv_used = 1'b1;
              more    = !mv_last;
              u.last  = mv_last;
              if (pre || post) begin
                `DEC_EA(ea_s, sz)
              end else begin
                u.ea_en = 1'b1;
                u.ea_base = rd68041_pkg::B_REG; u.ea_reg = 5'd16;
                u.ea_disp = (sz == rd68041_pkg::SZ_L) ? {25'h0, k, 2'b00} : {26'h0, k, 1'b0};
              end
              if (ld_form) begin
                u.ld = 1'b1;
                u.b_sel = rd68041_pkg::S_MEM;
                u.b_sxw = (sz == rd68041_pkg::SZ_W);
                // PRM 4: with (An)+, An itself is not loaded.
                u.wr = !(post && mv_reg == {1'b1, sr});
                u.wr_reg = {1'b0, mv_reg};
              end else begin
                u.st = 1'b1;
                u.b_reg = (pre && mv_reg == {1'b1, sr}) ? 5'd16 : {1'b0, mv_reg};
              end
            end
          end
          12'b1010_1111_1100: bad = 1'b1;          // ILLEGAL: vector 4 (UM 8.2.4)
          12'b1010_11??_????: begin                // TAS <ea>: a locked read-modify-write
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_B, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_B;
            u.op = rd68041_pkg::OP_TAS;
            `DEC_SRC(rd68041_pkg::SZ_B, 1'b0)
            if (sm == 3'd0) begin
              u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_B;
            end else begin
              u.st = 1'b1; u.lk = 1'b1; u.lke = 1'b1;
            end
          end
          12'b1010_????_????: begin                // TST <ea>
            sz = op[7:6];
            if (!ok_mode(sm, sr) || (sm == 3'd1 && sz == rd68041_pkg::SZ_B)) bad = 1'b1;
            need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = sz;
            `DEC_SRC(sz, 1'b0)
            u.cc = rd68041_pkg::CC_LOGIC;
          end
          12'b1100_00??_????: begin                // MULU.L, MULS.L
            logic [15:0] x;
            x = ew(w, 0);
            if (sm == 3'd1 || !ok_mode(sm, sr) || x[15] || x[9:3] != 7'h0) bad = 1'b1;
            need =  4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_L;
            `DEC_SRC(rd68041_pkg::SZ_L, 1'b1)
            u.b_reg = dreg(x[14:12]);
            u.op = rd68041_pkg::OP_MUL;
            u.fn = {1'b0, x[10], 1'b1, x[11]};
            u.wr = 1'b1; u.wr_reg = dreg(x[14:12]);
            if (x[10]) begin u.wr2 = 1'b1; u.wr2_reg = dreg(x[2:0]); end
          end
          12'b1100_01??_????: begin                // DIVU.L, DIVS.L
            logic [15:0] x;
            x = ew(w, 0);
            if (sm == 3'd1 || !ok_mode(sm, sr) || x[15] || x[9:3] != 7'h0) bad = 1'b1;
            need =  4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_L;
            `DEC_SRC(rd68041_pkg::SZ_L, 1'b1)
            u.b_reg = dreg(x[14:12]);                // Dq
            u.op = rd68041_pkg::OP_DIV;
            u.fn = {1'b0, x[10], 1'b1, x[11]};
            u.wr = 1'b1; u.wr_reg = dreg(x[14:12]);
            // The remainder, when there is a register for it (DIVxL.L, or the
            // 64-bit form); wr2_reg is also the 64-bit dividend's high half.
            u.wr2_reg = dreg(x[2:0]);
            u.wr2 = (x[2:0] != x[14:12]);
            u.cc = rd68041_pkg::CC_NONE;
          end
          12'b1110_0100_????: begin                // TRAP #n: vector 32 + n (UM 8.2.3)
            u.len = 5'd2;
            u.xvec = 8'd32 + {4'h0, op[3:0]}; u.xnext = 1'b1;
          end
          12'b1110_0101_0???: begin                // LINK.W An,#d16
            need =  4'd1; pop =  4'd1;
            u.len = 5'd4;
            `DEC_EA(ea_fix(3'd4, 3'd7, w0), rd68041_pkg::SZ_L)
            u.a_reg = ay;
            u.imm = ews(w, 0);
            u.op = rd68041_pkg::OP_LINK;
            u.st = 1'b1; u.sz = rd68041_pkg::SZ_L;
            u.wr = (op[2:0] != 3'd7); u.wr_reg = ay;
            u.wr2 = 1'b1; u.wr2_reg = 5'd15;
          end
          12'b1110_0101_1???: begin                // UNLK An
            u.len = 5'd2;
            `DEC_EA(ea_fix(3'd2, sr, w0), rd68041_pkg::SZ_L)
            u.ld = 1'b1; u.sz = rd68041_pkg::SZ_L;
            u.b_sel = rd68041_pkg::S_MEM;
            u.wr = 1'b1; u.wr_reg = ay;
            u.imm = 32'd4;
            u.wr2 = 1'b1; u.wr2_reg = 5'd15;
          end
          12'b1110_0110_0???: begin                // MOVE An,USP
            u.len = 5'd2;
            priv = 1'b1; u.cof = 1'b1;
            u.b_reg = ay;
            u.crw = 1'b1; u.cr = rd68041_pkg::CR_USP;
          end
          12'b1110_0110_1???: begin                // MOVE USP,An
            u.len = 5'd2;
            priv = 1'b1; u.cof = 1'b1;
            u.crr = 1'b1; u.cr = rd68041_pkg::CR_USP;
            u.wr = 1'b1; u.wr_reg = ay;
          end
          12'b1110_0111_0000: begin                // RESET: RSTO for 512 clocks (UM 5.8)
            u.len = 5'd2;
            priv = 1'b1;
            u.ser = 1'b1; u.rsto = 1'b1;
          end
          12'b1110_0111_0001: begin                // NOP
            // UM 2.1 (and PRM 4): NOP waits for the pipeline, the writes
            // pending included, to finish: serialising does that.
            u.len = 5'd2;
            u.ser = 1'b1; u.cof = 1'b1;
          end
          12'b1110_0111_0010: begin                // STOP #imm
            need =  4'd1; pop =  4'd1;
            u.len = 5'd4;
            priv = 1'b1;
            u.a_sel = rd68041_pkg::S_IMM; u.imm = {16'h0, ew(w, 0)};
            u.op = rd68041_pkg::OP_SR; u.fn = 4'd3;
            u.srw = 1'b1; u.ser = 1'b1; u.stp = 1'b1; u.cof = 1'b1;
          end
          12'b1110_0111_0011: begin                // RTE
            // The SR into T0, the PC into T1, the format word: the stack past
            // the frame into A7 (or a format error) and the word into T2;
            // then the SR and the PC (UM 8.4). A throwaway frame ($1) runs
            // RTE again, on the stack the SR it loaded names (UM 8.4.2).
            u.len = 5'd2;
            priv = 1'b1;
            more = (sq != 5'd3);
            u.last = !more;
            case (sq)
              5'd0: begin
                `DEC_EA(ea_fix(3'd2, 3'd7, w0), rd68041_pkg::SZ_W)
                u.ld = 1'b1; u.sz = rd68041_pkg::SZ_W;
                u.b_sel = rd68041_pkg::S_MEM;
                u.wr = 1'b1; u.wr_reg = 5'd16;
              end
              5'd1: begin
                `DEC_EA(ea_fix(3'd2, 3'd7, w0), rd68041_pkg::SZ_L)
                u.ea_disp = 32'd2;
                u.ld = 1'b1; u.sz = rd68041_pkg::SZ_L;
                u.b_sel = rd68041_pkg::S_MEM;
                u.wr = 1'b1; u.wr_reg = 5'd17;
              end
              5'd2: begin
                `DEC_EA(ea_fix(3'd2, 3'd7, w0), rd68041_pkg::SZ_W)
                u.ea_disp = 32'd6;
                u.ld = 1'b1; u.sz = rd68041_pkg::SZ_W;
                u.b_sel = rd68041_pkg::S_MEM;
                u.op = rd68041_pkg::OP_RTEF;
                u.wr = 1'b1; u.wr_reg = 5'd15;
                u.wr2 = 1'b1; u.wr2_reg = 5'd18;
              end
              default: begin
                u.a_reg = 5'd16; u.b_reg = 5'd17; u.wr2_reg = 5'd18;
                u.op = rd68041_pkg::OP_RTE;
                u.srw = 1'b1; u.ser = 1'b1; u.cof = 1'b1;
              end
            endcase
          end
          12'b1110_0111_0100: begin                // RTD #d16
            need =  4'd1; pop =  4'd1;
            u.len = 5'd4;
            `DEC_EA(ea_fix(3'd3, 3'd7, w0), rd68041_pkg::SZ_L)
            u.ld = 1'b1; u.sz = rd68041_pkg::SZ_L;
            u.b_sel = rd68041_pkg::S_MEM;
            u.br = rd68041_pkg::BR_MEM;
            u.imm = ews(w, 0) + 32'd4;
            u.wr2 = 1'b1; u.wr2_reg = 5'd15;
          end
          12'b1110_0111_0101: begin                // RTS
            u.len = 5'd2;
            `DEC_EA(ea_fix(3'd3, 3'd7, w0), rd68041_pkg::SZ_L)
            u.ld = 1'b1; u.sz = rd68041_pkg::SZ_L;
            u.br = rd68041_pkg::BR_MEM;
          end
          12'b1110_0111_0110: begin                // TRAPV
            u.len = 5'd2;
            u.trap = 1'b1; u.cond = 4'h9;            // VS
          end
          12'b1110_0111_0111: begin                // RTR: the CCR, then RTS
            u.len = 5'd2;
            if (!stp) begin
              `DEC_EA(ea_fix(3'd3, 3'd7, w0), rd68041_pkg::SZ_W)
              u.ld = 1'b1; u.sz = rd68041_pkg::SZ_W;
              u.a_sel = rd68041_pkg::S_MEM;
              u.op = rd68041_pkg::OP_TOCCR;
              u.cc = rd68041_pkg::CC_SET; u.cond = 4'd3;
              u.last = 1'b0; more = 1'b1;
            end else begin
              `DEC_EA(ea_fix(3'd3, 3'd7, w0), rd68041_pkg::SZ_L)
              u.ld = 1'b1; u.sz = rd68041_pkg::SZ_L;
              u.br = rd68041_pkg::BR_MEM;
            end
          end
          12'b1110_0111_101?: begin                // MOVEC
            // The active supervisor stack pointer (the ISP, or the MSP when
            // M is set) is A7; the other two are the core's.
            logic [15:0] x;
            logic [3:0]  c;
            logic        act;
            x = ew(w, 0);
            c = cr_of(x[11:0]);
            act = (c == (msp_on ? rd68041_pkg::CR_MSP : rd68041_pkg::CR_ISP));
            need =  4'd1; pop =  4'd1;
            u.len = 5'd4;
            priv = 1'b1; u.cof = 1'b1;
            if (c == 4'hF) bad = 1'b1;
            if (op[0]) begin                     // Rn -> Rc
              // Serialising: what comes after sees the register as written
              // -- the caches' enables, the TTRs' cache modes, VBR.
              u.ser = 1'b1;
              u.b_reg = {1'b0, x[15:12]};
              if (act) begin u.wr = 1'b1; u.wr_reg = 5'd15; end
              else begin u.crw = 1'b1; u.cr = c; end
            end else begin                       // Rc -> Rn
              if (act) u.b_reg = 5'd15;
              else begin u.crr = 1'b1; u.cr = c; end
              u.wr = 1'b1; u.wr_reg = {1'b0, x[15:12]};
            end
          end
          12'b1110_10??_????: begin                // JSR <ea>
            if (!ctrl(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 0));
            u.len = 5'(2 + 2*need);
            if (!stp) begin
              `DEC_EA(ea_s, rd68041_pkg::SZ_L)
              pop = need;
              u.b_sel = rd68041_pkg::S_EA;
              u.wr = 1'b1; u.wr_reg = 5'd16;
              u.last = 1'b0; more = 1'b1;
            end else begin
              `DEC_EA(ea_fix(3'd4, 3'd7, w0), rd68041_pkg::SZ_L)
              // The return address: the instruction's length is the first
              // micro-operation's, the extension words being gone by now.
              u.a_sel = rd68041_pkg::S_IMM; u.imm = pc + {27'h0, len_in};
              u.op = rd68041_pkg::OP_PASSA;
              u.st = 1'b1; u.sz = rd68041_pkg::SZ_L;
              u.b_reg = 5'd16;
              u.br = rd68041_pkg::BR_EA;
            end
          end
          12'b1110_11??_????: begin                // JMP <ea>
            if (!ctrl(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            `DEC_EA(ea_s, rd68041_pkg::SZ_L)
            u.b_sel = rd68041_pkg::S_EA;
            u.br = rd68041_pkg::BR_EA;
          end
          default: bad = 1'b1;
        endcase
      end

      // =======================================================================
      // 0101: ADDQ, SUBQ, Scc, DBcc, TRAPcc
      // =======================================================================
      4'b0101: begin
        if (op[7:6] == 2'b11) begin
          if (sm == 3'd1) begin
            // DBcc Dn,<label>
            need =  4'd1; pop =  4'd1;
            u.len = 5'd4;
            u.b_reg = dy;
            u.op = rd68041_pkg::OP_DBCC;
            u.cond = op[11:8];
            u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_W;
            u.br = rd68041_pkg::BR_DISP;
            u.tgt = pc + 32'd2 + ews(w, 0);
          end else if (sm == 3'd7 && sr >= 3'd2) begin
            // TRAPcc, with no operand, a word or a long (PRM 4).
            if (sr > 3'd4) bad = 1'b1;
            need = (sr == 3'd2) ? 4'd1 : (sr == 3'd3) ? 4'd2 : 4'd0;
            pop  = need;
            u.len = 5'(2 + 2*need);
            u.trap = 1'b1; u.cond = op[11:8];
          end else begin
            // Scc <ea>
            if (!dalt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_B, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_B;
            u.op = rd68041_pkg::OP_SCC;
            u.cond = op[11:8];
            if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_B; end
            else begin `DEC_EA(ea_s, rd68041_pkg::SZ_B) u.st = 1'b1; end
          end
        end else begin
          sz = op[7:6];
          if (!ok_mode(sm, sr) || !((sm == 3'd1) || dalt(sm, sr)) ||
              (sm == 3'd1 && sz == rd68041_pkg::SZ_B)) begin
            bad = 1'b1;
          end else begin
            need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = sz;
            u.a_sel = rd68041_pkg::S_IMM; u.imm = (op[11:9] == 3'd0) ? 32'd8 : {29'h0, op[11:9]};
            if (sm == 3'd1) begin
              // To An: the whole register, no flags (PRM 4, ADDQ).
              u.op = op[8] ? rd68041_pkg::OP_SUBA : rd68041_pkg::OP_ADDA;
              u.sz = rd68041_pkg::SZ_L;
              u.b_reg = ay;
              u.wr = 1'b1; u.wr_reg = ay;
            end else begin
              u.op = op[8] ? rd68041_pkg::OP_SUB : rd68041_pkg::OP_ADD;
              u.cc = op[8] ? rd68041_pkg::CC_SUB : rd68041_pkg::CC_ADD;
              `DEC_SRC(sz, 1'b0)
              if (sm == 3'd0) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = sz; end
              else            u.st = 1'b1;
            end
          end
        end
      end

      // =======================================================================
      // 0110: BRA, BSR, Bcc
      // =======================================================================
      4'b0110: begin
        logic [31:0] d;
        if (op[7:0] == 8'h00) begin
          need =  4'd1; d = ews(w, 0);
        end else if (op[7:0] == 8'hFF) begin
          need =  4'd2; d = {ew(w, 0), ew(w, 1)};
        end else begin
          need =  4'd0; d = {{24{op[7]}}, op[7:0]};
        end
        pop = need;
        u.len = 5'(2 + 2*need);
        // PRM 4: "the PC contains the instruction address plus two".
        u.tgt = pc + 32'd2 + d;
        u.br  = rd68041_pkg::BR_DISP;
        u.cond = (op[11:8] == 4'd1) ? 4'd0 : op[11:8];    // BSR's 1 is "true", as BRA's 0
        if (op[11:8] == 4'd1) begin
          `DEC_EA(ea_fix(3'd4, 3'd7, w0), rd68041_pkg::SZ_L)
          u.a_sel = rd68041_pkg::S_IMM; u.imm = pc + 32'(2 + 2*need);
          u.op = rd68041_pkg::OP_PASSA;
          u.st = 1'b1; u.sz = rd68041_pkg::SZ_L;
        end
      end

      // =======================================================================
      // 0111: MOVEQ
      // =======================================================================
      4'b0111: begin
        if (op[8]) bad = 1'b1;
        u.len = 5'd2;
        u.b_sel = rd68041_pkg::S_IMM; u.imm = {{24{op[7]}}, op[7:0]};
        u.cc = rd68041_pkg::CC_LOGIC;
        u.wr = 1'b1; u.wr_reg = dn;
      end

      // =======================================================================
      // 1000 OR, DIV.W, SBCD, PACK, UNPK; 1001 SUB, SUBA, SUBX; 1011 CMP, CMPA,
      // EOR, CMPM; 1100 AND, MUL.W, ABCD, EXG; 1101 ADD, ADDA, ADDX
      // =======================================================================
      4'b1000, 4'b1001, 4'b1011, 4'b1100, 4'b1101: begin
        logic [2:0] om;
        logic       addx_form;
        om = op[8:6];
        sz = (om == 3'd3) ? rd68041_pkg::SZ_W : (om == 3'd7) ? rd68041_pkg::SZ_L : om[1:0];
        addx_form = om[2] && om[1:0] != 2'b11 && (sm == 3'd0 || sm == 3'd1);
        if ((op[15:12] == 4'b1000 || op[15:12] == 4'b1100) && (om == 3'd3 || om == 3'd7)) begin
          // DIVU.W, DIVS.W (1000); MULU.W, MULS.W (1100)
          if (sm == 3'd1 || !ok_mode(sm, sr)) bad = 1'b1;
          need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
          u.len = 5'(2 + 2*need);
          u.sz = rd68041_pkg::SZ_W;
          `DEC_SRC(rd68041_pkg::SZ_W, 1'b1)
          u.b_reg = dn;
          u.op = (op[15:12] == 4'b1000) ? rd68041_pkg::OP_DIV : rd68041_pkg::OP_MUL;
          u.fn = {2'b00, 1'b0, om[2]};
          u.wr = 1'b1; u.wr_reg = dn;
        end else if ((op[15:12] == 4'b1000 || op[15:12] == 4'b1100) && om[2] &&
                     (sm == 3'd0 || sm == 3'd1) && om[1:0] == 2'b00) begin
          // ABCD, SBCD: Dy,Dx or -(Ay),-(Ax), as ADDX and SUBX.
          u.sz = rd68041_pkg::SZ_B;
          u.op = rd68041_pkg::OP_BCD; u.fn = (op[15:12] == 4'b1100) ? 4'd0 : 4'd1;
          u.len = 5'd2;
          if (sm == 3'd0) begin
            u.a_reg = dy; u.b_reg = dn;
            u.wr = 1'b1; u.wr_reg = dn; u.wr_sz = rd68041_pkg::SZ_B;
          end else if (!stp) begin
            `DEC_EA(ea_fix(3'd4, sr, w0), rd68041_pkg::SZ_B)
            u.ld = 1'b1;
            u.b_sel = rd68041_pkg::S_MEM;
            u.op = rd68041_pkg::OP_PASSB;
            u.wr = 1'b1; u.wr_reg = 5'd16;
            u.last = 1'b0; more = 1'b1;
          end else begin
            `DEC_EA(ea_fix(3'd4, dr, w0), rd68041_pkg::SZ_B)
            u.ld = 1'b1; u.st = 1'b1;
            u.a_reg = 5'd16;
            u.b_sel = rd68041_pkg::S_MEM;
          end
        end else if (op[15:12] == 4'b1000 && om[2] && (sm == 3'd0 || sm == 3'd1)) begin
          // PACK (10100), UNPK (11000): Dx,Dy,#adj or -(Ax),-(Ay),#adj; the
          // source at bits 2-0, the destination at 11-9 (PRM 4).
          logic unpk;
          // Bits 8-4: 10100 PACK, 11000 UNPK.
          unpk = (op[7:4] == 4'b1000);
          if (op[7:4] != 4'b0100 && op[7:4] != 4'b1000) bad = 1'b1;
          need  = 4'd1;
          u.len = 5'd4;
          u.imm = {16'h0, ew(w, 0)};
          u.a_sel = rd68041_pkg::S_REG;
          if (sm == 3'd0) begin
            pop = 4'd1;
            u.b_reg = dy;
            u.op = unpk ? rd68041_pkg::OP_UNPK : rd68041_pkg::OP_PACK;
            u.wr = 1'b1; u.wr_reg = dn;
            u.wr_sz = unpk ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_B;
          end else if (!unpk) begin
            // PACK: the first byte read is the low one; then the high, the
            // pack; then the byte written.
            u.sz = rd68041_pkg::SZ_B;
            more = (sq != 5'd2);
            u.last = !more;
            case (sq)
              5'd0: begin
                `DEC_EA(ea_fix(3'd4, sr, w0), rd68041_pkg::SZ_B)
                u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM;
                u.wr = 1'b1; u.wr_reg = 5'd16;
              end
              5'd1: begin
                `DEC_EA(ea_fix(3'd4, sr, w0), rd68041_pkg::SZ_B)
                u.imm = {16'h0, ew(w, 0)};
                u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM;
                u.a_reg = 5'd16;
                u.op = rd68041_pkg::OP_PACK; u.fn = 4'd1;
                u.wr = 1'b1; u.wr_reg = 5'd17;
              end
              default: begin
                `DEC_EA(ea_fix(3'd4, dr, w0), rd68041_pkg::SZ_B)
                u.b_reg = 5'd17;
                u.st = 1'b1;
                pop = 4'd1;
              end
            endcase
          end else begin
            // UNPK: the byte read and unpacked; the low byte written, then the
            // high.
            u.sz = rd68041_pkg::SZ_B;
            more = (sq != 5'd2);
            u.last = !more;
            case (sq)
              5'd0: begin
                `DEC_EA(ea_fix(3'd4, sr, w0), rd68041_pkg::SZ_B)
                u.imm = {16'h0, ew(w, 0)};
                u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM;
                u.op = rd68041_pkg::OP_UNPK;
                u.wr = 1'b1; u.wr_reg = 5'd16;
              end
              5'd1: begin
                `DEC_EA(ea_fix(3'd4, dr, w0), rd68041_pkg::SZ_B)
                u.b_reg = 5'd16;
                u.st = 1'b1;
              end
              default: begin
                `DEC_EA(ea_fix(3'd4, dr, w0), rd68041_pkg::SZ_B)
                u.b_reg = 5'd16;
                u.op = rd68041_pkg::OP_MOVEP; u.fn = 4'b1001;
                u.st = 1'b1;
                pop = 4'd1;
              end
            endcase
          end
        end else if (op[15:12] == 4'b1100 && om[2] && (sm == 3'd0 || sm == 3'd1)) begin
          // EXG (PRM 4): 01000 Dx,Dy; 01001 Ax,Ay; 10001 Dx,Ay.
          u.len = 5'd2;
          case (op[7:3])
            5'b01000: begin u.a_reg = dn;   u.b_reg = dy; end
            5'b01001: begin u.a_reg = an_r; u.b_reg = ay; end
            5'b10001: begin u.a_reg = dn;   u.b_reg = ay; end
            default:  bad = 1'b1;
          endcase
          u.op = rd68041_pkg::OP_EXG;
          u.wr = 1'b1;  u.wr_reg = u.b_reg;        // the second register gets the first's
          u.wr2 = 1'b1; u.wr2_reg = u.a_reg;       // ... and the first the second's
        end else if ((op[15:12] == 4'b1001 || op[15:12] == 4'b1101) && addx_form) begin
          // ADDX, SUBX: Dy,Dx or -(Ay),-(Ax).
          u.sz = sz;
          u.op = (op[15:12] == 4'b1101) ? rd68041_pkg::OP_ADDX : rd68041_pkg::OP_SUBX;
          u.cc = (op[15:12] == 4'b1101) ? rd68041_pkg::CC_ADD : rd68041_pkg::CC_SUB;
          u.len = 5'd2;
          if (sm == 3'd0) begin
            u.a_reg = dy; u.b_reg = dn;
            u.wr = 1'b1; u.wr_reg = dn; u.wr_sz = sz;
          end else if (!stp) begin
            `DEC_EA(ea_fix(3'd4, sr, w0), sz)
            u.ld = 1'b1;
            u.b_sel = rd68041_pkg::S_MEM;
            u.op = rd68041_pkg::OP_PASSB; u.cc = rd68041_pkg::CC_NONE;
            u.wr = 1'b1; u.wr_reg = 5'd16;
            u.last = 1'b0; more = 1'b1;
          end else begin
            `DEC_EA(ea_fix(3'd4, dr, w0), sz)
            u.ld = 1'b1; u.st = 1'b1;
            u.a_reg = 5'd16;
            u.b_sel = rd68041_pkg::S_MEM;
          end
        end else if (op[15:12] == 4'b1011 && om[2] && om[1:0] != 2'b11 && sm == 3'd1) begin
          // CMPM (Ay)+,(Ax)+
          u.sz = sz;
          u.len = 5'd2;
          if (!stp) begin
            `DEC_EA(ea_fix(3'd3, sr, w0), sz)
            u.ld = 1'b1;
            u.b_sel = rd68041_pkg::S_MEM;
            u.wr = 1'b1; u.wr_reg = 5'd16;
            u.last = 1'b0; more = 1'b1;
          end else begin
            `DEC_EA(ea_fix(3'd3, dr, w0), sz)
            u.ld = 1'b1;
            u.a_reg = 5'd16;
            u.b_sel = rd68041_pkg::S_MEM;
            u.op = rd68041_pkg::OP_CMP; u.cc = rd68041_pkg::CC_CMP;
          end
        end else begin
          need = ea_len(sm, sr, sz, ew(w, 0)); pop = need;
          u.len = 5'(2 + 2*need);
          u.sz = sz;
          case (op[15:12])
            4'b1000: begin u.op = rd68041_pkg::OP_OR;  u.cc = rd68041_pkg::CC_LOGIC; end
            4'b1100: begin u.op = rd68041_pkg::OP_AND; u.cc = rd68041_pkg::CC_LOGIC; end
            4'b1001: begin u.op = rd68041_pkg::OP_SUB; u.cc = rd68041_pkg::CC_SUB;   end
            4'b1101: begin u.op = rd68041_pkg::OP_ADD; u.cc = rd68041_pkg::CC_ADD;   end
            default: begin
              u.op = om[2] ? rd68041_pkg::OP_EOR : rd68041_pkg::OP_CMP;
              u.cc = om[2] ? rd68041_pkg::CC_LOGIC : rd68041_pkg::CC_CMP;
            end
          endcase
          if (!ok_mode(sm, sr)) bad = 1'b1;
          if (om == 3'd3 || om == 3'd7) begin
            // ADDA, SUBA, CMPA.
            `DEC_SRC(sz, 1'b1)
            u.a_sxw = (om == 3'd3);
            u.b_reg = an_r;
            if (op[15:12] == 4'b1011) begin
              // PRM 4: compared at 32 bits, the word extended (the core's alu_sz).
              u.op = rd68041_pkg::OP_CMP; u.cc = rd68041_pkg::CC_CMP;
            end else begin
              u.op = (op[15:12] == 4'b1101) ? rd68041_pkg::OP_ADDA : rd68041_pkg::OP_SUBA;
              u.cc = rd68041_pkg::CC_NONE;
              u.wr = 1'b1; u.wr_reg = an_r;
            end
          end else if (!om[2]) begin
            // <ea> op Dn -> Dn (CMP: flags only). An as a source is word or long.
            if ((op[15:12] == 4'b1000 || op[15:12] == 4'b1100) && sm == 3'd1) bad = 1'b1;
            if (sm == 3'd1 && sz == rd68041_pkg::SZ_B) bad = 1'b1;
            `DEC_SRC(sz, 1'b1)
            u.b_reg = dn;
            if (op[15:12] != 4'b1011) begin
              u.wr = 1'b1; u.wr_reg = dn; u.wr_sz = sz;
            end
          end else begin
            // Dn op <ea> -> <ea>; EOR's register form is EOR Dn,Dm.
            u.a_reg = dn;
            if (op[15:12] == 4'b1011 && sm == 3'd0) begin
              u.b_reg = dy;
              u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = sz;
            end else if (sm <= 3'd1 || !malt(sm, sr)) begin
              bad = 1'b1;
            end else begin
              `DEC_SRC(sz, 1'b0)
              u.st = 1'b1;
            end
          end
        end
      end

      // =======================================================================
      // 1110: the shifts and rotates (bit fields: M7, part 2)
      // =======================================================================
      4'b1110: begin
        if (op[7:6] == 2'b11) begin
          if (op[11]) begin
            // The bit fields (PRM 4): <ea>{offset:width}, both from the
            // extension word or from data registers. The width and offset
            // into T3 (and, for memory, the field's byte address into T0);
            // for memory, the five bytes read into T1 and T2; the operation;
            // for memory and a modifying one, the five bytes written back.
            logic [15:0] x;
            logic [2:0]  t;
            logic        rg, md, mm;
            x  = ew(w, 0);
            t  = op[10:8];              // 0 TST 1 EXTU 2 CHG 3 EXTS 4 CLR 5 FFO 6 SET 7 INS
            rg = (sm == 3'd0);
            md = (t == 3'd2) || (t == 3'd4) || (t == 3'd6) || (t == 3'd7);
            mm = md && !rg;
            if (!(rg || (md ? (ctrl(sm, sr) && malt(sm, sr)) : ctrl(sm, sr)))) bad = 1'b1;
            need  = 4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1));
            u.len = 5'(2 + 2*need);
            u.sz  = rd68041_pkg::SZ_L;
            more  = rg ? (sq == 5'd0) : (sq != (mm ? 5'd5 : 5'd3));
            u.last = !more;
            if (!more) pop = need;
            if (sq == 5'd0) begin
              // The parameters, and for memory the address.
              if (x[11]) u.a_reg = dreg(x[8:6]);
              else begin u.a_sel = rd68041_pkg::S_IMM; u.imm = {27'h0, x[10:6]}; end
              if (x[5])  u.b_reg = dreg(x[2:0]);
              else begin u.b_sel = rd68041_pkg::S_IMM; u.imm[15:8] = {3'h0, x[4:0]}; end
              u.fn = {2'b00, x[11], x[5]};
              u.op = rd68041_pkg::OP_BFPRM;
              u.wr = 1'b1; u.wr_reg = 5'd19;
              if (!rg) begin
                `DEC_EA(ea_s, rd68041_pkg::SZ_L)
                u.wr2 = 1'b1; u.wr2_reg = 5'd16;
              end
            end else if (!rg && (sq == 5'd1 || sq == 5'd2)) begin
              // The bytes the field covers, one to five: a first access into
              // T1, then any last byte into T2 with the parameters beside it.
              // EA sizes both from T3 (bfa).
              u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_REG; u.ea_reg = 5'd16;
              u.ld = 1'b1;
              u.b_sel = rd68041_pkg::S_MEM;
              u.bfa = sq[1:0];
              if (sq == 5'd1) begin
                u.wr = 1'b1; u.wr_reg = 5'd17;
              end else begin
                u.sz = rd68041_pkg::SZ_B;
                u.a_reg = 5'd19;
                u.op = rd68041_pkg::OP_BFPK;
                u.wr = 1'b1; u.wr_reg = 5'd18;
              end
            end else if (rg ? (sq == 5'd1) : (sq == 5'd3)) begin
              // The operation. A: what BFINS inserts, or BFFFO's offset; B:
              // the register or the first four bytes; C: the parameters (and
              // the fifth byte).
              if (t == 3'd5) begin
                if (x[11]) u.a_reg = dreg(x[8:6]);
                else begin u.a_sel = rd68041_pkg::S_IMM; u.imm = {27'h0, x[10:6]}; end
              end else begin
                u.a_reg = dreg(x[14:12]);
              end
              u.b_reg   = rg ? dy : 5'd17;
              u.wr2_reg = rg ? 5'd19 : 5'd18;
              u.op = rd68041_pkg::OP_BF;
              u.fn = {rg, t};
              u.cc = rd68041_pkg::CC_SET; u.cond = 4'hF;       // set by the operation
              if (t == 3'd1 || t == 3'd3 || t == 3'd5) begin
                u.wr = 1'b1; u.wr_reg = dreg(x[14:12]);
              end else if (md) begin
                u.wr = 1'b1; u.wr_reg = rg ? dy : 5'd17;
                if (!rg) begin u.wr2 = 1'b1; end
              end
            end else begin
              // Memory, modified: written back as it was read (bfa).
              u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_REG; u.ea_reg = 5'd16;
              u.st = 1'b1;
              u.bfa = (sq == 5'd4) ? 2'd1 : 2'd2;
              if (sq == 5'd4) begin
                u.b_reg = 5'd17;
              end else begin
                u.sz = rd68041_pkg::SZ_B;
                u.b_reg = 5'd18;
              end
            end
          end else begin
            // A memory word, by one (PRM 4).
            if (!malt(sm, sr)) bad = 1'b1;
            need = ea_len(sm, sr, rd68041_pkg::SZ_W, ew(w, 0)); pop = need;
            u.len = 5'(2 + 2*need);
            u.sz = rd68041_pkg::SZ_W;
            `DEC_SRC(rd68041_pkg::SZ_W, 1'b0)
            u.st = 1'b1;
            u.a_sel = rd68041_pkg::S_IMM; u.imm = 32'd1;
            u.op = rd68041_pkg::OP_SHIFT;
            u.fn = {1'b0, op[8], op[10:9]};
          end
        end else begin
          // A data register, by an immediate 1-8 or by Dx mod 64.
          u.len = 5'd2;
          u.sz = op[7:6];
          u.b_reg = dy;
          if (op[5]) begin
            u.a_reg = dn;
          end else begin
            u.a_sel = rd68041_pkg::S_IMM;
            u.imm = (op[11:9] == 3'd0) ? 32'd8 : {29'h0, op[11:9]};
          end
          u.op = rd68041_pkg::OP_SHIFT;
          u.fn = {1'b0, op[8], op[4:3]};
          u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = u.sz;
        end
      end

      // 1010: the A-line, vector 10 (UM 8.2.4).
      4'b1010: begin
        u.len = 5'd2;
        u.xvec = 8'd10;
      end

      // 1111: the F-line. CINV, CPUSH, MOVE16, PFLUSH ($F500-$F51F) and
      // PTEST ($F548, $F568), privileged; the floating-point instructions
      // (M11; on the MC68LC040 each is vector 11 with a format $4 frame, M12);
      // the rest, a scope of 00 for CINV and CPUSH among it, is vector 11
      // (UM 9.4).
      default: begin
        u.len = 5'd2;
        if (op[11:8] == 4'h4 && op[4:3] != 2'b00) begin
          // CINV and CPUSH (PRM 6): caches in 7-6 (01 data, 10 instruction,
          // 11 both), push in 5, scope in 4-3 (01 a line, 10 a page, 11
          // all), An the line's or page's physical address. Serialising: the
          // exception state machine runs them, and fetching starts again
          // behind (UM 4.5: "the pipeline is flushed, and instruction
          // prefetches are repeated").
          priv = 1'b1;
          u.b_reg = ay;
          u.cmo = 1'b1; u.ser = 1'b1; u.cof = 1'b1;
          u.fn = {op[7:6], op[4:3]};
          u.cond = {3'b000, op[5]};
        end else if (op[11:8] == 4'h5 && (op[7:5] == 3'b000 || op[7:3] == 5'b01001 ||
                                          op[7:3] == 5'b01101)) begin
          // PFLUSH, PTEST (PRM 6; UM 3.7): op[4:3] PFLUSH's opmode (00 N,
          // 01 the page, 10 AN, 11 A), op[5] PTEST's R; An the logical
          // address, DFC the space. Run as CINV is, the MMU's operation
          // in place of the caches' (cond[1]).
          priv = 1'b1;
          u.b_reg = ay;
          u.cmo = 1'b1; u.ser = 1'b1; u.cof = 1'b1;
          u.fn = op[6] ? {2'b01, 1'b0, op[5]} : {2'b00, op[4:3]};
          u.cond = 4'b0010;
        end else if (op[11:5] == 7'b0110000 || op[11:3] == 9'b011000100) begin
          // MOVE16 (PRM 4): a line read of the source into the data memory
          // unit's buffer, a line write of the destination from it, then the
          // address registers that postincrement, by 16 -- after the
          // transfers, so that a fault leaves them as they were.
          logic        two;
          logic [2:0]  ra, rb2;
          logic [31:0] abs;
          logic        s_abs, d_abs, inc_a, inc_b;
          two   = op[5];                           // (Ax)+,(Ay)+
          abs   = {ew(w, 0), ew(w, 1)};
          ra    = op[2:0];
          rb2   = w0[14:12];
          s_abs = !two && op[3];
          d_abs = !two && !op[3];
          inc_a = two || !op[4];                   // Ax, or Ay of opmodes 00, 01
          // Ay of (Ax)+,(Ay)+ -- once only when it is Ax: PRM 4 does not say,
          // the cputest corpus increments it once (doc/oracles.md).
          inc_b = two && (rb2 != ra);
          need  = two ? 4'd1 : 4'd2;
          u.len = two ? 5'd4 : 5'd6;
          more  = (sq < (5'd1 + {4'h0, inc_a} + {4'h0, inc_b}));
          u.last = !more;
          if (!more) pop = need;
          u.sz = rd68041_pkg::SZ_L;
          case (sq)
            5'd0, 5'd1: begin
              u.m16 = 1'b1;
              if (sq == 5'd0) u.ld = 1'b1; else u.st = 1'b1;
              if ((sq == 5'd0) ? s_abs : d_abs) begin
                u.ea_en = 1'b1; u.ea_base = rd68041_pkg::B_NONE; u.ea_disp = abs;
              end else begin
                `DEC_EA(ea_fix(3'd2, (sq == 5'd1 && two) ? rb2 : ra, w0), rd68041_pkg::SZ_L)
              end
            end
            default: begin
              u.a_sel = rd68041_pkg::S_IMM; u.imm = 32'd16;
              u.b_reg = areg((sq == 5'd3 || !inc_a) ? rb2 : ra);
              u.op = rd68041_pkg::OP_ADDA;
              u.wr = 1'b1; u.wr_reg = areg((sq == 5'd3 || !inc_a) ? rb2 : ra);
            end
          endcase
        end else if (op[11:9] == 3'b001) begin
          // The floating-point instructions (cpID 1; PRM 5, 8; doc/fpu.md).
          // c is the command word (types 000 and 001), w[0].
          logic [15:0] c;
          logic  [2:0] ocl, fm, nl, kk, fix;
          logic  [1:0] osz;
          logic  [3:0] nx;
          logic        isdn, isimm, pre, post;
          c    = w0;
          fix  = 3'd0;
          nx   = 4'd0;
          // The command word, at least, before anything is decided from it.
          need = (op[8:7] == 2'b00) ? 4'd1 : 4'd0;
          ocl  = c[15:13];
          fm   = c[12:10];
          isdn = (sm == 3'd0);
          isimm = is_imm(sm, sr);
          pre  = (sm == 3'd4);
          post = (sm == 3'd3);
          u.fcmd = c;
          u.fchk = (sq == 5'd0);
          // The operand's long words, and its size on the bus (PRM 5: L S X P
          // W D B, 0-6).
          nl  = (fm == 3'd5) ? 3'd2 : (fm == 3'd2 || fm == 3'd3 || fm == 3'd7) ? 3'd3 : 3'd1;
          osz = (fm == 3'd6) ? rd68041_pkg::SZ_B : (fm == 3'd4) ? rd68041_pkg::SZ_W :
                rd68041_pkg::SZ_L;
          kk  = sq[2:0];
          case (op[8:6])
            3'b000: begin
              case (ocl)
                3'b000: begin
                  // FPm,FPn: the operation alone.
                  need = 4'd1; pop = 4'd1; u.len = 5'd4;
                  u.fpk = rd68041_fpu_pkg::FK_OP;
                  // An opmode PRM 5 does not have: the FPU's F-line, here
                  // the MC68LC040's (below).
                  if (FPU == 0 && !rd68041_fpu_pkg::legal_op(c[6:0])) fline = 1'b1;
                end
                3'b010: begin
                  if (fm == 3'd7) begin
                    // FMOVECR: unimplemented (UM Table 9-10); with an <ea>
                    // field it is an F-line instruction (the FPSP's x_fline).
                    need = 4'd1; pop = 4'd1; u.len = 5'd4;
                    if (op[5:0] != 6'h0) fline = 1'b1;
                    u.fpk = rd68041_fpu_pkg::FK_OP;
                  end else begin
                    // <ea>,FPn: the operand's long words, the last with the
                    // operation; -(An) the last first.
                    if (sm == 3'd1 || (isdn && nl != 3'd1) || !ok_mode(sm, sr)) fline = 1'b1;
                    if (FPU == 0 && !rd68041_fpu_pkg::legal_op(c[6:0])) fline = 1'b1;
                    nx = isimm ? ((nl == 3'd1) ? ((fm == 3'd6 || fm == 3'd4) ? 4'd1 : 4'd2)
                                               : {nl, 1'b0})
                               : ea_len(sm, sr, osz, ew(w, 1));
                    need  = 4'd1 + nx;
                    u.len = 5'(2 + 2*need);
                    more  = ({2'b00, kk} < 5'(nl - 3'd1));
                    u.last = !more;
                    if (!more) pop = need;
                    fix   = pre ? 3'(nl - 3'd1 - kk) : kk;
                    u.fidx = {2'b00, fix};
                    u.fpk = more ? rd68041_fpu_pkg::FK_IN : rd68041_fpu_pkg::FK_OP;
                    if (isdn) begin
                      u.b_sel = rd68041_pkg::S_REG; u.b_reg = dy;
                    end else if (isimm) begin
                      u.b_sel = rd68041_pkg::S_IMM;
                      u.imm = (nl != 3'd1) ? ewl(w, 1 + 2*int'(fix)) :
                              (fm == 3'd6) ? imm_sb : (fm == 3'd4) ? imm_sw : imm_sl;
                    end else begin
                      u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM;
                      u.sz = (nl == 3'd1) ? osz : rd68041_pkg::SZ_L;
                      `DEC_EA(ea_s, u.sz)
                      if (!(pre || post)) u.ea_disp = u.ea_disp + {27'h0, fix, 2'b00};
                    end
                  end
                end
                3'b011: begin
                  // FMOVE FPm,<ea>: the conversion, then its long words.
                  if (sm == 3'd1 || (isdn && nl != 3'd1) || (!isdn && !dalt(sm, sr)))
                    fline = 1'b1;
                  nx    = isdn ? 4'd0 : ea_len(sm, sr, osz, ew(w, 1));
                  need  = 4'd1 + nx;
                  u.len = 5'(2 + 2*need);
                  more  = ({2'b00, kk} < {2'b00, nl});
                  u.last = !more;
                  if (!more) pop = need;
                  if (sq == 5'd0) begin
                    u.fpk = rd68041_fpu_pkg::FK_OP;
                    // The destination's address, for the frame of an
                    // exception taken here (an unsupported data type, UM
                    // 9.6.2): computed, not accessed, and An left alone --
                    // but for packed, which always takes it (Table 9-10):
                    // (An)+ and -(An) step over its twelve bytes first, as
                    // UAE040 does, and the frame has -(An)'s new An.
                    if (!isdn) begin
                      `DEC_EA(ea_s, rd68041_pkg::SZ_L)
                      if (fm == 3'd3 || fm == 3'd7) u.ea_step = 4'd12;
                      else u.ea_upd = rd68041_pkg::U_NONE;
                    end
                  end else begin
                    fix = pre ? 3'(nl - kk) : 3'(kk - 3'd1);
                    u.fidx = {2'b00, fix};
                    u.fpk = rd68041_fpu_pkg::FK_OUT;
                    if (isdn) begin
                      u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = osz;
                    end else begin
                      u.st = 1'b1;
                      u.sz = (nl == 3'd1) ? osz : rd68041_pkg::SZ_L;
                      `DEC_EA(ea_s, u.sz)
                      if (!(pre || post)) u.ea_disp = u.ea_disp + {27'h0, fix, 2'b00};
                    end
                  end
                end
                3'b100, 3'b101: begin
                  // FMOVE(M) of FPCR, FPSR, FPIAR (PRM 5): in that order in
                  // memory, one a micro-operation; an empty list is FPIAR.
                  // (An)+ and -(An) both ways, as PRM 5's tables have them
                  // (its text has neither to memory nor from memory,
                  // doc/manual-contradictions.md).
                  logic [2:0] lst, nr, r0, r1, r2, rk;
                  logic       tom;
                  rk  = 3'd0;
                  tom = ocl[0];
                  lst = (fm == 3'd0) ? 3'b001 : fm;
                  nr  = {2'b00, lst[2]} + {2'b00, lst[1]} + {2'b00, lst[0]};
                  // The k-th selected register, in memory order: FPCR 2,
                  // FPSR 1, FPIAR 0.
                  r0 = lst[2] ? 3'd2 : lst[1] ? 3'd1 : 3'd0;
                  r1 = (lst[2] && lst[1]) ? 3'd1 : 3'd0;
                  r2 = 3'd0;
                  if ((isdn && nr != 3'd1) || (sm == 3'd1 && lst != 3'b001) ||
                      (isimm && tom) || (tom && !(isdn || sm == 3'd1) && !malt(sm, sr)) ||
                      (!tom && !ok_mode(sm, sr)))
                    fline = 1'b1;
                  nx    = isimm ? {nr, 1'b0} : ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1));
                  need  = 4'd1 + nx;
                  u.len = 5'(2 + 2*need);
                  more  = ({2'b00, kk} < 5'(nr - 3'd1));
                  u.last = !more;
                  if (!more) pop = need;
                  fix = pre ? 3'(nr - 3'd1 - kk) : kk;
                  rk  = (fix == 3'd0) ? r0 : (fix == 3'd1) ? r1 : r2;
                  u.fidx = {2'b00, rk};
                  u.fpk = tom ? rd68041_fpu_pkg::FK_CRR : rd68041_fpu_pkg::FK_CRW;
                  if (isdn || sm == 3'd1) begin
                    if (tom) begin u.wr = 1'b1; u.wr_reg = {1'b0, sm[0], sr}; end
                    else begin u.b_sel = rd68041_pkg::S_REG; u.b_reg = {1'b0, sm[0], sr}; end
                  end else if (isimm) begin
                    u.b_sel = rd68041_pkg::S_IMM; u.imm = ewl(w, 1 + 2*int'(fix));
                  end else begin
                    if (tom) u.st = 1'b1;
                    else begin u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM; end
                    `DEC_EA(ea_s, rd68041_pkg::SZ_L)
                    if (!(pre || post)) u.ea_disp = u.ea_disp + {27'h0, fix, 2'b00};
                  end
                end
                3'b001: fline = 1'b1;                    // no such opclass
                default: begin
                  // FMOVEM of FP0-FP7 (PRM 5): three long words a register,
                  // the decode stage counting the list (mv_*); a dynamic list
                  // is read from Dn once the pipeline behind has emptied.
                  logic tom, pm, dyn;
                  logic [4:0] sub;
                  tom = ocl[0];
                  pm  = !c[12];                          // predecrement
                  dyn = c[11];
                  if (tom ? !(pm ? pre : (ctrl(sm, sr) && malt(sm, sr)))
                          : (pm || !(ctrl(sm, sr) || post)))
                    fline = 1'b1;
                  need  = 4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_L, ew(w, 1));
                  u.len = 5'(2 + 2*need);
                  if (dyn && sq == 5'd0) qwait = 1'b1;
                  sub = sq - {k[2:0], 1'b0} - {2'b00, k[2:0]};
                  if (sq == 5'd0 && mv_none) begin
                    // No registers: a pending exception is still taken.
                    u.fpk = rd68041_fpu_pkg::FK_COND;
                    u.fcmd = 16'h0;
                    pop = need;
                  end else begin
                    mv_used = (sub == 5'd2);
                    more    = !(mv_last && sub == 5'd2);
                    u.last  = !more;
                    if (!more) pop = need;
                    fix = pm ? 3'(3'd2 - sub[2:0]) : sub[2:0];
                    u.fidx = {2'b00, fix};
                    u.freg = mv_reg[2:0];
                    u.fpk = tom ? rd68041_fpu_pkg::FK_RGR : rd68041_fpu_pkg::FK_RGW;
                    if (tom) u.st = 1'b1;
                    else begin u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM; end
                    `DEC_EA(ea_s, rd68041_pkg::SZ_L)
                    if (!(pre || post))
                      u.ea_disp = u.ea_disp + {25'h0, k[2:0], 3'b000} + {26'h0, k[2:0], 2'b00} +
                                  {27'h0, fix, 2'b00};
                  end
                end
              endcase
            end
            3'b001: begin
              // FScc, FDBcc, FTRAPcc (PRM 5): the predicate in c[5:0].
              u.fpk = rd68041_fpu_pkg::FK_COND;
              u.fcmd = {10'h0, c[5:0]};
              if (sm == 3'd1) begin
                need = 4'd2; pop = 4'd2; u.len = 5'd6;
                u.b_reg = dy;
                u.op = rd68041_pkg::OP_DBCC;
                u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_W;
                u.br = rd68041_pkg::BR_DISP;
                // PRM 5, FDBcc: from the displacement word's address.
                u.tgt = pc + 32'd4 + ews(w, 1);
              end else if (sm == 3'd7 && sr >= 3'd2) begin
                if (sr > 3'd4) fline = 1'b1;
                need = 4'd1 + ((sr == 3'd2) ? 4'd1 : (sr == 3'd3) ? 4'd2 : 4'd0);
                pop  = need;
                u.len = 5'(2 + 2*need);
                u.trap = 1'b1;
              end else begin
                if (!dalt(sm, sr)) fline = 1'b1;
                need = 4'd1 + ea_len(sm, sr, rd68041_pkg::SZ_B, ew(w, 1)); pop = need;
                u.len = 5'(2 + 2*need);
                u.sz = rd68041_pkg::SZ_B;
                u.op = rd68041_pkg::OP_SCC;
                if (isdn) begin u.wr = 1'b1; u.wr_reg = dy; u.wr_sz = rd68041_pkg::SZ_B; end
                else begin `DEC_EA(ea_s, rd68041_pkg::SZ_B) u.st = 1'b1; end
              end
            end
            3'b010, 3'b011: begin
              // FBcc (PRM 5), FNOP among them: the predicate in op[5:0].
              logic [31:0] d;
              need = op[6] ? 4'd2 : 4'd1;
              d    = op[6] ? {ew(w, 0), ew(w, 1)} : ews(w, 0);
              pop  = need;
              u.len = 5'(2 + 2*need);
              u.fpk = rd68041_fpu_pkg::FK_COND;
              u.fcmd = {10'h0, op[5:0]};
              u.tgt = pc + 32'd2 + d;
              u.br  = rd68041_pkg::BR_DISP;
            end
            3'b100: begin
              // FSAVE <ea> (UM 9.8), privileged: the frame's long words, once
              // the FPU's state is settled.
              priv = 1'b1;
              u.fchk = 1'b0;
              if (!((ctrl(sm, sr) && malt(sm, sr)) || pre)) fline = 1'b1;
              if (sq == 5'd0) qwait = 1'b1;
              need  = ea_len(sm, sr, rd68041_pkg::SZ_L, w0);
              u.len = 5'(2 + 2*need);
              more  = (sq < (fp_svn - 5'd1));
              u.last = !more;
              if (!more) pop = need;
              fix = 3'd0;
              u.fidx = pre ? 5'(fp_svn - 5'd1 - sq) : sq;
              u.fpk = rd68041_fpu_pkg::FK_SAVE;
              u.st = 1'b1;
              `DEC_EA(ea_s, rd68041_pkg::SZ_L)
              if (!pre) u.ea_disp = u.ea_disp + {25'h0, u.fidx, 2'b00};
            end
            3'b101: begin
              // FRESTORE <ea> (UM 9.8), privileged: its first long word,
              // then, once that is in, the rest the frame has, then its
              // state taken; (An)+ moves An past it last.
              priv = 1'b1;
              u.fchk = 1'b0;
              if (!(ctrl(sm, sr) || post)) fline = 1'b1;
              if (sq == 5'd1) qwait = 1'b1;
              need  = ea_len(sm, sr, rd68041_pkg::SZ_L, w0);
              u.len = 5'(2 + 2*need);
              more  = (sq == 5'd0) || (sq < fp_rsn);
              u.last = !more;
              if (!more) pop = need;
              fix = 3'd0;
              if (sq == 5'd0 || sq < fp_rsn) begin
                u.fidx = sq;
                u.fpk = rd68041_fpu_pkg::FK_RST;
                u.ld = 1'b1; u.b_sel = rd68041_pkg::S_MEM;
                if (post) begin
                  `DEC_EA(ea_fix(3'd2, sr, w0), rd68041_pkg::SZ_L)
                end else begin
                  `DEC_EA(ea_s, rd68041_pkg::SZ_L)
                end
                u.ea_disp = u.ea_disp + {25'h0, sq, 2'b00};
              end else begin
                u.fpk = rd68041_fpu_pkg::FK_REND;
                if (post) begin
                  u.a_sel = rd68041_pkg::S_IMM; u.imm = {25'h0, fp_rsn, 2'b00};
                  u.b_reg = ay;
                  u.op = rd68041_pkg::OP_ADDA;
                  u.wr = 1'b1; u.wr_reg = ay;
                end
              end
            end
            default: fline = 1'b1;
          endcase
          // The MC68LC040 (UM A.5.1): a legal FP instruction is decoded for
          // its length and its <ea> only, which is calculated and neither
          // accessed nor stepped, and it is vector 11 once it is done (the
          // PC the next instruction's), with a format $4 frame holding the
          // address and the instruction's own PC. The address is the
          // operand's, -(An)'s An less the operand's size; FMOVEM's, the
          // control registers', FSAVE's and FRESTORE's An itself; 0 for a
          // register or an immediate (UM A.5.1) and for those with no <ea>.
          // As UAE040 -L has them (doc/oracles.md): A.5.1 says only that it
          // is the operand's, "using the addressing mode".
          if (FPU == 0) begin
            logic        f4ea;
            logic  [3:0] f4sz;
            f4ea = 1'b0;
            f4sz = 4'd0;
            case (op[8:6])
              3'b000: begin
                f4ea = (ocl != 3'b000) && !(ocl == 3'b010 && fm == 3'd7);
                if (ocl == 3'b010 || ocl == 3'b011)
                  f4sz = (fm == 3'd2 || fm == 3'd3 || fm == 3'd7) ? 4'd12 :
                         (fm == 3'd5) ? 4'd8 : (fm == 3'd4) ? 4'd2 :
                         (fm == 3'd6) ? ((sr == 3'd7) ? 4'd2 : 4'd1) : 4'd4;
              end
              3'b001: begin
                f4ea = !(sm == 3'd1 || (sm == 3'd7 && sr >= 3'd2));
                f4sz = (sr == 3'd7) ? 4'd2 : 4'd1;
              end
              3'b100, 3'b101: f4ea = 1'b1;
              default: ;
            endcase
            u = '0;
            u.valid = 1'b1;
            u.last  = 1'b1;
            u.pc    = pc;
            u.len   = 5'(2 + 2*need);
            u.xvec  = 8'd11;
            u.xnext = 1'b1;
            if (f4ea && sm >= 3'd2 && !isimm) begin
              `DEC_EA(ea_s, rd68041_pkg::SZ_L)
              u.ea_upd = rd68041_pkg::U_NONE;
              if (pre) u.ea_disp = -{28'h0, f4sz};
              else if (post) u.ea_disp = 32'h0;
            end
            pop     = need;
            more    = 1'b0;
            qwait   = 1'b0;
            mv_used = 1'b0;
          end
        end else begin
          u.xvec = 8'd11;
        end
      end
    endcase

    // The indirection prelude of an instruction whose <ea> is op[5:0]: its
    // first micro-operation reads the pointer into T3, and the instruction's
    // own follow (doc/microarch.md). It takes no extension word: the second
    // address needs them. It is only the read: not the FP operation the
    // instruction's first would be, nor the MC68LC040's exception, which
    // the instruction's own micro-operation takes, on the second address.
    if (uses_ea && seq == 5'd0 && sm >= 3'd2 && is_ind(sm, sr, ew(w, int'(eak)))) begin
      `DEC_IND(ea_sp)
      ind_out = 2'b01;
      pop  = 4'd0;
      more = 1'b1;
      mv_used = 1'b0;
    end

    // After the first micro-operation the instruction's extension words may
    // be gone from the queue, and with them what its length was computed from.
    if (seq != 5'd0) u.len = len_in;

    // A privileged instruction in user mode is a privilege violation, which
    // comes before anything else wrong with it; then what is not built
    // (halt); then an illegal encoding. `need` is kept: whether an
    // instruction is legal can depend on its extension words (MOVEC's
    // register field, the full format's reserved encodings), so it is decided
    // only once they are all in the queue.
    if ((priv && !sup) || u.halt || bad || fline) begin
      logic h;
      h = u.halt && !(priv && !sup);
      u = '0;
      u.valid = 1'b1;
      u.last  = 1'b1;
      u.pc    = pc;
      u.len   = (seq != 5'd0) ? len_in : 5'd2;
      u.halt  = h;
      u.xvec  = h ? 8'd0 : (priv && !sup) ? 8'd8 : fline ? 8'd11 : 8'd4;
      qwait   = 1'b0;
      pop     =  4'd0;
      more    = 1'b0;
      mv_used = 1'b0;
    end
  end

  assign uop = u;

endmodule
