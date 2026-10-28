// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The floating-point unit (UM 9; doc/fpu.md): FP0-FP7, FPCR, FPSR, FPIAR, the
// exception state and its frame, and the data path (rd68041_fpu_dp), driven by
// the FP micro-operations in the core's execute stage.
//
// An FP micro-operation in EX is evaluated there -- at once, or when the data
// path answers -- into what it returns (a result long word, a predicate, an
// exception for the micro-operation to take) and a commit record: the FP
// register, FPCR, FPSR, FPIAR and state it changes. The record is applied when
// the micro-operation leaves write-back (w_commit), dropped if it is flushed
// first; an FP micro-operation waits in EX while a record is outstanding, so
// every one sees committed state. Operand and result staging, and the frame
// image's fields, are written in EX: no live frame can be overwritten there,
// since an FP instruction finding an exception pending takes it before it
// computes anything (doc/fpu.md, "Commit at write-back").

module rd68041_fpu
(
    input  logic         clk,
    input  logic         rst_n,
    input  logic         crst,

    // --- execute --------------------------------------------------------------
    input  logic         x_v,        // an FP micro-operation is in EX, to execute
    input  logic         x_adv,      // ... it goes on to W this clock
    input  logic  [3:0]  x_k,        // FK_*
    input  logic         x_chk,      // the instruction's first: a pending exception is taken here
    input  logic         x_last,     // the instruction's last (FMOVE out: the post exception)
    input  logic [15:0]  x_cmd,      // the command word, or the predicate in 5-0
    input  logic  [4:0]  x_idx,      // the long word
    input  logic  [2:0]  x_reg,      // FMOVEM's register
    input  logic [31:0]  x_b,        // the operand long word
    input  logic [31:0]  x_pc,       // the instruction's address
    input  logic [31:0]  x_ea,       // its effective address (format $2 and $3 frames)
    input  logic         x_eam,      // ... which is a memory one (else 0 is stacked)
    output logic         x_wait,     // EX waits
    output logic [31:0]  x_res,
    output logic         x_ct,       // FK_COND: the predicate holds
    output logic  [7:0]  x_xv,       // an exception this micro-operation takes, 0 none
    output logic         x_xn,       // ... after it (else before)
    output logic  [3:0]  x_xf,       // ... its frame's format
    output logic [31:0]  x_xea,      // ... the frame's address field
    output logic         x_xc,       // ... and it commits all the same (BSUN)

    // --- write-back -------------------------------------------------------------
    input  logic         w_commit,   // the micro-operation with the record completes
    input  logic         flush,      // everything not yet committed goes

    // --- decode ---------------------------------------------------------------
    output logic  [4:0]  sv_n,       // FSAVE: the frame's long words
    output logic  [4:0]  rs_n        // FRESTORE: the frame being restored's (from its first)
);

  // ===========================================================================
  // State
  // ===========================================================================
  localparam logic [79:0] NAN_X = {1'b0, 15'h7FFF, 64'hFFFF_FFFF_FFFF_FFFF};
  logic [79:0] fpr [0:7];
  logic [15:0] fpcr;             // ENABLE 15-8, MODE 7-4 (UM 9.2.2)
  logic [31:0] fpsr;             // CC 27-24, QUOTIENT 23-16, EXC 15-8, AEXC 7-3
  logic [31:0] fpiar;
  logic  [1:0] st;               // ST_*
  logic        pend;             // an exception is pending (the frame's E1 or E3)
  logic [31:0] fr [0:24];        // the busy frame's image, long words 1-24 (UM Figure 9-10;
                                 // 0, the format, is FSAVE's own)

  // Staging.
  logic [31:0] s_in [0:2];       // an operand's long words
  logic [31:0] s_out [0:2];      // FMOVE out's result
  logic [31:0] rs_fmt;           // FRESTORE's format long word
  logic  [7:0] p_xv;             // FMOVE out's post-instruction exception, for its last
  logic        p_pend;           // ... and the state it leaves

  // The commit record.
  logic        rec_v;
  logic        rec_w;            // ... its micro-operation has gone on to W
  logic        rc_fpr_we, rc_fpsr_we, rc_fpcr_we, rc_fpiar_we, rc_st_we, rc_null;
  logic  [2:0] rc_fpr_n;
  logic [79:0] rc_fpr_v;
  logic [31:0] rc_fpsr_v, rc_fpiar_v;
  logic [15:0] rc_fpcr_v;
  logic  [1:0] rc_st_v;
  logic        rc_pend_v;
  logic        rc_idle;          // an operation: a frame held, nothing pending, goes

  wire  [1:0]  prec  = fpcr[7:6];
  wire  [1:0]  rmode = fpcr[5:4];
  wire  [7:0]  en    = fpcr[15:8];

  // ===========================================================================
  // The command word (PRM 5; doc/fpu.md, "What is in hardware")
  // ===========================================================================
  // FRESTORE's resumed operation (FK_REND) runs the frame's CMDREG1B.
  wire        rsm   = (x_k == rd68041_fpu_pkg::FK_REND);
  wire [15:0] cmdw  = rsm ? fr[16][31:16] : x_cmd;
  wire [2:0] opcl  = cmdw[15:13];
  wire [2:0] srcsp = cmdw[12:10];          // a register, or a memory format
  wire [2:0] dstr  = cmdw[9:7];
  wire [6:0] opm   = cmdw[6:0];
  // Memory formats: 0 L, 1 S, 2 X, 3 P, 4 W, 5 D, 6 B, 7 P (dynamic k).
  localparam logic [2:0] M_L = 3'd0, M_S = 3'd1, M_X = 3'd2, M_P = 3'd3, M_W = 3'd4,
                         M_D = 3'd5, M_B = 3'd6, M_PK = 3'd7;

  // The opmodes built (fsel: the data path's operation; frc: the precision
  // forced, 0 FPCR's, 1 single, 2 double; mono: one operand).
  logic       impl, mono, cuo, e3op, is_cmp, is_tst, is_abs, is_neg;
  logic [3:0] fsel;
  logic [1:0] frc;
  always_comb begin
    impl = 1'b1; mono = 1'b0; cuo = 1'b0; e3op = 1'b0; is_cmp = 1'b0; is_tst = 1'b0;
    is_abs = 1'b0; is_neg = 1'b0; fsel = rd68041_fpu_pkg::D_MOVE; frc = 2'd0;
    if (opm[6]) frc = opm[2] ? 2'd2 : 2'd1;
    case (opm)
      7'h00, 7'h40, 7'h44: begin fsel = rd68041_fpu_pkg::D_MOVE; mono = 1'b1; cuo = 1'b1; end
      7'h04, 7'h41, 7'h45: begin fsel = rd68041_fpu_pkg::D_SQRT; mono = 1'b1; e3op = 1'b1; end
      7'h18, 7'h58, 7'h5C: begin fsel = rd68041_fpu_pkg::D_MOVE; mono = 1'b1; cuo = 1'b1; is_abs = 1'b1; end
      7'h1A, 7'h5A, 7'h5E: begin fsel = rd68041_fpu_pkg::D_MOVE; mono = 1'b1; cuo = 1'b1; is_neg = 1'b1; end
      7'h20, 7'h60, 7'h64: begin fsel = rd68041_fpu_pkg::D_DIV; e3op = 1'b1; end
      7'h22, 7'h62, 7'h66: begin fsel = rd68041_fpu_pkg::D_ADD; e3op = 1'b1; end
      7'h23, 7'h63, 7'h67: begin fsel = rd68041_fpu_pkg::D_MUL; e3op = 1'b1; end
      7'h28, 7'h68, 7'h6C: begin fsel = rd68041_fpu_pkg::D_SUB; e3op = 1'b1; end
      7'h24:               begin fsel = rd68041_fpu_pkg::D_SDIV; e3op = 1'b1; end
      7'h27:               begin fsel = rd68041_fpu_pkg::D_SMUL; e3op = 1'b1; end
      7'h38:               begin fsel = rd68041_fpu_pkg::D_CMP; is_cmp = 1'b1; end
      7'h3A:               begin fsel = rd68041_fpu_pkg::D_MOVE; mono = 1'b1; is_tst = 1'b1; end
      default:             impl = 1'b0;
    endcase
  end
  wire is_mvcr = (opcl == 3'b010) && (srcsp == 3'b111);

  // ===========================================================================
  // Operands
  // ===========================================================================
  // The source as it came: from the staging and the last long word (B).
  wire [31:0] ln0 = (x_idx[1:0] == 2'd0) ? x_b : s_in[0];
  wire [31:0] ln1 = (x_idx[1:0] == 2'd1) ? x_b : s_in[1];
  wire [31:0] ln2 = (x_idx[1:0] == 2'd2) ? x_b : s_in[2];
  wire [95:0] mem96 = {ln0, ln1, ln2};
  rd68041_fpu_pkg::fpn_t src_m;                   // a memory or register source, unpacked
  // Each format unpacked by a continuous assignment, then chosen: with two
  // processors in one simulation (sim/tb/core_smp_tb), iverilog 12 woke the
  // two instances' always_comb on the package functions' variables forever
  // (doc/coding-standard.md, "iverilog 12").
  rd68041_fpu_pkg::fpn_t src_l, src_s, src_w, src_b, src_d, src_x;
  assign src_l = rd68041_fpu_pkg::unpack_i(ln0);
  assign src_s = rd68041_fpu_pkg::unpack_s(ln0);
  assign src_w = rd68041_fpu_pkg::unpack_i({{16{ln0[15]}}, ln0[15:0]});
  assign src_b = rd68041_fpu_pkg::unpack_i({{24{ln0[7]}}, ln0[7:0]});
  assign src_d = rd68041_fpu_pkg::unpack_d({ln0, ln1});
  assign src_x = rd68041_fpu_pkg::unpack_x({mem96[95:80], mem96[63:0]});   // X (P is unsupported)
  always_comb begin
    case (srcsp)
      M_L:     src_m = src_l;
      M_S:     src_m = src_s;
      M_W:     src_m = src_w;
      M_B:     src_m = src_b;
      M_D:     src_m = src_d;
      default: src_m = src_x;
    endcase
  end
  // A resumed operation's operands: ETEMP and FPTEMP, their exponents 16
  // bits with ETE15 and FPTE15 (the FPSP normalises a denormal so, res_func.S).
  function automatic rd68041_fpu_pkg::fpn_t unpack_t(input logic [15:0] ex, input logic e15,
                                    input logic [63:0] m);
    if (e15 && m[63]) unpack_t = rd68041_fpu_pkg::fp_mk(rd68041_fpu_pkg::C_NORM, ex[15],
                                                        {3'b111, ex[14:0]}, m);
    else              unpack_t = rd68041_fpu_pkg::unpack_x({ex, m});
  endfunction
  wire fp_src  = (opcl == 3'b000);
  rd68041_fpu_pkg::fpn_t srcn, dstn;
  assign srcn = rsm ? unpack_t(fr[22][31:16], fr[15][28], {fr[23], fr[24]}) :
                fp_src ? rd68041_fpu_pkg::unpack_x(fpr[srcsp]) : src_m;
  assign dstn = (rsm && !mono) ? unpack_t(fr[19][31:16], fr[17][28], {fr[20], fr[21]})
                               : rd68041_fpu_pkg::unpack_x(fpr[dstr]);
  wire  pk_src = !rsm && !fp_src && (srcsp == M_P || srcsp == M_PK);

  function automatic logic [2:0] tag_of(input rd68041_fpu_pkg::fpn_t x);
    tag_of = x.cls;
  endfunction
  function automatic logic [3:0] cc_of(input rd68041_fpu_pkg::fpn_t x);
    // N Z I NAN (UM Table 9-7).
    cc_of = {x.sign, x.cls == rd68041_fpu_pkg::C_ZERO, x.cls == rd68041_fpu_pkg::C_INF, x.cls == rd68041_fpu_pkg::C_NAN};
  endfunction

  // ===========================================================================
  // The pending exception's vector (UM 9.7; doc/fpu.md, "Which vector")
  // ===========================================================================
  // The frame's NMCEXC bits: snan 3, operr 2, ovfl 1, unfl 0; NMNEXC unsup 1.
  wire [7:0] nmc  = fr[14][23:16];
  wire [7:0] exen = fpsr[15:8] & en;
  logic [7:0] pv;
  always_comb begin
    pv = 8'd0;
    if (exen[7])                          pv = 8'd48;   // BSUN
    else if (exen[6] || nmc[3])           pv = 8'd54;   // SNAN
    else if (exen[5] || nmc[2])           pv = 8'd52;   // OPERR
    else if (exen[4] || nmc[1])           pv = 8'd53;   // OVFL
    else if (exen[3] || nmc[0])           pv = 8'd51;   // UNFL
    else if (exen[2])                     pv = 8'd50;   // DZ
    else if (exen[1] || exen[0])          pv = 8'd49;   // INEX2, INEX1
    else if (nmc[4])                      pv = 8'd55;   // unsupported
    // UM 9.7.7's note: an overflow with OVFL disabled, INEX2 enabled.
    else if (fpsr[12] && en[1])           pv = 8'd49;
  end

  // ===========================================================================
  // The data path
  // ===========================================================================
  logic        dp_start, dp_busy, dp_done;
  logic  [3:0] dp_op;
  rd68041_fpu_pkg::fpn_t        dp_a, dp_b, dp_r;
  logic  [1:0] dp_prec, dp_isz;
  logic        d_snan, d_operr, d_ovfl, d_unfl, d_dz, d_inex, d_spec, d_dnz, d_lt, d_eq, d_un;
  logic [17:0] d_iexp;
  logic [66:0] d_iman;
  logic [31:0] d_int;
  rd68041_fpu_dp u_dp (
      .clk(clk), .rst_n(rst_n), .crst(crst), .kill(flush), .start_i(dp_start && !flush),
      .op_i(dp_op), .a_i(dp_a),
      .b_i(dp_b), .prec_i(dp_prec), .rmode_i(rmode), .isz_i(dp_isz), .busy(dp_busy), .done(dp_done),
      .r(dp_r), .f_snan(d_snan), .f_operr(d_operr), .f_ovfl(d_ovfl), .f_unfl(d_unfl),
      .f_dz(d_dz), .f_inex(d_inex), .f_spec(d_spec), .f_dnz(d_dnz), .i_exp(d_iexp), .i_man(d_iman),
      .r_int(d_int), .c_lt(d_lt), .c_eq(d_eq), .c_un(d_un));

  // ===========================================================================
  // Execute
  // ===========================================================================
  // ph: 0 not started; 1 the data path runs; 2 done, its outputs held.
  logic [1:0] ph;
  // The micro-operation may start: no record is outstanding but its own.
  wire  can   = x_v && !rec_v && (ph == 2'd0);
  // The pending exception, taken here: nothing else happens.
  wire  takep = x_chk && pend && (pv != 8'd0);

  // FK_OP's decisions at its start.
  wire  op_gen   = (opcl == 3'b000) || (opcl == 3'b010);   // to a register
  wire  op_out   = (opcl == 3'b011);                       // FMOVE out
  wire  op_unim  = op_gen && (is_mvcr || (!impl && rd68041_fpu_pkg::unimp_op(opm)));
  wire  op_ill   = op_gen && !is_mvcr && !rd68041_fpu_pkg::legal_op(opm);
  // Unsupported data types (UM 9.6.2): a denormal or unnormal source, or a
  // dyadic one's destination; a packed source; FMOVE out of one, or to packed.
  wire  src_dn   = (srcn.cls == rd68041_fpu_pkg::C_DNRMX) || (srcn.cls == rd68041_fpu_pkg::C_DNRMS);
  wire  dst_dn   = !mono && (dstn.cls == rd68041_fpu_pkg::C_DNRMX);
  wire  op_uns   = !rsm && op_gen && impl && !is_mvcr && (pk_src || src_dn || dst_dn);
  wire  [2:0] ofmt = srcsp;                                // FMOVE out's format
  rd68041_fpu_pkg::fpn_t outn;
  assign outn = rd68041_fpu_pkg::unpack_x(fpr[dstr]);                       // FMOVE out's source: FPm in 9-7
  wire  out_uns  = !rsm && op_out && ((outn.cls == rd68041_fpu_pkg::C_DNRMX) || ofmt == M_P || ofmt == M_PK);
  // FMOVE.X out does not round; nor do FTST and FCMP.
  wire  out_x    = op_out && (ofmt == M_X);
  wire  out_int  = op_out && (ofmt == M_L || ofmt == M_W || ofmt == M_B);
  wire  no_dp    = out_x || is_tst;

  // The operation's precision: FS/FD force one, the destination format for
  // FMOVE out, else FPCR's. PREC = 11 is "Undefined" (UM Table 9-1): double,
  // as UAE040 rounds it (doc/divergences.md).
  logic [1:0] op_prec;
  always_comb begin
    if (op_out)          op_prec = (ofmt == M_S) ? rd68041_fpu_pkg::P_S : (ofmt == M_D) ? rd68041_fpu_pkg::P_D : rd68041_fpu_pkg::P_X;
    else if (frc == 2'd1) op_prec = rd68041_fpu_pkg::P_S;
    else if (frc == 2'd2) op_prec = rd68041_fpu_pkg::P_D;
    else                 op_prec = (prec == 2'd3) ? rd68041_fpu_pkg::P_D : prec;
  end
  always_comb begin
    dp_op = op_out ? (out_int ? rd68041_fpu_pkg::D_INT : rd68041_fpu_pkg::D_MOVE) : fsel;
    dp_a  = dstn;
    dp_b  = op_out ? outn : srcn;
    if (!op_out && (is_abs || is_neg) && srcn.cls != rd68041_fpu_pkg::C_NAN)
      dp_b.sign = is_abs ? 1'b0 : !srcn.sign;
    dp_prec = op_prec;
    dp_isz  = (ofmt == M_B) ? 2'd0 : (ofmt == M_W) ? 2'd1 : 2'd2;
  end
  // The FRESTOREd frame's resumed operation (doc/fpu.md, "The state frames"):
  // CU_SAVEPC $FE, E1 and E3 clear.
  wire        rs_busy  = (rs_fmt[31:16] == 16'h4160);
  wire        rs_run   = rs_busy && (fr[2][31:24] == 8'hFE) && !fr[18][26] && !fr[18][25];
  wire        rs_go    = rs_run && op_gen && impl;
  assign dp_start = can && !takep &&
                    (((x_k == rd68041_fpu_pkg::FK_OP) && !op_unim && !op_ill && !op_uns && !out_uns && !no_dp) ||
                     (rsm && rs_go));
  // The address FPIAR takes: the instruction's, or a resumed one's own.
  wire [31:0] i_pc = rsm ? fpiar : x_pc;

  // --- FK_OP's outcome, once the data path (or no data path) has answered ----
  // The exception byte (UM 9.2.3.3), the accrued one (9.2.3.4), and whether a
  // trap is posted or taken.
  logic  [7:0] o_exc;
  logic  [4:0] o_aexc;
  logic [79:0] o_res;            // the register's new value
  logic        o_wr;             // ... written
  logic  [3:0] o_cc;
  logic        o_post;           // posted: pending at commit (opclass 000/010)
  logic  [7:0] o_xv;             // taken now (FMOVE out: by its last)
  logic        o_e3;
  logic        o_e3now;          // an overflow or underflow to a register: taken at once
  logic [95:0] o_out;            // FMOVE out's long words
  rd68041_fpu_pkg::fpn_t        o_rn;
  always_comb begin
    logic snan, operr, ovfl, unfl, dz, inex, ovinf;
    logic [7:0] ex;
    o_rn = dp_r;
    snan = d_snan; operr = d_operr; ovfl = d_ovfl; unfl = d_unfl; dz = d_dz; inex = d_inex;
    if (no_dp) begin
      // FMOVE.X out: the register as it is, an SNAN quieted (UM 9.7.2.1); FTST.
      o_rn = op_out ? outn : srcn;
      snan = (o_rn.cls == rd68041_fpu_pkg::C_NAN) && o_rn.snan;
      operr = 1'b0; ovfl = 1'b0; unfl = 1'b0; dz = 1'b0; inex = 1'b0;
      if (o_rn.cls == rd68041_fpu_pkg::C_NAN) o_rn.man[62] = 1'b1;
    end
    ex = {1'b0, snan, operr, ovfl, unfl, dz, inex, 1'b0};
    o_exc  = ex;
    o_aexc = {snan | operr, ovfl, unfl & inex, dz, inex | ovfl};
    o_e3   = e3op && !op_out && (ovfl || unfl || inex) && !(snan || operr || dz);
    // Every overflow and underflow to a register is taken at once: E3's, and
    // FMOVE's, FABS's and FNEG's in E1 (the frame says which).
    o_e3now = op_gen && (ovfl || unfl) && !(snan || operr || dz);
    o_res  = rd68041_fpu_pkg::pack_x(o_rn);
    o_cc   = cc_of(o_rn);
    if (is_cmp) begin
      // FCMP (PRM 5's table): N if less, Z if equal; N also for equal
      // negative zeros and infinities; a NaN is unordered, its sign N.
      o_cc = d_un ? {o_rn.sign, 1'b0, 1'b0, 1'b1}
                  : {d_lt || (d_eq && dstn.sign && dstn.cls != rd68041_fpu_pkg::C_NORM), d_eq, 1'b0, 1'b0};
    end
    if (is_tst) o_cc = cc_of(srcn);
    // An overflow or underflow that traps leaves the destination alone; the
    // condition codes are those of the result the FPSP will store, as UAE040
    // sets them: Table 9-12's for an overflow, and for an underflow the
    // denormalised result (UM 9.7.5.2), which is Table 9-13's zero only
    // when nothing of it is left.
    if (ovfl && op_gen)
      o_cc = {o_rn.sign, 1'b0,
              (rmode == rd68041_fpu_pkg::R_N) || (rmode == rd68041_fpu_pkg::R_M && o_rn.sign) ||
              (rmode == rd68041_fpu_pkg::R_P && !o_rn.sign), 1'b0};
    else if (unfl && op_gen)
      o_cc = {o_rn.sign, d_dnz, 2'b00};
    // A register destination is written unless a trap comes before it: the
    // nonmaskable overflow and underflow, or an enabled SNAN, OPERR, DZ (UM
    // 9.7); an inexact result is written even when its trap is enabled.
    o_post = op_gen && ((ex[6:2] & en[6:2]) != 5'h0 || ovfl || unfl || (ex[1] & en[1]));
    o_wr   = op_gen && !is_cmp && !is_tst &&
             !(ovfl || unfl || ((ex[6:2] & en[6:2]) != 5'h0));
    // FMOVE out (UM 9.7, opclass 011): an operand error or an SNAN to an
    // integer, an overflow, an underflow -- nonmaskable -- or an enabled one,
    // taken after it, in priority order.
    o_xv = 8'd0;
    if (op_out) begin
      if (snan && (out_int || en[6]))         o_xv = 8'd54;
      else if (operr)                         o_xv = 8'd52;
      else if (ovfl)                          o_xv = 8'd53;
      else if (unfl)                          o_xv = 8'd51;
      else if (inex && en[1])                 o_xv = 8'd49;
    end
    // An overflow to memory stores "an undefined result" (UM 9.7.4.2): Table
    // 9-12's value, as UAE040 stores it -- the infinity, or the largest number.
    ovinf = (rmode == rd68041_fpu_pkg::R_N) || (rmode == rd68041_fpu_pkg::R_M && o_rn.sign) ||
            (rmode == rd68041_fpu_pkg::R_P && !o_rn.sign);
    case (ofmt)
      M_S:     o_out = ovfl ? {o_rn.sign, ovinf ? {8'hFF, 23'h0} : {8'hFE, {23{1'b1}}}, 64'h0}
                            : {rd68041_fpu_pkg::pack_s(o_rn), 64'h0};
      M_D:     o_out = ovfl ? {o_rn.sign, ovinf ? {11'h7FF, 52'h0} : {11'h7FE, {52{1'b1}}}, 32'h0}
                            : {rd68041_fpu_pkg::pack_d(o_rn), 32'h0};
      M_X:     o_out = {o_res[79:64], 16'h0, o_res[63:0]};
      default: o_out = {d_int, 64'h0};                       // B W L
    endcase
  end

  // --- the result EX returns, the predicate, the exception -------------------
  // UM Table 9-8: the predicate on N Z I NAN; bit 4 marks the IEEE-nonaware
  // ones, which set BSUN when NAN is (but EQ, NE, F, T: 0x0x, 0x1x... below).
  function automatic logic pred(input logic [4:0] p, input logic [3:0] c);
    logic n, z, nan;
    n = c[3]; z = c[2]; nan = c[0];
    case (p[3:0])
      4'h0: pred = 1'b0;                                       // F
      4'h1: pred = z;                                          // EQ
      4'h2: pred = !(nan || z || n);                           // OGT
      4'h3: pred = z || !(nan || n);                           // OGE
      4'h4: pred = n && !(nan || z);                           // OLT
      4'h5: pred = z || (n && !nan);                           // OLE
      4'h6: pred = !(nan || z);                                // OGL
      4'h7: pred = !nan;                                       // OR
      4'h8: pred = nan;                                        // UN
      4'h9: pred = nan || z;                                   // UEQ
      4'hA: pred = nan || !(n || z);                           // UGT
      4'hB: pred = nan || z || !n;                             // UGE
      4'hC: pred = nan || (n && !z);                           // ULT
      4'hD: pred = nan || z || n;                              // ULE
      4'hE: pred = !z;                                         // NE
      default: pred = 1'b1;                                    // T
    endcase
  endfunction
  wire [3:0] fcc   = fpsr[27:24];
  wire       bsun  = x_cmd[4] && fcc[0];                       // a nonaware test, unordered

  // The frame's longs FSAVE writes, and FRESTORE's.
  function automatic logic [31:0] sv_long(input logic [1:0] s, input logic [4:0] k,
                                          input logic [31:0] f);
    if (k == 5'd0) begin
      case (s)
        rd68041_fpu_pkg::ST_NULL: sv_long = 32'h0000_0000;
        rd68041_fpu_pkg::ST_IDLE: sv_long = 32'h4100_0000;
        rd68041_fpu_pkg::ST_UNIM: sv_long = 32'h4130_0000;
        default: sv_long = 32'h4160_0000;
      endcase
    end else sv_long = f;
  endfunction
  // An unimplemented frame's long k is the busy frame's k + 12.
  wire [4:0] fr_k  = (st == rd68041_fpu_pkg::ST_UNIM && x_idx != 5'd0) ? x_idx + 5'd12 : x_idx;
  wire [4:0] rs_k  = (rs_fmt[23:16] == 8'h30 && x_idx != 5'd0) ? x_idx + 5'd12 : x_idx;
  assign sv_n = (st == rd68041_fpu_pkg::ST_BUSY) ? 5'd25 : (st == rd68041_fpu_pkg::ST_UNIM) ? 5'd13 : 5'd1;
  // FRESTORE's frames (UM 9.8): null, idle, unimplemented, busy, version $41.
  wire rs_ok = (rs_fmt[31:24] == 8'h00) ||
               ((rs_fmt[31:24] == 8'h41) &&
                (rs_fmt[23:16] == 8'h00 || rs_fmt[23:16] == 8'h30 || rs_fmt[23:16] == 8'h60));
  assign rs_n = !rs_ok || rs_fmt[31:24] == 8'h00 ? 5'd1 :
                (rs_fmt[23:16] == 8'h60) ? 5'd25 : (rs_fmt[23:16] == 8'h30) ? 5'd13 : 5'd1;
  wire [31:0] fmt0 = (x_idx == 5'd0) ? x_b : rs_fmt;          // FRESTORE's first, as it comes
  wire        rs_ok0 = (fmt0[31:24] == 8'h00) ||
                       ((fmt0[31:24] == 8'h41) &&
                        (fmt0[23:16] == 8'h00 || fmt0[23:16] == 8'h30 || fmt0[23:16] == 8'h60));

  always_comb begin
    x_res = 32'h0; x_ct = 1'b0;
    x_xv = 8'd0; x_xn = 1'b0; x_xf = 4'h0; x_xea = 32'h0; x_xc = 1'b0;
    case (x_k)
      rd68041_fpu_pkg::FK_OUT:  x_res = s_out[x_idx[1:0]];
      rd68041_fpu_pkg::FK_CRR:  x_res = (x_idx[1:0] == 2'd2) ? {16'h0, fpcr} : (x_idx[1:0] == 2'd1) ? fpsr : fpiar;
      rd68041_fpu_pkg::FK_RGR:  x_res = (x_idx[1:0] == 2'd0) ? {fpr[x_reg][79:64], 16'h0} :
                       (x_idx[1:0] == 2'd1) ? fpr[x_reg][63:32] : fpr[x_reg][31:0];
      rd68041_fpu_pkg::FK_SAVE: x_res = sv_long(st, x_idx, fr[fr_k]);
      rd68041_fpu_pkg::FK_COND: x_ct  = pred(x_cmd[4:0], fcc);
      default: ;
    endcase
    if (x_v) begin
      if (takep) begin
        x_xv = pv; x_xf = 4'h0;                           // pre-instruction, format $0
      end else if (x_k == rd68041_fpu_pkg::FK_OP) begin
        if (op_ill) begin
          x_xv = 8'd11; x_xf = 4'h0;                      // F-line
        end else if (op_unim) begin
          x_xv = 8'd11; x_xn = 1'b1; x_xf = 4'h2;         // UM 9.6.1: after it, format $2
          x_xea = x_eam ? x_ea : 32'h0;
        end else if (op_uns) begin
          x_xv = 8'd55; x_xn = 1'b1; x_xf = 4'h0;         // UM 9.6.2: format $0
        end else if (out_uns) begin
          x_xv = 8'd55; x_xn = 1'b1; x_xf = 4'h3; x_xea = x_eam ? x_ea : 32'h0;
        end else if (ph == 2'd2 && o_e3now) begin
          // An overflow or underflow to a register, E3's or E1's: taken at
          // once, after the instruction (UM 9.7.4.2, 9.7.5.2: "either a
          // pre-instruction or a post-instruction exception"), format $3, as
          // UAE040 takes it.
          x_xv = d_ovfl ? 8'd53 : 8'd51; x_xn = 1'b1; x_xf = 4'h3;
          x_xea = x_eam ? x_ea : 32'h0;
        end
      end else if (x_k == rd68041_fpu_pkg::FK_COND && bsun && en[7]) begin
        x_xv = 8'd48; x_xf = 4'h0; x_xc = 1'b1;           // BSUN: before it, FPSR set
      end else if (x_k == rd68041_fpu_pkg::FK_OUT && x_last && p_xv != 8'd0) begin
        x_xv = p_xv; x_xn = 1'b1; x_xf = 4'h3; x_xea = x_eam ? x_ea : 32'h0;
      end else if (x_k == rd68041_fpu_pkg::FK_RST && x_idx == 5'd0 && !rs_ok0) begin
        x_xv = 8'd14; x_xf = 4'h0;                        // a frame FRESTORE does not take
      end
    end
  end

  // EX waits: for an outstanding record, then for the data path.
  assign x_wait = x_v && ((ph == 2'd0) ? (rec_v || dp_start) : (ph == 2'd1));

  // ===========================================================================
  // The registers
  // ===========================================================================
  // CMDREG3B from CMDREG1B (UM Figure 9-11; the FPSP's gen_except.S mapping).
  function automatic logic [15:0] cmd3(input logic [15:0] c);
    logic [15:0] m;
    m = c;
    if (c[6:0] == 7'h04) m[6:0] = 7'h05;                  // FSQRT, UM 9.8
    cmd3 = (m & 16'h03C3) | ({15'h0, m[2]} << 5) | ({13'h0, m[5:3]} << 2);
  endfunction
  // FPSR as an operation leaves it: EXC replaced, AEXC accumulated, CC.
  function automatic logic [31:0] fpsr_op(input logic [31:0] f, input logic [7:0] ex,
                                          input logic [4:0] ax, input logic [3:0] cc,
                                          input logic wcc);
    fpsr_op = {f[31:28], wcc ? cc : f[27:24], f[23:16], ex, f[7:3] | ax, 3'b000};
  endfunction
  wire [15:0] cmd1 = (cmdw[6:0] == 7'h04) ? {cmdw[15:7], 7'h05} : cmdw;
  wire        o_e3ovf = d_ovfl;  // an E3 overflow's WBTEMP is rounded
  // FMOVE to an integer: the rounded magnitude is the largest negative one.
  wire [63:0] i_lim   = (ofmt == M_B) ? 64'h80 : (ofmt == M_W) ? 64'h8000 : 64'h8000_0000;
  wire        o_negmax = dp_r.sign && (d_iman[66:3] == i_lim);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 8; i++) fpr[i] <= NAN_X;
      for (int i = 0; i <= 24; i++) fr[i] <= '0;
      for (int i = 0; i < 3; i++) begin s_in[i] <= '0; s_out[i] <= '0; end
      fpcr <= '0; fpsr <= '0; fpiar <= '0; st <= rd68041_fpu_pkg::ST_NULL; pend <= 1'b0;
      rs_fmt <= '0; p_xv <= '0; p_pend <= 1'b0; ph <= 2'd0;
      rec_v <= 1'b0; rec_w <= 1'b0; rc_fpr_we <= 1'b0; rc_fpsr_we <= 1'b0; rc_fpcr_we <= 1'b0;
      rc_fpiar_we <= 1'b0; rc_st_we <= 1'b0; rc_null <= 1'b0; rc_fpr_n <= '0; rc_fpr_v <= '0;
      rc_fpsr_v <= '0; rc_fpiar_v <= '0; rc_fpcr_v <= '0; rc_st_v <= '0; rc_pend_v <= 1'b0;
      rc_idle <= 1'b0;
    end else if (crst) begin
      // RSTI resets the FPU (UM 9.2: "a reset ... sets FP7-FP0 to NANs").
      // (Written out: a loop variable here would be a register to yosys.)
      fpr[0] <= NAN_X; fpr[1] <= NAN_X; fpr[2] <= NAN_X; fpr[3] <= NAN_X;
      fpr[4] <= NAN_X; fpr[5] <= NAN_X; fpr[6] <= NAN_X; fpr[7] <= NAN_X;
      fpcr <= '0; fpsr <= '0; fpiar <= '0; st <= rd68041_fpu_pkg::ST_NULL; pend <= 1'b0;
      ph <= 2'd0; rec_v <= 1'b0;
    end else begin
      // --- the record, applied ------------------------------------------------
      // (Only its own micro-operation's: another FP one may leave W while the
      // record's is still in EX.)
      if (w_commit && rec_v && rec_w) begin
        if (rc_fpr_we)   fpr[rc_fpr_n] <= rc_fpr_v;
        if (rc_fpsr_we)  fpsr <= rc_fpsr_v;
        if (rc_fpcr_we)  fpcr <= rc_fpcr_v;
        if (rc_fpiar_we) fpiar <= rc_fpiar_v;
        // The state: as the record says; else an FP instruction leaves the
        // null state for the idle one (UM 9.8), and an operation that leaves
        // nothing pending, any frame that was held.
        if (rc_st_we) begin st <= rc_st_v; pend <= rc_pend_v; end
        else if (rc_null && (st == rd68041_fpu_pkg::ST_NULL || rc_idle)) begin st <= rd68041_fpu_pkg::ST_IDLE; pend <= 1'b0; end
        rec_v <= 1'b0;
        if (rc_st_we && rc_st_v == rd68041_fpu_pkg::ST_NULL) begin
          fpr[0] <= NAN_X; fpr[1] <= NAN_X; fpr[2] <= NAN_X; fpr[3] <= NAN_X;
          fpr[4] <= NAN_X; fpr[5] <= NAN_X; fpr[6] <= NAN_X; fpr[7] <= NAN_X;
        end
      end

      // --- execute ------------------------------------------------------------
      if (can && !takep && !flush) begin
        // A record by default makes nothing change but null to idle.
        rc_fpr_we <= 1'b0; rc_fpsr_we <= 1'b0; rc_fpcr_we <= 1'b0; rc_fpiar_we <= 1'b0;
        rc_st_we <= 1'b0; rc_null <= 1'b0; rc_idle <= 1'b0;
        // Done in this clock: EX hands it on (or holds it, done).
        if (!dp_start) ph <= 2'd2;
        case (x_k)
          rd68041_fpu_pkg::FK_IN:  s_in[x_idx[1:0]] <= x_b;
          rd68041_fpu_pkg::FK_CRW: begin
            rec_v <= 1'b1; rc_null <= 1'b1;
            case (x_idx[1:0])
              2'd2:    begin rc_fpcr_we <= 1'b1; rc_fpcr_v <= x_b[15:0] & 16'hFFF0; end
              2'd1:    begin rc_fpsr_we <= 1'b1; rc_fpsr_v <= x_b & 32'h0FFF_FFF8; end
              default: begin rc_fpiar_we <= 1'b1; rc_fpiar_v <= x_b; end
            endcase
          end
          rd68041_fpu_pkg::FK_RGW: begin
            s_in[x_idx[1:0]] <= x_b;
            if (x_idx[1:0] == 2'd2) begin
              rec_v <= 1'b1; rc_null <= 1'b1;
              rc_fpr_we <= 1'b1; rc_fpr_n <= x_reg;
              rc_fpr_v <= {s_in[0][31:16], s_in[1], x_b};
            end
          end
          rd68041_fpu_pkg::FK_COND: begin
            // BSUN (UM 9.7.1): set in EXC when a nonaware test finds NAN.
            // FPIAR is not loaded: the FPSP copies the frame's PC into it
            // "for MC68881/MC68882 compatibility" (UM 9.7.1.1).
            if (bsun) begin
              rec_v <= 1'b1; rc_fpsr_we <= 1'b1;
              rc_fpsr_v <= {fpsr[31:16], fpsr[15:8] | 8'h80, fpsr[7:0]};
            end
          end
          rd68041_fpu_pkg::FK_RST: begin
            if (x_idx == 5'd0) rs_fmt <= x_b;
            else fr[rs_k] <= x_b;
          end
          rd68041_fpu_pkg::FK_SAVE: begin
            // FSAVE's last long word: the FPU idle after it (UM 9.8).
            if (x_last) begin
              rec_v <= 1'b1; rc_st_we <= 1'b1;
              rc_st_v <= (st == rd68041_fpu_pkg::ST_NULL) ? rd68041_fpu_pkg::ST_NULL : rd68041_fpu_pkg::ST_IDLE; rc_pend_v <= 1'b0;
            end
          end
          rd68041_fpu_pkg::FK_OUT: begin
            // FMOVE out's exception, taken after its last long word: an E1
            // one, saved in the unimplemented-instruction frame's size (below).
            // Without one it is a nonconditional instruction executed: null
            // to idle (UM 9.8), as for every one but FSAVE and FRESTORE.
            if (x_last && p_pend) begin
              rec_v <= 1'b1; rc_st_we <= 1'b1; rc_st_v <= rd68041_fpu_pkg::ST_UNIM; rc_pend_v <= 1'b1;
            end else begin
              rec_v <= 1'b1; rc_null <= 1'b1;
            end
          end
          // FMOVEM out, and the control registers read: null to idle too
          // (UM 9.8: "until the first nonconditional floating-point
          // instruction is executed"; doc/bugs-found.md M13).
          rd68041_fpu_pkg::FK_RGR,
          rd68041_fpu_pkg::FK_CRR: begin
            rec_v <= 1'b1; rc_null <= 1'b1;
          end
          rd68041_fpu_pkg::FK_REND: begin
            if (!rs_go) begin
              rec_v <= 1'b1; rc_st_we <= 1'b1;
              if (rs_fmt[31:24] == 8'h00) begin
                rc_st_v <= rd68041_fpu_pkg::ST_NULL; rc_pend_v <= 1'b0;
                rc_fpcr_we <= 1'b1; rc_fpcr_v <= '0;
                rc_fpsr_we <= 1'b1; rc_fpsr_v <= '0;
                rc_fpiar_we <= 1'b1; rc_fpiar_v <= '0;
              end else if (rs_fmt[23:16] == 8'h00) begin
                rc_st_v <= rd68041_fpu_pkg::ST_IDLE; rc_pend_v <= 1'b0;
              end else if (fr[18][26] || fr[18][25]) begin
                rc_st_v <= (rs_fmt[23:16] == 8'h30) ? rd68041_fpu_pkg::ST_UNIM : rd68041_fpu_pkg::ST_BUSY;
                rc_pend_v <= 1'b1;
              end else begin
                rc_st_v <= rd68041_fpu_pkg::ST_IDLE; rc_pend_v <= 1'b0;   // nothing to report, nor to run
              end
            end else begin
              ph <= 2'd1;                              // the resumed operation, below
            end
          end
          rd68041_fpu_pkg::FK_OP: begin
            if (dp_start) begin
              ph <= 2'd1;
            end else begin
              rec_v <= 1'b1; rc_null <= 1'b1; rc_idle <= 1'b1;
              // FPIAR (UM 9.2.4): the instruction's address.
              rc_fpiar_we <= !op_ill; rc_fpiar_v <= x_pc;
              if (op_unim || op_uns || out_uns) begin
                // The frame (doc/fpu.md, Table 9-16).
                rc_st_we  <= 1'b1;
                rc_st_v   <= op_unim ? rd68041_fpu_pkg::ST_UNIM : rd68041_fpu_pkg::ST_BUSY;
                rc_pend_v <= 1'b1;
                fr[16] <= {cmd1, 16'h0};                                // CMDREG1B
                fr[10] <= x_pc;                                         // FPIARCU
                fr[2]  <= 32'h0;                                        // CU_SAVEPC
                fr[14] <= {8'h0, 3'b000, !op_unim, 4'h0, 16'h0};        // NMCEXC unsup
                // E1: in the unimplemented-instruction frame, a packed source
                // (UM 9.8's E1, as the FPSP's get_op reads it; not Table
                // 9-16's "always 1": doc/manual-contradictions.md 9).
                fr[18] <= {5'h0, !op_unim || pk_src, 2'b00, 2'b00, op_unim, out_uns, 4'h0, 16'h0};
                if (out_uns) begin
                  fr[15] <= {tag_of(outn), 5'h0, 24'h0};
                  fr[22] <= {fpr[dstr][79:64], 16'h0};
                  fr[23] <= fpr[dstr][63:32];
                  fr[24] <= fpr[dstr][31:0];
                end else if (pk_src) begin
                  // A packed source's 96 bits: 95-64 in FPTM 31-0, the rest in ETM.
                  fr[15] <= 32'h0;
                  fr[21] <= mem96[95:64];
                  fr[23] <= mem96[63:32];
                  fr[24] <= mem96[31:0];
                  fr[17] <= {tag_of(dstn), 5'h0, 24'h0};
                end else begin
                  fr[15] <= {tag_of(srcn), 5'h0, 24'h0};
                  fr[22] <= {srcn.sign, srcn.exp[14:0], 16'h0};
                  fr[23] <= srcn.man[63:32];
                  fr[24] <= srcn.man[31:0];
                  fr[17] <= {tag_of(dstn), 5'h0, 24'h0};
                  fr[19] <= {fpr[dstr][79:64], 16'h0};
                  fr[20] <= fpr[dstr][63:32];
                  fr[21] <= fpr[dstr][31:0];
                end
              end else if (no_dp) begin
                // FTST, FMOVE.X out: at once.
                rc_fpsr_we <= 1'b1;
                rc_fpsr_v  <= fpsr_op(fpsr, o_exc, o_aexc, o_cc, !op_out);
                if (op_out) begin
                  s_out[0] <= o_out[95:64]; s_out[1] <= o_out[63:32]; s_out[2] <= o_out[31:0];
                  p_xv <= o_xv; p_pend <= (o_xv != 8'd0);
                  if (o_xv != 8'd0) begin
                    fr[16] <= {cmd1, 16'h0}; fr[10] <= x_pc; fr[2] <= 32'h0;
                    fr[15] <= {tag_of(outn), 5'h0, 24'h0};
                    fr[22] <= {fpr[dstr][79:64], 16'h0};
                    fr[23] <= fpr[dstr][63:32]; fr[24] <= fpr[dstr][31:0];
                    fr[18] <= {5'h0, 1'b1, 2'b00, 2'b00, 1'b0, 1'b1, 4'h0, 16'h0};
                    fr[14] <= {8'h0, 4'h0, 1'b1, 3'b000, 16'h0};
                  end
                end else if (o_post) begin
                  rc_st_we <= 1'b1; rc_st_v <= rd68041_fpu_pkg::ST_UNIM; rc_pend_v <= 1'b1;
                  fr[16] <= {cmd1, 16'h0}; fr[10] <= x_pc; fr[2] <= 32'h0;
                  fr[15] <= {tag_of(srcn), 5'h0, 24'h0};
                  fr[22] <= {srcn.sign, srcn.exp[14:0], 16'h0};
                  fr[23] <= srcn.man[63:32]; fr[24] <= srcn.man[31:0];
                  fr[18] <= {5'h0, 1'b1, 2'b00, 8'h0, 16'h0};
                  fr[14] <= {8'h0, 4'h0, o_exc[6], 3'b000, 16'h0};
                end
              end
            end
          end
          default: ;
        endcase
      end

      // --- the data path's answer: FK_OP's (or FRESTORE's resumed) outcome ------
      if (ph == 2'd1 && dp_done && !flush) begin
        ph <= 2'd2;
        rec_v <= 1'b1; rc_null <= 1'b1; rc_idle <= 1'b1;
        rc_fpr_we <= o_wr; rc_fpr_n <= dstr; rc_fpr_v <= o_res;
        rc_fpsr_we <= 1'b1; rc_fpsr_v <= fpsr_op(fpsr, o_exc, o_aexc, o_cc, !op_out);
        rc_fpiar_we <= 1'b1; rc_fpiar_v <= i_pc;
        rc_fpcr_we <= 1'b0;
        rc_st_we <= 1'b0;
        if (op_out) begin
          s_out[0] <= o_out[95:64]; s_out[1] <= o_out[63:32]; s_out[2] <= o_out[31:0];
          p_xv <= o_xv; p_pend <= (o_xv != 8'd0);
          if (o_xv != 8'd0) begin
            // UM Table 9-16, opclass 011: E1 and T; ETEMP the source register,
            // FPTEMP the intermediate (OVFL, UNFL, INEX), WBTEMP the integer
            // (OPERR); NMCEXC the nonmaskable ones.
            fr[16] <= {cmd1, 16'h0}; fr[10] <= i_pc; fr[2] <= 32'h0;
            fr[15] <= {tag_of(outn), 5'h0, 24'h0};
            fr[22] <= {fpr[dstr][79:64], 16'h0};
            fr[23] <= fpr[dstr][63:32]; fr[24] <= fpr[dstr][31:0];
            fr[19] <= {dp_r.sign, d_iexp[14:0], 16'h0};
            fr[20] <= d_iman[66:35]; fr[21] <= d_iman[34:3];
            fr[17] <= {3'b000, d_iexp[15], 4'h0, 24'h0};
            fr[6]  <= 32'h0; fr[7] <= d_iman[66:35]; fr[8] <= d_iman[34:3];
            if (out_int && o_xv == 8'd52) begin
              // An integer that does not fit: FPTEMP as the FPSP reads it
              // (x_operr.S) -- the largest negative integer, which the MC68040
              // calls an operand error too, sign-extended behind $C000; a real
              // overflow, an exponent of $4000 with its sign.
              if (o_negmax) begin
                fr[19] <= 32'hC000_0000; fr[20] <= 32'hFFFF_FFFF;
                fr[21] <= (ofmt == M_B) ? 32'hFFFF_FF80 : (ofmt == M_W) ? 32'hFFFF_8000 :
                                          32'h8000_0000;
              end else begin
                fr[19] <= {dp_r.sign, 15'h4000, 16'h0}; fr[20] <= 32'h0; fr[21] <= 32'h0;
              end
            end
            fr[18] <= {5'h0, 1'b1, 2'b00, 2'b00, 1'b0, 1'b1, 4'h0, 16'h0};
            fr[14] <= {8'h0, 4'h0, (o_xv == 8'd54), (o_xv == 8'd52), (o_xv == 8'd53),
                       (o_xv == 8'd51), 16'h0};
          end
        end else if (o_post) begin
          // E3 in the busy frame, E1 in the unimplemented-instruction frame's
          // size (the FPSP's x_snan.S expands that one into a busy frame).
          rc_st_we <= 1'b1; rc_pend_v <= 1'b1;
          rc_st_v <= o_e3 ? rd68041_fpu_pkg::ST_BUSY : rd68041_fpu_pkg::ST_UNIM;
          fr[10] <= i_pc; fr[2] <= 32'h0;
          fr[16] <= {cmd1, 16'h0};
          fr[15] <= {tag_of(srcn), 5'h0, 24'h0};
          fr[22] <= {srcn.sign, srcn.exp[14:0], 16'h0};
          fr[23] <= srcn.man[63:32]; fr[24] <= srcn.man[31:0];
          fr[17] <= {tag_of(dstn), 5'h0, 24'h0};
          fr[14] <= {8'h0, 4'h0, o_exc[6], o_exc[5], o_exc[4], o_exc[3], 16'h0};
          if (o_e3) begin
            // E3 (UM Table 9-16): WBTEMP the intermediate -- rounded for an
            // overflow, not for an underflow or inexact -- with its guard,
            // round, sticky and its exponent's bit 15; CMDREG3B.
            fr[13] <= {cmd3(x_cmd), 16'h0};
            fr[6]  <= {dp_r.sign, d_iexp[14:0], 16'h0};
            fr[7]  <= o_e3ovf ? dp_r.man[63:32] : d_iman[66:35];
            fr[8]  <= o_e3ovf ? dp_r.man[31:0]  : d_iman[34:3];
            fr[15] <= {tag_of(srcn), 1'b0, 1'b0, 1'b0, o_e3ovf ? 2'b00 : d_iman[2:1],
                       !o_e3ovf && d_iman[0], 7'h0, 16'h0};
            fr[17] <= {tag_of(dstn), 1'b0, 4'h0, 3'b000, d_iexp[15], 4'h0, 16'h0};
            // E3, and T clear even when taken at once: Table 9-16 has "either",
            // UAE040 clears it, the FPSP ignores it with E3 (util.S, g_opcls).
            fr[18] <= {5'h0, 1'b0, 1'b1, 1'b0, 8'h0, 16'h0};
            fr[19] <= {fpr[dstr][79:64], 16'h0};
            fr[20] <= fpr[dstr][63:32]; fr[21] <= fpr[dstr][31:0];
          end else begin
            // E1: FPTEMP the destination -- or, for FMOVE, FABS, FNEG, the
            // intermediate (overflow: rounded) -- and ETEMP the source.
            fr[18] <= {5'h0, 1'b1, 2'b00, 8'h0, 16'h0};
            if (cuo && (d_ovfl || d_unfl || d_inex)) begin
              fr[19] <= {dp_r.sign, d_iexp[14:0], 16'h0};
              fr[20] <= d_ovfl ? dp_r.man[63:32] : d_iman[66:35];
              fr[21] <= d_ovfl ? dp_r.man[31:0]  : d_iman[34:3];
              fr[15] <= {rd68041_fpu_pkg::C_NORM, 5'h0, 24'h0};
            end else begin
              fr[19] <= {fpr[dstr][79:64], 16'h0};
              fr[20] <= fpr[dstr][63:32]; fr[21] <= fpr[dstr][31:0];
            end
          end
        end
      end

      // The record's micro-operation goes on to W (the clock it is made, for
      // one done at once); a flushed one's record goes.
      if (x_adv && x_v) rec_w <= 1'b1;
      if (can && !takep && !flush && !x_adv) rec_w <= 1'b0;
      if (ph == 2'd1 && dp_done && !x_adv) rec_w <= 1'b0;
      if (flush && !(w_commit && rec_w)) begin rec_v <= 1'b0; rec_w <= 1'b0; end
      if (flush || x_adv) ph <= 2'd0;
    end
  end

  logic unused_ok;
  assign unused_ok = &{1'b0, dp_busy, d_spec, o_rn.snan, src_m.snan, nmc[7:5], d_iexp[17:16],
                       fmt0[15:0],
                       fr[14][31:24]};

endmodule
