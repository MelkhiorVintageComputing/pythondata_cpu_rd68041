// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The integer unit: decode, <ea> calculate, <ea> fetch, execute and write-back
// (UM 2.1; doc/microarch.md), with instruction fetch (rd68041_ifu) and the data
// memory unit (rd68041_dmu) beside it, and the arbiter that puts both on the bus
// unit's one request port.
//
// One micro-operation per stage, in order. A stage holds while the one after it
// does. The hazards and how each is met:
//
//   <ea> calculate needs an address or index register:
//     - an update an older micro-operation made here ((An)+, -(An)) is taken
//       from that one's OF, EX or W register, newest first;
//     - a result an older one will compute in EX is waited for until it is in W,
//       and taken from there; then the register file.
//   execute reads its operands from the register file, and from W for the one
//     micro-operation ahead of it, which is all that can be in flight.
//   <ea> fetch reads through the data memory unit, which waits for any queued
//     write to the same long word (UM 7.7).
//   a taken branch, resolved in EX, empties decode, <ea> calculate and <ea>
//     fetch -- cancelling a read already on the bus -- and restarts fetching.
//
// The condition codes exist twice: as EX has made them (for the next branch)
// and as write-back has committed them (the architectural ones, which an
// exception puts back into EX's).
//
// Exceptions (M8; doc/microarch.md, "Exceptions") are taken at write-back. A
// micro-operation carries the vector of the one it takes (`xvec`): decode's
// (illegal, privileged, A- and F-line, TRAP #n, a fetch that faulted), <ea>
// fetch's (an operand read that faulted) or execute's (the traps, a format
// error, an odd branch target). Write-back completes it first or not
// (`xnext`), empties the pipeline behind it, and the exception state machine
// (`xs`) changes the SR and the stack pointer once every write has gone, then
// lets rd68041_exc build the frame and fetch the vector. A trace and an
// interrupt are taken the same way at an instruction boundary. The SR's
// writers are serialising: nothing issues after one until it is past
// write-back, and fetching starts again behind it.
//
// No caches and no MMU yet. A micro-operation marked halt (an instruction not
// built) stops everything when it reaches write-back (`halted`); so does a
// double bus fault.

module rd68041_core #(
    parameter int FPU = 1                    // 0: the MC68LC040
) (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          crst,

    // The bus unit's request port (doc/bus.md).
    output logic          req_valid,
    input  logic          req_ready,
    output logic [31:0]   req_addr,
    output logic          req_write,
    output logic  [1:0]   req_siz,
    output logic  [1:0]   req_tt,
    output logic  [2:0]   req_tm,
    output logic          req_lock,
    output logic          req_locke,
    output logic  [1:0]   req_tln,
    output logic  [1:0]   req_upa,
    output logic          req_ciout,
    output logic          lock_seq,
    output logic [127:0]  req_wdata,
    input  logic          rsp_valid,
    input  logic [31:0]   rsp_data,
    input  logic  [1:0]   rsp_lw,
    input  logic          rsp_tci,
    input  logic          rsp_done,
    input  logic          rsp_err,
    input  logic          rsp_avec,

    // Interrupts (UM 8.2.9) and the RESET instruction.
    input  logic  [2:0]   ipl_n,        // IPL2-IPL0, as on the pins
    output logic          ipend,        // an interrupt is pending (IPEND, active high)
    output logic          rsto,         // RSTO, active high

    // The snooper's port to the caches (rd68041_snoop).
    input  logic [31:0]   sn_addr,
    output logic          sn_dhit,
    output logic          sn_ddirty,
    output logic          sn_ihit,
    output logic [31:0]   sn_rdata,
    input  logic          sn_wr,
    input  logic  [3:0]   sn_wbe,
    input  logic [31:0]   sn_wdata,
    input  logic          sn_dinv,
    input  logic          sn_iinv,
    input  logic          sn_rd,

    // For testbenches and, later, PST.
    output logic          retire,       // an instruction completed this clock
    output logic [31:0]   retire_pc,    // its address
    output logic [31:0]   retire_npc,   // the next instruction's
    output logic          halted,
    output logic [15:0]   sr
);

  // ===========================================================================
  // State
  // ===========================================================================
  logic [4:0] ccr_x, ccr_w;                  // EX's and the committed CCR

  // The SR's system byte (UM 2.2.1): T1 T0 S M, 0, I2-I0. Reset leaves $27
  // (UM 8.2.10): S set, M and the trace bits clear, the mask at 7.
  logic [7:0] sr_sys;
  wire        sup    = sr_sys[5];
  wire        msp_on = sr_sys[4];
  wire  [1:0] tr     = sr_sys[7:6];
  wire  [2:0] imask  = sr_sys[2:0];
  assign sr = {sr_sys, 3'b000, ccr_w};

  // The control registers (UM 2.2.2, 3.1, 4.2), written at write-back; MMUSR by
  // PTEST too. A7
  // (register 15) is the active stack pointer -- the USP in user mode, the ISP
  // or, with M set, the MSP in supervisor mode -- and the other two are held
  // here; whatever changes S or M swaps them.
  logic  [2:0] sfc, dfc;
  logic [31:0] cacr, tc, itt0, itt1, dtt0, dtt1, usp, isp, vbr, msp, mmusr, urp, srp;

  // Interrupts (UM 7.5.1, 8.2.9): IPL2-IPL0 synchronised, and a level taken
  // once two consecutive samples agree; pending above the mask, or on a
  // transition to level 7 until that is taken.
  logic [2:0] ipl_s0, ipl_s1, ipl_s2, ipl_lvl;
  logic       nmi;
  wire        int_pend = (ipl_lvl > imask) || nmi;
  assign ipend = int_pend;

  // ===========================================================================
  // The arbiter: the data memory unit first, then instruction fetch.
  // ===========================================================================
  logic        d_valid, d_ready, d_write, d_rsp, d_done, d_err;
  logic [31:0] d_addr;
  logic  [1:0] d_siz, d_tt;
  logic  [2:0] d_tm;
  logic        d_lock, d_locke, lock_open, d_avec, d_ci, d_tci;
  logic  [1:0] d_tln, d_upa, d_lw;
  logic [127:0] d_wdata128;
  logic        f_valid, f_ready, f_rsp, f_done, f_err, f_ci;
  logic [31:0] f_addr;
  logic  [2:0] f_tm;
  logic  [1:0] f_siz, f_upa;

  wire pick_d = d_valid;
  assign req_valid = d_valid || f_valid;
  assign req_addr  = pick_d ? d_addr : f_addr;
  assign req_write = pick_d && d_write;
  assign req_siz   = pick_d ? d_siz : f_siz;
  assign req_ciout = pick_d ? d_ci : f_ci;
  assign req_upa   = pick_d ? d_upa : f_upa;
  assign req_tln   = pick_d ? d_tln : 2'b00;
  assign req_tt    = pick_d ? d_tt : rd68041_pkg::TT_NORMAL;
  assign req_lock  = pick_d && d_lock;
  assign req_locke = pick_d && d_locke;
  assign lock_seq  = lock_open;
  assign req_tm    = pick_d ? d_tm : f_tm;
  assign req_wdata = d_wdata128;
  assign d_ready   = pick_d && req_ready;
  assign f_ready   = !pick_d && req_ready;

  // Whose transfer is on the bus, and whose the responses are: those are
  // registered on the edge after the event, which may also be the edge the next
  // transfer is taken on, so they go to whoever owned the bus before that edge.
  logic own_d, rsp_own_d;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      own_d     <= 1'b0;
      rsp_own_d <= 1'b0;
    end else begin
      if (req_valid && req_ready) own_d <= pick_d;
      rsp_own_d <= own_d;
    end
  end
  assign d_rsp  = rsp_valid &&  rsp_own_d;
  assign d_done = rsp_done  &&  rsp_own_d;
  assign d_err  = rsp_err   &&  rsp_own_d;
  assign d_avec = rsp_avec  &&  rsp_own_d;
  assign d_lw   = rsp_lw;
  assign d_tci  = rsp_tci;
  assign f_rsp  = rsp_valid && !rsp_own_d;
  assign f_done = rsp_done  && !rsp_own_d;
  assign f_err  = rsp_err   && !rsp_own_d;

  // ===========================================================================
  // Reset (UM 8.2.10): the interrupt stack pointer from 0, the PC from 4.
  // ===========================================================================
  typedef enum logic [1:0] { R_SSP, R_PC, R_RUN, R_HALT } rs_e;
  rs_e rs;

  // ===========================================================================
  // Pipeline registers
  // ===========================================================================
  rd68041_pkg::uop_t ea_u, of_u, ex_u, w_u;
  logic [31:0] of_ea, ex_ea, w_ea;            // the effective address
  logic [31:0] of_an, ex_an, w_an;            // the address register's update
  logic [31:0] of_md;                         // the operand read
  logic        of_got;                        // ... is in of_md
  logic        of_er, of_av;                  // ... ended in a bus error, or with AVEC
  logic        of_at, of_ma;                  // ... an ATC fault, on the second page
  logic [31:0] ex_md;
  logic        ex_er, ex_av;
  logic        ex_mw, ex_mok;                 // EX's read was released from OF, answered
  logic        of_rel_q;                      // ... OF released one last clock
  logic [31:0] w_res;                         // the result
  logic [31:0] w_res2;                        // the second result
  logic [31:0] w_std;                         // the store's data
  logic        w_nowr;                        // EX cancelled the register results
  logic        w_nost;                        // ... and the store
  logic        w_cof;                         // a change of flow, for tracing
  logic        div_go, div_done;              // the divider was started; it has answered
  logic [31:0] w_npc;                         // the next instruction's address
  logic  [4:0] w_ccr;                         // the CCR it leaves

  // ===========================================================================
  // The exception state machine (doc/microarch.md, "Exceptions")
  // ===========================================================================
  //   XS_IDLE  running
  //   XS_SR    a serialising instruction or an exception has left write-back,
  //            and everything behind it is gone: once every write has, the SR
  //            and the stack pointers change, in one clock
  //   XS_SEQ   rd68041_exc builds the frame and fetches the vector
  //   XS_STOP  STOP: waiting for an interrupt (UM 8.2.9; trace never stops it)
  //   XS_RST   RESET: RSTO for 512 clocks (UM 7.10)
  //   XS_CMO   CINV, CPUSH: until the caches are done (UM 4.2)
  //   XS_UNDO  before XS_SR, for an exception an instruction takes after some of
  //            its micro-operations completed: the registers they changed are put
  //            back, so that the instruction restarts from where it began (below)
  typedef enum logic [2:0] { XS_IDLE, XS_SR, XS_SEQ, XS_STOP, XS_RST, XS_CMO, XS_UNDO } xs_e;
  xs_e         xs;
  logic        x_exc;                         // XS_SR: an exception (else serialising only)
  logic        x_int;                         // ... an interrupt, at level x_lvl
  logic  [2:0] x_lvl;
  logic  [7:0] x_vec;
  logic  [3:0] x_fmt;
  logic [31:0] x_pc, x_addr;                  // the frame's PC; ADDR or FA
  logic [31:0] x_pcf;                         // $4's PC of the faulted instruction
  logic [15:0] x_ssw, x_sr0;                  // the SSW; the SR the frame holds
  logic        x_srw, x_stp, x_rst, x_trc;    // serialising: an SR write, STOP, RESET, traced
  logic        x_cmo;                         // ... CINV or CPUSH: x_cmf {caches, scope}, at x_cma
  logic        x_cmm;                         // ... PFLUSH or PTEST: x_cmf[2:0] the kind
  logic  [3:0] x_cmf;
  logic        x_cmp;
  logic [31:0] x_cma;
  logic  [7:0] x_srn;                         // ... the new SR's system byte
  logic [31:0] x_npc, x_ipc;                  // ... the next instruction, and this one
  logic        x_rerun;                       // ... RTE of a throwaway frame: RTE again
  logic        xs_int;                        // the sequence running is an interrupt's
  logic  [8:0] rst_cnt;
  logic        x_flush;                       // write-back empties the pipeline behind it

  // The undo log. The pipeline commits an instruction a micro-operation at a
  // time, so an instruction can have changed registers -- an address register
  // a (An)+ moved, the registers MOVEM loaded so far -- before a later
  // micro-operation of it faults. The restart model wants it to have done
  // nothing (UM 8.3, doc/checkpoint.md): each architectural register an
  // instruction's micro-operations before its last write is saved here the
  // first time, and XS_UNDO writes the saved values back when the instruction
  // takes an exception before completing.
  logic [31:0] ush [0:15];
  logic [15:0] umask;
  logic        ic_op, ic_busy;                // CINV, CPUSH of the instruction cache
  logic        dc_op, dc_busy;                // ... and of the data cache
  logic        mm_op, mm_busy;                // PFLUSH, PTEST
  logic        wf_pend, wf_xs, wf_ack;        // a write faulted (rd68041_dmu)
  logic [383:0] wf_tail;                      // ... its frame's long words 3-14
  logic        x_wft;                         // the frame is a write's fault: wf_tail
  logic        mmusr_we;
  logic [31:0] mmusr_v;
  // The instruction ATC's flushes and table searches, done by the data unit.
  logic        if_flush, if_fl_s, iw_req, iw_s, iw_fill, iw_fill_s, iw_done;
  logic  [1:0] if_fl_kind;
  logic [31:0] if_fl_la, iw_la, iw_entry;
  logic [31:12] iw_fill_la;

  // ===========================================================================
  // Instruction fetch and decode
  // ===========================================================================
  logic [191:0] q_win;
  logic   [3:0] q_cnt, q_pop;
  logic  [31:0] q_pc;
  logic  [11:0] q_err, q_atc;
  logic         q_aerr;
  logic         go;
  logic  [31:0] go_pc;
  logic         redirect;
  logic  [31:0] redirect_pc;

  rd68041_ifu u_ifu (
      .clk(clk), .rst_n(rst_n), .crst(crst),
      .go(go), .go_pc(go_pc),
      .stop(halted || lock_open || xs == XS_SR || xs == XS_STOP || xs == XS_RST ||
            xs == XS_CMO || xs == XS_UNDO),
      .sup(sup), .ic_en(cacr[15]), .itt0(itt0), .itt1(itt1),
      .q_win(q_win), .q_cnt(q_cnt), .q_pc(q_pc), .q_err(q_err), .q_atc(q_atc), .q_aerr(q_aerr), .q_pop(q_pop),
      .f_valid(f_valid), .f_ready(f_ready), .f_addr(f_addr), .f_tm(f_tm), .f_siz(f_siz),
      .f_ci(f_ci), .f_upa(f_upa),
      .f_rsp(f_rsp), .f_lw(rsp_lw), .f_data(rsp_data), .f_tci(rsp_tci), .f_done(f_done),
      .f_err(f_err),
      .ic_op(ic_op), .ic_scope(x_cmf[1:0]), .ic_addr(x_cma), .ic_p8k(tc[14]), .ic_busy(ic_busy),
      .tc_e(tc[15]), .tc_p(tc[14]),
      .if_flush(if_flush), .if_fl_kind(if_fl_kind), .if_fl_la(if_fl_la), .if_fl_s(if_fl_s),
      .iw_req(iw_req), .iw_la(iw_la), .iw_s(iw_s), .iw_fill(iw_fill), .iw_fill_la(iw_fill_la),
      .iw_fill_s(iw_fill_s), .iw_entry(iw_entry), .iw_done(iw_done),
      .sn_addr(sn_addr), .sn_ihit(sn_ihit), .sn_iinv(sn_iinv));



  // The register file's ports (below, "The register file").
  logic [4:0]  ra, rb, rc, rd, re;
  logic [31:0] qa, qb, qc, qd, qe, q15, rf_o0, rf_o1, rf_o2;
  logic        we0, we1, we2;
  logic [4:0]  wa0, wa1, wa2;
  logic [31:0] wd0, wd1, wd2;
  logic [3:0]  wm0;

  logic  [4:0] seq;                           // micro-operations issued of this instruction
  logic  [1:0] de_ind, dec_ind;               // the indirection preludes it has
  logic  [4:0] de_len;                        // its length
  wire         step = (seq != 5'd0);
  logic [15:0] de_op;
  logic [31:0] de_pc;
  logic        de_halted;                     // a halt has been issued

  logic [rd68041_pkg::UOP_W-1:0] dec_v;
  logic [3:0]  dec_need, dec_pop;
  logic        dec_more;

  // MOVEM's register list (PRM 4): the mask from its first extension word, then
  // what is left of it. The next register is its lowest set bit -- which for
  // -(An), whose mask is reversed, is A7 first and D0 last.
  logic [15:0] mv_mask, mv_src, mv_bit;
  logic  [4:0] mv_k;
  logic  [3:0] mv_reg;
  logic        mv_last, mv_used;
  wire  [15:0] cur_op = step ? de_op : q_win[191:176];
  // FMOVEM's list (PRM 5): the command word's low byte, or Dn's, read once the
  // pipeline behind decode is empty (de_dyn); reversed, so that the lowest
  // set bit is the first register both ways: FP0 first for the control modes
  // and (An)+, FP7 first for -(An) -- the decoder takes mv_reg[2:0].
  wire  [15:0] cmd_w  = q_win[175:160];
  wire         de_fmm = (FPU != 0) && !step && (q_win[191:182] == 10'b1111_0010_00) &&
                        (cmd_w[15:14] == 2'b11);
  wire         de_dyn = de_fmm && cmd_w[11];
  function automatic logic [7:0] rev8(input logic [7:0] v);
    rev8 = {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]};
  endfunction
  assign mv_src = step   ? mv_mask :
                  de_dyn ? {8'h0, rev8(qa[7:0])} :
                  de_fmm ? {8'h0, rev8(cmd_w[7:0])} : q_win[175:160];
  always_comb begin
    mv_bit = 16'h0;
    mv_reg = 4'd0;
    for (int b = 15; b >= 0; b--) begin
      if (mv_src[b]) begin
        mv_bit = 16'h1 << b;
        mv_reg = (cur_op[5:3] == 3'd4) ? 4'(15 - b) : 4'(b);
      end
    end
  end
  assign mv_last = (mv_src & (mv_src - 16'h1)) == 16'h0;

  logic        dec_qwait;
  logic  [4:0] fp_svn, fp_rsn;
  rd68041_decode #(.FPU(FPU)) u_dec (
      .op      (cur_op),
      .w       (step ? q_win[191:16] : q_win[175:0]),
      .seq     (seq),
      .ind_in  (de_ind),
      .len_in  (de_len),
      .pc      (step ? de_pc : q_pc),
      .mv_reg  (mv_reg),
      .mv_last (mv_last),
      .mv_k    (mv_k),
      .sup     (sup),
      .msp_on  (msp_on),
      .mv_none (mv_src == 16'h0),
      .fp_svn  (fp_svn),
      .fp_rsn  (fp_rsn),
      .qwait   (dec_qwait),
      .uop     (dec_v),
      .need    (dec_need),
      .pop     (dec_pop),
      .more    (dec_more),
      .mv_used (mv_used),
      .ind_out (dec_ind));

  rd68041_pkg::uop_t dec_u;
  assign dec_u = dec_v;

  wire de_avail = step ? (q_cnt >= dec_pop)
                       : (q_cnt != 4'd0) && ({1'b0, q_cnt} >= 5'd1 + {1'b0, dec_need});

  // An instruction a word of which came from a fetch that faulted is an access
  // fault (format $7) instead, as soon as that word is in the queue -- or at
  // once, if the operation word is one (its `need` means nothing). Not once
  // its words are all there: fetching stops at an ATC fault (rd68041_ifu,
  // xstop), and the words after the faulted ones never come (an instruction
  // whose extension words are on a page not resident, doc/bugs-found.md).
  // FA is the faulted fetch's address; the SSW says a long-word read of
  // program space (UM 8.4.6.7: "the TM field contains $2 and $6"). A redirect
  // to an odd address is an address error (format $2, ADDR the address with
  // bit 0 cleared).
  wire [11:0] de_wmask = (12'h1 << (4'd1 + dec_need)) - 12'h1;
  wire [11:0] de_qmask = (12'h1 << q_cnt) - 12'h1;     // the words there
  wire        de_ferr0 = !step && (q_cnt != 4'd0) && q_err[0];
  wire        de_ferr  = de_ferr0 || (!step && |(q_err & de_wmask & de_qmask));
  wire        de_aerr  = !step && q_aerr;
  logic [3:0] de_fk;                          // the first faulted word
  always_comb begin
    de_fk = 4'd0;
    for (int k = 11; k >= 0; k--) if (q_err[k] && de_wmask[k] && de_qmask[k]) de_fk = 4'(k);
  end
  wire [31:0] de_fa = q_pc + {27'h0, de_fk, 1'b0};
  rd68041_pkg::uop_t de_u;                    // what decode issues
  always_comb begin
    de_u = dec_u;
    if (de_aerr || de_ferr) begin
      de_u = '0;
      de_u.valid = 1'b1;
      de_u.last  = 1'b1;
      de_u.pc    = q_pc;
      de_u.len   = 5'd2;
      if (de_aerr) begin
        de_u.xvec = 8'd3;
        de_u.imm  = {q_pc[31:1], 1'b0};
      end else begin
        de_u.xvec = 8'd2;
        de_u.imm  = de_fa & 32'hFFFF_FFFC;
        de_u.tgt  = {16'h0, 4'h0, 1'b0, q_atc[de_fk], 1'b0, 1'b1, 1'b0, 2'b00, rd68041_pkg::TT_NORMAL,
                     sup ? rd68041_pkg::TM_SUP_CODE : rd68041_pkg::TM_USER_CODE};
      end
    end
  end
  wire de_more = dec_more && !(de_aerr || de_ferr);

  // Nothing issues while a serialising micro-operation is in flight, nor while
  // exception processing runs.
  wire ser_fly = (ea_u.valid && ea_u.ser) || (of_u.valid && of_u.ser) ||
                 (ex_u.valid && ex_u.ser) || (w_u.valid && w_u.ser);

  logic ea_stall, of_stall, ex_stall, w_stall;
  wire  running  = (rs == R_RUN) && !halted;
  // FSAVE, FRESTORE's second step and a dynamic FMOVEM wait for the pipeline
  // behind decode to empty: the FPU's state, Dn, settled.
  wire  pipe_quiet = !ea_u.valid && !of_u.valid && !ex_u.valid && !w_u.valid;
  wire  de_issue = running && (de_avail || de_ferr || de_aerr) && !de_halted && !ea_stall &&
                   !(dec_qwait && !pipe_quiet) &&
                   !redirect && (xs == XS_IDLE) && !ser_fly && !x_flush;
  assign q_pop = de_issue && !(de_aerr || de_ferr) ? (step ? dec_pop : 4'd1 + dec_pop) : 4'd0;

  // The exception sequencer's micro-operations, in decode's place.
  logic [rd68041_pkg::UOP_W-1:0] exc_v;
  logic                          exc_valid;
  rd68041_pkg::uop_t             exc_u;
  assign exc_u = exc_v;
  wire  xs_take = running && (xs == XS_SEQ) && exc_valid && !ea_stall && !redirect && !x_flush;

  // ===========================================================================
  // The register file
  // ===========================================================================
  rd68041_regfile u_rf (
      .clk(clk), .rst_n(rst_n),
      .ra(ra), .rb(rb), .rc(rc), .rd(rd), .re(re),
      .qa(qa), .qb(qb), .qc(qc), .qd(qd), .qe(qe), .q15(q15),
      .o0(rf_o0), .o1(rf_o1), .o2(rf_o2),
      .we0(we0), .wa0(wa0), .wd0(wd0), .wm0(wm0),
      .we1(we1), .wa1(wa1), .wd1(wd1),
      .we2(we2), .wa2(wa2), .wd2(wd2));

  function automatic logic [3:0] lanes_of(input logic [1:0] sz);
    lanes_of = (sz == rd68041_pkg::SZ_B) ? 4'b0001 :
               (sz == rd68041_pkg::SZ_W) ? 4'b0011 : 4'b1111;
  endfunction
  // A register as a byte or word write leaves it: only the low bytes change.
  function automatic logic [31:0] merge(input logic [31:8] old, input logic [31:0] nw,
                                        input logic [1:0] sz);
    case (sz)
      rd68041_pkg::SZ_B: merge = {old[31:8], nw[7:0]};
      rd68041_pkg::SZ_W: merge = {old[31:16], nw[15:0]};
      default:           merge = nw;
    endcase
  endfunction

  // W's own writes, as a forwarding source: the result, else the update.
  function automatic logic w_has(input rd68041_pkg::uop_t u, input logic [4:0] r);
    w_has = u.valid && ((u.wr && u.wr_reg == r) ||
                        (u.ea_upd != rd68041_pkg::U_NONE && {2'b01, u.ea_an} == r));
  endfunction

  // ===========================================================================
  // <ea> calculate
  // ===========================================================================
  wire [4:0] base_r  = (ea_u.ea_base == rd68041_pkg::B_REG) ? ea_u.ea_reg : {2'b01, ea_u.ea_an};
  // A bit field's accesses read T3, its parameters, on the index port.
  wire [4:0] index_r = (ea_u.bfa != 2'd0) ? 5'd19 : {1'b0, ea_u.ea_xn};
  wire need_base  = ea_u.valid && ea_u.ea_en &&
                    (ea_u.ea_base == rd68041_pkg::B_AN || ea_u.ea_base == rd68041_pkg::B_REG);
  wire need_index = ea_u.valid && ea_u.ea_en && (ea_u.ea_xen || ea_u.bfa != 2'd0);

  // How many bytes a bit field in memory covers, from OP_BFPRM's parameters
  // (the width in 18-13, the bit offset into the first byte in 10-8): one to
  // five. They are accessed as a byte, a word, a word and a byte, a long word,
  // or a long word and a byte -- none outside the field, which could be in
  // another page or a device (UM 7.3 leaves the split to the part).
  function automatic logic [2:0] bf_bytes(input logic [2:0] boff, input logic [5:0] width);
    bf_bytes = 3'(({3'h0, boff} + width + 6'd7) >> 3);
  endfunction

  // A result an older micro-operation has not computed yet.
  function automatic logic pending(input rd68041_pkg::uop_t u, input logic [4:0] r);
    pending = u.valid && ((u.wr && (u.wr_reg == r)) || (u.wr2 && (u.wr2_reg == r)));
  endfunction
  function automatic logic upd_of(input rd68041_pkg::uop_t u, input logic [4:0] r);
    upd_of = u.valid && (u.ea_upd != rd68041_pkg::U_NONE) && ({2'b01, u.ea_an} == r);
  endfunction

  // The value of r as <ea> calculate must see it.
  function automatic logic [31:0] ea_val(input logic [4:0] r, input logic [31:0] rf,
                                         input rd68041_pkg::uop_t o, input logic [31:0] o_an,
                                         input rd68041_pkg::uop_t x, input logic [31:0] x_an,
                                         input rd68041_pkg::uop_t w, input logic w_ok,
                                         input logic [31:0] w_a,
                                         input logic [31:0] w_r, input logic [31:0] w_r2);
    if (upd_of(o, r))                                     ea_val = o_an;
    else if (upd_of(x, r))                                ea_val = x_an;
    else if (w.valid && w.wr && w_ok && w.wr_reg == r)    ea_val = merge(rf[31:8], w_r, w.wr_sz);
    else if (w.valid && w.wr2 && w_ok && w.wr2_reg == r)  ea_val = w_r2;
    else if (upd_of(w, r))                                ea_val = w_a;
    else                                                  ea_val = rf;
  endfunction

  // A dynamic FMOVEM's Dn, read for decode while <ea> calculate is empty.
  assign ra = (!ea_u.valid && de_dyn) ? {2'b00, cmd_w[6:4]} : base_r;
  assign rb = index_r;

  wire ea_wait = (need_base  && (pending(of_u, base_r)  || pending(ex_u, base_r))) ||
                 (need_index && (pending(of_u, index_r) || pending(ex_u, index_r)));

  wire [31:0] base_v  = ea_val(base_r,  qa, of_u, of_an, ex_u, ex_an, w_u, !w_nowr, w_an, w_res, w_res2);
  wire [31:0] index_v = ea_val(index_r, qb, of_u, of_an, ex_u, ex_an, w_u, !w_nowr, w_an, w_res, w_res2);

  logic [31:0] agu_base, agu_disp, agu_ea;
  always_comb begin
    case (ea_u.ea_base)
      rd68041_pkg::B_AN,
      rd68041_pkg::B_REG: agu_base = base_v;
      rd68041_pkg::B_PC: agu_base = ea_u.pc;
      default:           agu_base = 32'h0;
    endcase
    agu_disp = (ea_u.ea_upd == rd68041_pkg::U_PRE) ? -{28'h0, ea_u.ea_step} : ea_u.ea_disp;
    if (ea_u.bfa == 2'd2) agu_disp = (bf_bytes(index_v[10:8], index_v[18:13]) == 3'd3) ? 32'd2 : 32'd4;
  end

  // The micro-operation as it goes on to <ea> fetch: a bit field's accesses
  // sized to its bytes, its second dropped when there is no last byte.
  rd68041_pkg::uop_t ea_out;
  always_comb begin
    logic [2:0] n;
    ea_out = ea_u;
    n = bf_bytes(index_v[10:8], index_v[18:13]);
    if (ea_u.bfa == 2'd1)
      ea_out.sz = (n == 3'd1) ? rd68041_pkg::SZ_B :
                  (n <= 3'd3) ? rd68041_pkg::SZ_W : rd68041_pkg::SZ_L;
    if (ea_u.bfa == 2'd2 && n != 3'd3 && n != 3'd5) begin
      ea_out.ld = 1'b0;
      ea_out.st = 1'b0;
    end
  end

  rd68041_agu u_agu (.base(agu_base), .index(ea_u.ea_xen ? index_v : 32'h0),
                     .index_w(!ea_u.ea_xl), .scale(ea_u.ea_scale), .disp(agu_disp),
                     .ea(agu_ea));

  // (An)+ leaves An + step; -(An) the address itself.
  wire [31:0] ea_an_new = (ea_u.ea_upd == rd68041_pkg::U_POST) ? base_v + {28'h0, ea_u.ea_step}
                                                               : agu_ea;

  assign ea_stall = of_stall || ea_wait;

  // ===========================================================================
  // <ea> fetch
  // ===========================================================================
  logic        ld_done, ld_err, st_ready, st_empty, ld_cancel;
  logic [31:0] ld_data;
  logic        ld_valid;
  logic        ld_take, ld_fast;
  logic [31:0] ld_fdata;
  logic [31:0] ld_addr;
  logic  [1:0] ld_sz;
  logic [31:0] reset_v;

  // A read waits for any older write it overlaps that is not in the data
  // memory unit's queue yet -- one in EX or in W; the unit checks its queue
  // itself (UM 7.7).
  function automatic logic [2:0] nb(input logic [1:0] sz);
    nb = (sz == rd68041_pkg::SZ_B) ? 3'd1 : (sz == rd68041_pkg::SZ_W) ? 3'd2 : 3'd4;
  endfunction
  // On the page offset, as the data memory unit compares its queue's: the
  // addresses are logical, and two mappings of one page share it
  // (rd68041_dmu, `overlap`; doc/bugs-found.md M13).
  function automatic logic touches(input logic [11:0] a0, input logic [1:0] s0,
                                   input logic [11:0] a1, input logic [1:0] s1);
    logic [11:2] b0, b1, e0, e1;
    b0 = a0[11:2];
    b1 = a1[11:2];
    e0 = b0 + {9'h0, ({1'b0, a0[1:0]} + nb(s0)) > 3'd4};
    e1 = b1 + {9'h0, ({1'b0, a1[1:0]} + nb(s1)) > 3'd4};
    touches = (b0 == b1) || (b0 == e1) || (e0 == b1) || (e0 == e1);
  endfunction
  // A MOVE16 write covers its whole line; a MOVE16 read waits for every write
  // (the data memory unit holds the line in between).
  function automatic logic st_hits(input rd68041_pkg::uop_t x, input logic [11:0] x_a,
                                   input rd68041_pkg::uop_t w, input logic [11:0] w_a);
    st_hits = w.valid && w.st &&
              (x.m16 || (w.m16 ? (x_a[11:4] == w_a[11:4]) : touches(x_a, x.sz, w_a, w.sz)));
  endfunction
  wire st_ahead = st_hits(of_u, of_ea[11:0], ex_u, ex_ea[11:0]) ||
                  st_hits(of_u, of_ea[11:0], w_u, w_ea[11:0]);

  // Exception processing's reads -- the vector -- wait for every write before
  // them, the frame's: UM 8.1 fetches the vector last, and a double bus fault
  // on it leaves memory as it then is (UM 8.2.1).
  wire xs_hold  = of_u.xs && ((ex_u.valid && ex_u.st) || (w_u.valid && w_u.st) || !st_empty);
  // A write is translated here, so that its access fault is the
  // instruction's, taken before it writes anything -- the frame then has no
  // write-back pending, and the instruction is restarted (UM 8.4.6;
  // doc/divergences.md): the data unit's probe, when translation or a data
  // TTR is on. A locked write's page was checked by its read (ld_wr). One of
  // exception processing's writes that faults is a double bus fault (w_dbl).
  wire mmu_on   = tc[15] || dtt0[15] || dtt1[15];
  wire of_prb   = of_u.st && !of_u.ld && !of_u.lk && mmu_on;
  wire of_mem   = of_u.ld || of_prb;
  wire of_wants = of_u.valid && of_mem && !of_got && running &&
                  (of_prb || (!st_ahead && !xs_hold));
  // A read released to EX before its answer (rd68041_dmu, ld_take) that has
  // not had it: EX waits, and OF asks for nothing -- one read is outstanding.
  wire ex_wait  = ex_u.valid && ex_mw && !ex_mok;
  assign ld_valid = (rs == R_SSP || rs == R_PC) || (of_wants && !ex_wait);
  assign ld_addr  = (rs == R_SSP) ? 32'h0 : (rs == R_PC) ? 32'h4 : of_ea;
  assign ld_sz    = (rs == R_RUN) ? of_u.sz : rd68041_pkg::SZ_L;

  // Nothing older than <ea> fetch's instruction is left in EX or W -- the
  // uops there, if any, are its own (one instruction's share its address) --
  // so no interrupt can be taken, and no branch resolved, between its read
  // and its end: what a cache-inhibited read needs (rd68041_dmu; UM 4.3.2).
  // Exception processing's reads and the reset's go at once.
  wire of_oldest = (rs != R_RUN) || of_u.xs ||
                   ((!ex_u.valid || ex_u.pc == of_u.pc) && (!w_u.valid || w_u.pc == of_u.pc));
  // ... and while EX waits for its released read, which the data unit's slot
  // may hold: nothing older than EX's.
  wire ld_oldest = ex_wait ? (!w_u.valid || w_u.pc == ex_u.pc) : of_oldest;

  // The answer is OF's when it is not for the read released last clock or
  // still pending in EX.
  wire of_ans   = ld_done && !of_rel_q && !ex_wait;
  // OF releases a plain read to EX when the data unit takes it this clock
  // (ld_take, from registers): EX then has its answer from ld_fast/ld_fdata
  // on the same edge, or waits for ld_done. Not a locked, alternate, MOVE16
  // or acknowledge read, exception processing's, or an FP operation's (the
  // FPU starts on its operand in EX).
  wire of_relk  = of_u.ld && !of_u.lk && !of_u.alt && !of_u.ack && !of_u.m16 && !of_u.xs &&
                  !of_u.halt && (of_u.fpk == rd68041_fpu_pkg::FK_NONE) && (rs == R_RUN);
  wire of_rel   = of_relk && of_wants && ld_take && !ex_wait && !of_ans;
  wire of_ready = !of_mem || of_got || of_ans || of_rel;
  assign of_stall = ex_stall || (of_u.valid && !of_ready);

  // A read that ended in a bus error is an access fault (UM 8.2.1), taken when
  // the micro-operation reaches write-back: FA its address, the SSW its
  // attributes (UM 8.4.6.2). The read of a locked read-modify-write counts as
  // a write ("the data memory unit considers the read portion of
  // read-modify-write transfers ... a write", UM 8.4.6.7). An acknowledge
  // cycle's bus error is an answer, not a fault.
  logic        ld_avec, ld_atc, ld_ma;
  wire         of_err = of_got ? of_er : ld_err;
  wire         of_atc = of_got ? of_at : ld_atc;
  wire         of_mab = of_got ? of_ma : ld_ma;
  wire         of_f   = of_u.valid && of_mem && !of_u.ack && of_err && !of_rel;
  // An access fault (ATC set) on a read checked as a write -- a probe, a
  // read-modify-write's read -- is a write's (RW 0); a bus error on an
  // unlocked one's read is the read's. A MOVES's space of data or program is
  // TM's data one (UM 3.5.2, Table 3-2).
  // An access fault's SSW (UM 8.4.6.2).
  function automatic logic [15:0] acc_ssw(input rd68041_pkg::uop_t u, input logic prb,
                                          input logic atc, input logic mab, input logic s,
                                          input logic [2:0] sfc_, input logic [2:0] dfc_);
    logic       rw;
    logic [2:0] afc, fc;
    logic [1:0] siz;
    rw  = !(u.lk || prb || (atc && u.st));
    afc = prb ? dfc_ : sfc_;
    fc  = (afc[1:0] == 2'b10) ? {afc[2], 2'b01} : afc;
    siz = (u.sz == rd68041_pkg::SZ_B) ? rd68041_pkg::SIZ_BYTE :
          (u.sz == rd68041_pkg::SZ_W) ? rd68041_pkg::SIZ_WORD : rd68041_pkg::SIZ_LONG;
    acc_ssw = {4'h0, mab, atc, u.lk, rw, 1'b0, siz,
               (u.alt && (fc[1:0] != 2'b01)) ? rd68041_pkg::TT_ALTFC : rd68041_pkg::TT_NORMAL,
               u.alt ? fc : (s ? rd68041_pkg::TM_SUP_DATA : rd68041_pkg::TM_USER_DATA)};
  endfunction
  wire [15:0] of_ssw = acc_ssw(of_u, of_prb, of_atc, of_mab, sup, sfc, dfc);
  wire [15:0] ex_ssw = acc_ssw(ex_u, 1'b0, ld_atc, ld_ma, sup, sfc, dfc);
  // The micro-operation as it goes on to EX; EX's released read, faulted.
  rd68041_pkg::uop_t of_out, ex_flt;
  always_comb begin
    of_out = of_u;
    if (of_f) begin
      of_out.xvec  = 8'd2;
      of_out.xnext = 1'b0;
      of_out.imm   = of_ea;
      of_out.tgt   = {16'h0, of_ssw};
    end
    ex_flt       = ex_u;
    ex_flt.xvec  = 8'd2;
    ex_flt.xnext = 1'b0;
    ex_flt.imm   = ex_ea;
    ex_flt.tgt   = {16'h0, ex_ssw};
  end

  // ===========================================================================
  // Execute
  // ===========================================================================
  assign rc = ex_u.a_reg;
  assign rd = ex_u.b_reg;
  assign re = ex_u.wr2_reg;                  // the high half of a 64-bit dividend

  function automatic logic [31:0] ex_reg(input logic [4:0] r, input logic [31:0] rf,
                                         input rd68041_pkg::uop_t w, input logic w_ok,
                                         input logic [31:0] w_a,
                                         input logic [31:0] w_r, input logic [31:0] w_r2);
    if (w.valid && w.wr && w_ok && w.wr_reg == r)        ex_reg = merge(rf[31:8], w_r, w.wr_sz);
    else if (w.valid && w.wr2 && w_ok && w.wr2_reg == r) ex_reg = w_r2;
    else if (upd_of(w, r))                        ex_reg = w_a;
    else                                          ex_reg = rf;
  endfunction

  function automatic logic [31:0] pick(input logic [1:0] sel, input logic [31:0] rv,
                                       input logic [31:0] imm, input logic [31:0] md,
                                       input logic [31:0] ea);
    case (sel)
      rd68041_pkg::S_REG: pick = rv;
      rd68041_pkg::S_IMM: pick = imm;
      rd68041_pkg::S_MEM: pick = md;
      default:            pick = ea;
    endcase
  endfunction

  // A control register as EX reads it: W's write to it first.
  function automatic logic [31:0] cr_val(input logic [3:0] c, input rd68041_pkg::uop_t w,
                                         input logic [31:0] w_r, input logic [31:0] cur);
    if (w.valid && w.crw && w.cr == c) cr_val = w_r;
    else                               cr_val = cur;
  endfunction

  logic [31:0] cr_cur;
  always_comb begin
    case (ex_u.cr)
      rd68041_pkg::CR_SFC:   cr_cur = {29'h0, sfc};
      rd68041_pkg::CR_DFC:   cr_cur = {29'h0, dfc};
      rd68041_pkg::CR_CACR:  cr_cur = cacr;
      rd68041_pkg::CR_TC:    cr_cur = tc;
      rd68041_pkg::CR_ITT0:  cr_cur = itt0;
      rd68041_pkg::CR_ITT1:  cr_cur = itt1;
      rd68041_pkg::CR_DTT0:  cr_cur = dtt0;
      rd68041_pkg::CR_DTT1:  cr_cur = dtt1;
      rd68041_pkg::CR_USP:   cr_cur = usp;
      rd68041_pkg::CR_VBR:   cr_cur = vbr;
      rd68041_pkg::CR_MSP:   cr_cur = msp;
      rd68041_pkg::CR_ISP:   cr_cur = isp;
      rd68041_pkg::CR_MMUSR: cr_cur = mmusr;
      rd68041_pkg::CR_URP:   cr_cur = urp;
      rd68041_pkg::CR_SRP:   cr_cur = srp;
      default:               cr_cur = {16'h0, sr_sys, 3'b000, ccr_x};
    endcase
  end

  // A register the micro-operation's own (An)+ or -(An) updates reads as
  // updated -- ADDA.L (A7)+,A7 adds to A7 + 4: PRM 2.2.4-2.2.5, the address
  // "is used" and then moved -- unless the micro-operation stores it, as
  // MOVE An,(An)+ does, which stores An as it was (doc/bugs-found.md M13).
  function automatic logic [31:0] ex_own(input rd68041_pkg::uop_t x, input logic [31:0] x_an,
                                         input logic [4:0] r, input logic [31:0] v);
    ex_own = (!x.st && upd_of(x, r)) ? x_an : v;
  endfunction
  wire [31:0] b_reg_v = ex_own(ex_u, ex_an, ex_u.b_reg,
                               ex_reg(ex_u.b_reg, qd, w_u, !w_nowr, w_an, w_res, w_res2));
  wire [31:0] a_raw = pick(ex_u.a_sel,
                           ex_own(ex_u, ex_an, ex_u.a_reg,
                                  ex_reg(ex_u.a_reg, qc, w_u, !w_nowr, w_an, w_res, w_res2)),
                           ex_u.imm, ex_md, ex_ea);
  wire [31:0] b_raw = ex_u.crr ? cr_val(ex_u.cr, w_u, w_res, cr_cur)
                               : pick(ex_u.b_sel, b_reg_v, ex_u.imm, ex_md, ex_ea);
  wire [31:0] c_v   = ex_own(ex_u, ex_an, ex_u.wr2_reg,
                             ex_reg(ex_u.wr2_reg, qe, w_u, !w_nowr, w_an, w_res, w_res2));
  wire [31:0] av = ex_u.a_sxw ? {{16{a_raw[15]}}, a_raw[15:0]} : a_raw;
  wire [31:0] bv = ex_u.b_sxw ? {{16{b_raw[15]}}, b_raw[15:0]} : b_raw;

  // --- the shifter (fn: {left, kind}) ---------------------------------------
  logic [31:0] sh_res;
  logic        sh_c, sh_v, sh_x, sh_xw;
  rd68041_shifter u_shift (
      .op(bv), .count(av[5:0]), .size(ex_u.sz), .kind(ex_u.fn[1:0]), .left(ex_u.fn[2]),
      .x_in(ccr_x[4]), .res(sh_res), .c_out(sh_c), .v_out(sh_v), .x_out(sh_x),
      .x_write(sh_xw));

  // --- the bit-field unit (OP_BF) -----------------------------------------------
  // C holds the parameters OP_BFPRM made -- width in 18-13, offset 12-8 -- and
  // for memory the last byte in 7-0; B the register or the first access, as
  // EA sized them (bf_bytes). bf_win is the five bytes from the field's first,
  // those it does not cover zero.
  logic [31:0] bf_field, bf_sx, bf_ins, bf_mreg;
  logic [39:0] bf_mmem, bf_win;
  logic [31:0] bf_m1, bf_m2;                      // the merged bytes, as written back
  always_comb begin
    case (bf_bytes(c_v[10:8], c_v[18:13]))
      3'd1:    bf_win = {bv[7:0], 32'h0};
      3'd2:    bf_win = {bv[15:0], 24'h0};
      3'd3:    bf_win = {bv[15:0], c_v[7:0], 16'h0};
      3'd4:    bf_win = {bv, 8'h0};
      default: bf_win = {bv, c_v[7:0]};
    endcase
  end
  always_comb begin
    case (bf_bytes(c_v[10:8], c_v[18:13]))
      3'd1:    begin bf_m1 = {24'h0, bf_mmem[39:32]}; bf_m2 = 32'h0;                   end
      3'd2:    begin bf_m1 = {16'h0, bf_mmem[39:24]}; bf_m2 = 32'h0;                   end
      3'd3:    begin bf_m1 = {16'h0, bf_mmem[39:24]}; bf_m2 = {24'h0, bf_mmem[23:16]}; end
      3'd4:    begin bf_m1 = bf_mmem[39:8];           bf_m2 = 32'h0;                   end
      default: begin bf_m1 = bf_mmem[39:8];           bf_m2 = {24'h0, bf_mmem[7:0]};   end
    endcase
  end
  logic        bf_msb, bf_zero;
  logic  [5:0] bf_ffo;
  always_comb begin
    case (ex_u.fn[2:0])
      3'd2:    bf_ins = ~bf_field;                     // BFCHG
      3'd4:    bf_ins = 32'h0;                         // BFCLR
      3'd6:    bf_ins = 32'hFFFF_FFFF;                 // BFSET
      default: bf_ins = av;                            // BFINS
    endcase
  end
  rd68041_bitfield u_bf (
      .is_reg(ex_u.fn[3]), .reg_data(bv), .mem_data(bf_win),
      .roff(c_v[12:8]), .boff(c_v[10:8]), .width(c_v[18:13]), .ins(bf_ins),
      .field(bf_field), .sxfield(bf_sx), .msb(bf_msb), .zero(bf_zero), .ffo(bf_ffo),
      .merged_reg(bf_mreg), .merged_mem(bf_mmem));

  // --- the multiplier (fn: {64-bit, long, signed}) ------------------------------
  // 32 x 32 to 64, signed or not, the word forms on sign- or zero-extended
  // halves (PRM 4, MULS and MULU).
  logic [63:0] prod;
  always_comb begin
    logic [63:0] pa, pb;
    if (ex_u.fn[1]) begin
      pa = ex_u.fn[0] ? {{32{av[31]}}, av} : {32'h0, av};
      pb = ex_u.fn[0] ? {{32{bv[31]}}, bv} : {32'h0, bv};
    end else begin
      pa = ex_u.fn[0] ? {{48{av[15]}}, av[15:0]} : {48'h0, av[15:0]};
      pb = ex_u.fn[0] ? {{48{bv[15]}}, bv[15:0]} : {48'h0, bv[15:0]};
    end
    prod = pa * pb;
  end

  // --- the divider (fn: {64-bit dividend, long, signed}) --------------------------
  // B the dividend (Dq), A the divisor, c_v the dividend's high half (Dr).
  logic        div_start, div_busy, div_zero, div_ovf, div_qneg;
  logic [31:0] div_q, div_r, div_qmag;
  logic [63:0] div_n;
  logic [31:0] div_d;
  always_comb begin
    if (ex_u.fn[2])      div_n = {c_v, bv};
    else if (ex_u.fn[0]) div_n = {{32{bv[31]}}, bv};
    else                 div_n = {32'h0, bv};
    if (ex_u.fn[1])      div_d = av;
    else                 div_d = ex_u.fn[0] ? {{16{av[15]}}, av[15:0]} : {16'h0, av[15:0]};
  end
  rd68041_divider u_div (
      .clk(clk), .rst_n(rst_n), .start(div_start), .dividend(div_n), .divisor(div_d),
      .is_signed(ex_u.fn[0]), .busy(div_busy), .quotient(div_q), .remainder(div_r),
      .div_zero(div_zero), .overflow(div_ovf), .q_mag(div_qmag), .q_neg(div_qneg));

  // A micro-operation that already takes an exception (decode's, or a read
  // that faulted) executes nothing that leaves EX: no branch, no division.
  wire ex_live   = ex_u.valid && (ex_u.xvec == 8'd0);

  // --- the FPU (doc/fpu.md) ------------------------------------------------------
  // An FP micro-operation in EX: the FPU evaluates it (EX waits, fp_wait), and
  // its commit record is applied when it leaves write-back.
  logic        fp_wait, fp_ct, fp_xn, fp_xc, fp_commit;
  logic [31:0] fp_res, fp_xea;
  logic  [7:0] fp_xv;
  logic  [3:0] fp_xf;
  wire         ex_fp   = ex_live && (ex_u.fpk != rd68041_fpu_pkg::FK_NONE) && running;
  // The micro-operation's <ea> is a memory one: it accesses it, or it is the
  // operation of an FMOVE out, which computes its destination for a frame.
  wire         fp_eam  = ex_u.ld || ex_u.st || (ex_u.fpk == rd68041_fpu_pkg::FK_OP && ex_u.ea_en);
  // The MC68LC040 has none (UM A.5): nothing comes from it, and decode makes
  // every FP instruction an exception (rd68041_decode).
  if (FPU != 0) begin : g_fpu
    rd68041_fpu u_fpu (
        .clk(clk), .rst_n(rst_n), .crst(crst),
        .x_v(ex_fp), .x_adv(ex_u.valid && !ex_stall), .x_k(ex_u.fpk), .x_chk(ex_u.fchk),
        .x_last(ex_u.last), .x_cmd(ex_u.fcmd), .x_idx(ex_u.fidx), .x_reg(ex_u.freg),
        .x_b(bv), .x_pc(ex_u.pc),
        // The frame's address field: the operand's first byte (each micro-op
        // moves the long word fidx of it).
        .x_ea(ex_ea - {25'h0, ex_u.fidx, 2'b00}), .x_eam(fp_eam),
        .x_wait(fp_wait), .x_res(fp_res), .x_ct(fp_ct), .x_xv(fp_xv), .x_xn(fp_xn),
        .x_xf(fp_xf), .x_xea(fp_xea), .x_xc(fp_xc),
        .w_commit(fp_commit), .flush(x_flush),
        .sv_n(fp_svn), .rs_n(fp_rsn));
  end else begin : g_nofpu
    assign fp_wait = 1'b0;
    assign fp_res  = 32'h0;
    assign fp_ct   = 1'b0;
    assign fp_xv   = 8'd0;
    assign fp_xn   = 1'b0;
    assign fp_xf   = 4'h0;
    assign fp_xea  = 32'h0;
    assign fp_xc   = 1'b0;
    assign fp_svn  = 5'd0;
    assign fp_rsn  = 5'd0;
    logic unused_fp;
    assign unused_fp = &{1'b0, ex_fp, fp_eam, fp_commit};
  end
  // A condition: the FP predicate for FBcc, FScc, FDBcc, FTRAPcc.
  wire ex_ct = (ex_u.fpk == rd68041_fpu_pkg::FK_COND) ? fp_ct : cond_true(ex_u.cond, ccr_x[3:0]);
  // The FP result EX returns: FMOVE out's long words, the control registers,
  // FMOVEM's, FSAVE's.
  wire ex_fpres = (ex_u.fpk == rd68041_fpu_pkg::FK_OUT) || (ex_u.fpk == rd68041_fpu_pkg::FK_CRR) ||
                  (ex_u.fpk == rd68041_fpu_pkg::FK_RGR) || (ex_u.fpk == rd68041_fpu_pkg::FK_SAVE);
  wire ex_is_div = ex_live && (ex_u.op == rd68041_pkg::OP_DIV);
  assign div_start = ex_is_div && !div_go && running && !ex_wait;
  // div_go: started; div_done: the answer is there (the clock after busy falls).
  wire div_wait = ex_is_div && !(div_go && !div_busy && div_done);

  // The quotient's fit (PRM 4): sixteen bits for the word forms, thirty-two
  // signed or unsigned for the long.
  // The divider's overflow is only the unsigned one: a signed quotient's
  // magnitude must also fit, up to $80000000 when it is negative.
  wire div_fits = (ex_u.fn[1]
                   ? (!ex_u.fn[0] || (div_qneg ? (div_qmag <= 32'h8000_0000) : !div_qmag[31]))
                   : (ex_u.fn[0] ? (div_qneg ? (div_qmag <= 32'h8000) : (div_qmag <= 32'h7FFF))
                                 : (div_qmag <= 32'hFFFF))) && !div_ovf;

  // --- the operation, and the condition codes at its size (PRM 3.4) ------------
  logic [31:0] res, res2, st_val;              // the result, the second, the store's data
  logic  [4:0] ccr_n;                       // X N Z V C
  logic        ex_nowr;                     // the register results are not written
  logic        ex_nost;                     // nor the store (CAS2's conditional ones)
  logic        ex_trap;                     // a conditional trap is taken
  logic        rte_bad;                     // RTE: a format this processor does not take
  logic        bcd_c, cmp2_out;
  // The ALU's width: the operation's, except that a sign-extended word source
  // (ADDA, SUBA, CMPA .W) is worked on at 32 bits; the load stays a word.
  wire [1:0] alu_sz = ex_u.a_sxw ? rd68041_pkg::SZ_L : ex_u.sz;
  always_comb begin
    logic [32:0] s;
    logic [31:0] m, p, q;
    logic        cin, sub, an, bn, rn, cy, vf, xop, z;
    logic  [4:0] msb;
    logic  [4:0] bitn;
    m   = (alu_sz == rd68041_pkg::SZ_B) ? 32'h0000_00FF :
          (alu_sz == rd68041_pkg::SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
    msb = (alu_sz == rd68041_pkg::SZ_B) ? 5'd7 : (alu_sz == rd68041_pkg::SZ_W) ? 5'd15 : 5'd31;

    // The adder: p + q + cin, or p - q - cin.
    p = bv; q = av; cin = 1'b0; sub = 1'b0; xop = 1'b0;
    case (ex_u.op)
      rd68041_pkg::OP_SUB, rd68041_pkg::OP_SUBA, rd68041_pkg::OP_CMP,
      rd68041_pkg::OP_CAS: sub = 1'b1;
      rd68041_pkg::OP_ADDX: begin cin = ccr_x[4]; xop = 1'b1; end
      rd68041_pkg::OP_SUBX: begin cin = ccr_x[4]; sub = 1'b1; xop = 1'b1; end
      rd68041_pkg::OP_NEG:  begin p = 32'h0; q = bv; sub = 1'b1; end
      rd68041_pkg::OP_NEGX: begin p = 32'h0; q = bv; sub = 1'b1; cin = ccr_x[4]; xop = 1'b1; end
      default: ;
    endcase
    s = sub ? ({1'b0, p} - {1'b0, q} - {32'h0, cin}) : ({1'b0, p} + {1'b0, q} + {32'h0, cin});

    res   = bv;
    res2  = 32'h0;
    ex_nowr = 1'b0;
    ex_nost = 1'b0;
    ex_trap = 1'b0;
    rte_bad = 1'b0;
    bcd_c   = 1'b0;
    cmp2_out = 1'b0;
    bitn  = ex_u.sz == rd68041_pkg::SZ_L ? av[4:0] : {2'b00, av[2:0]};
    case (ex_u.op)
      rd68041_pkg::OP_PASSB: res = bv;
      rd68041_pkg::OP_PASSA: res = av;
      rd68041_pkg::OP_ADD, rd68041_pkg::OP_ADDA, rd68041_pkg::OP_SUB, rd68041_pkg::OP_SUBA,
      rd68041_pkg::OP_CMP, rd68041_pkg::OP_ADDX, rd68041_pkg::OP_SUBX,
      rd68041_pkg::OP_NEG, rd68041_pkg::OP_NEGX: res = s[31:0];
      rd68041_pkg::OP_AND:   res = bv & av;
      rd68041_pkg::OP_OR:    res = bv | av;
      rd68041_pkg::OP_EOR:   res = bv ^ av;
      rd68041_pkg::OP_NOT:   res = ~bv;
      rd68041_pkg::OP_EXTW:  res = {bv[31:16], {8{bv[7]}}, bv[7:0]};
      rd68041_pkg::OP_EXTL:  res = {{16{bv[15]}}, bv[15:0]};
      rd68041_pkg::OP_EXTBL: res = {{24{bv[7]}}, bv[7:0]};
      rd68041_pkg::OP_SWAP:  res = {bv[15:0], bv[31:16]};
      rd68041_pkg::OP_CLR:   res = 32'h0;
      rd68041_pkg::OP_SHIFT: res = sh_res;
      rd68041_pkg::OP_SCC:   res = ex_ct ? 32'hFF : 32'h00;
      rd68041_pkg::OP_DBCC: begin
        res = {bv[31:16], bv[15:0] - 16'd1};
        ex_nowr = ex_ct;
      end
      rd68041_pkg::OP_BIT: begin
        case (ex_u.fn[1:0])
          2'd1:    res = bv ^  (32'h1 << bitn);
          2'd2:    res = bv & ~(32'h1 << bitn);
          2'd3:    res = bv |  (32'h1 << bitn);
          default: res = bv;
        endcase
      end
      rd68041_pkg::OP_MUL: begin
        res  = prod[31:0];
        res2 = prod[63:32];
      end
      rd68041_pkg::OP_DIV: begin
        // The word forms: the remainder in the high word, the quotient low.
        res     = ex_u.fn[1] ? div_q : {div_r[15:0], div_q[15:0]};
        res2    = div_r;
        // PRM 4: division by zero traps, the registers left alone.
        ex_nowr = !div_fits || div_zero;
        ex_trap = div_zero;
      end
      rd68041_pkg::OP_EXG: begin
        res  = av;
        res2 = bv;
      end
      rd68041_pkg::OP_LINK: begin
        res  = ex_ea;
        res2 = ex_ea + ex_u.imm;
      end
      rd68041_pkg::OP_BCD: begin
        // PRM 4, ABCD, SBCD, NBCD: decimal, with X; the carry is decimal.
        logic [8:0] lo, v9;
        logic [7:0] dd, ss;
        dd = (ex_u.fn[1:0] == 2'd2) ? 8'h00 : bv[7:0];
        ss = (ex_u.fn[1:0] == 2'd2) ? bv[7:0] : av[7:0];
        if (ex_u.fn[1:0] == 2'd0) begin
          lo = {5'h0, dd[3:0]} + {5'h0, ss[3:0]} + {8'h0, ccr_x[4]};
          v9 = {1'b0, dd[7:4], 4'h0} + {1'b0, ss[7:4], 4'h0} + lo;
          if (lo > 9'd9) v9 = v9 + 9'd6;
          bcd_c = (v9 > 9'h099);
          if (bcd_c) v9 = v9 + 9'h060;
        end else begin
          // The borrow is the binary difference's, or the low digit's
          // correction's: with a digit above 9 the correction can take the
          // difference back over zero ($00 - $FB: $105, then $0FF), and the
          // borrow stands (the cputest corpus: $9F, X and C set).
          lo = {5'h0, dd[3:0]} - {5'h0, ss[3:0]} - {8'h0, ccr_x[4]};
          v9 = {1'b0, dd[7:4], 4'h0} - {1'b0, ss[7:4], 4'h0} + lo;
          bcd_c = v9[8];
          if (lo[8]) v9 = v9 - 9'd6;
          bcd_c = bcd_c || v9[8];
          if (bcd_c) v9 = v9 - 9'h060;
        end
        res = {bv[31:8], v9[7:0]};
      end
      rd68041_pkg::OP_PACK: begin
        // PRM 4: the word plus the adjustment, bits 11-8 and 3-0 packed.
        logic [15:0] t;
        t   = (ex_u.fn[0] ? {bv[7:0], av[7:0]} : bv[15:0]) + ex_u.imm[15:0];
        res = {24'h0, t[11:8], t[3:0]};
      end
      rd68041_pkg::OP_UNPK: begin
        logic [15:0] t;
        t   = {4'h0, bv[7:4], 4'h0, bv[3:0]} + ex_u.imm[15:0];
        res = {16'h0, t};
      end
      rd68041_pkg::OP_CAS: begin
        // Dc loaded with the operand unless they compared equal; the operand
        // written back with Du if equal, or with itself (UM 7.4.5).
        res     = bv;
        ex_nowr = ((bv & m) == (av & m));
      end
      rd68041_pkg::OP_TAS: res = bv | 32'h80;
      rd68041_pkg::OP_BFPRM: begin
        // The offset from Dn is signed and whole; from the extension word,
        // 0 to 31. A width of 0 is 32 (PRM 4).
        logic [31:0] off;
        logic  [4:0] w5;
        off  = ex_u.fn[1] ? av : {27'h0, av[4:0]};
        w5   = ex_u.fn[0] ? bv[4:0] : bv[12:8];
        res  = {13'h0, (w5 == 5'd0) ? 6'd32 : {1'b0, w5}, off[4:0], 8'h00};
        res2 = ex_ea + {{3{off[31]}}, off[31:3]};
      end
      rd68041_pkg::OP_BFPK: res = {av[31:8], bv[7:0]};
      rd68041_pkg::OP_BF: begin
        case (ex_u.fn[2:0])
          3'd1:    res = bf_field;                     // BFEXTU
          3'd3:    res = bf_sx;                        // BFEXTS
          3'd5:    res = av + {26'h0, bf_ffo};          // BFFFO: the offset plus
          default: begin
            res  = ex_u.fn[3] ? bf_mreg : bf_m1;
            res2 = bf_m2;
          end
        endcase
      end
      rd68041_pkg::OP_CMP2: res = bv;
      rd68041_pkg::OP_MOVEP: begin
        // fn[3]: out of B's byte fn[1:0]; else A's byte into it.
        if (ex_u.fn[3]) res = (bv >> {ex_u.fn[1:0], 3'b000}) & 32'hFF;
        else            res = (bv & ~(32'hFF << {ex_u.fn[1:0], 3'b000})) |
                              ({24'h0, av[7:0]} << {ex_u.fn[1:0], 3'b000});
      end
      rd68041_pkg::OP_SR: begin
        // The SR with A, as ANDI, ORI, EORI and MOVE to SR, and STOP, make it.
        logic [15:0] cur, nv;
        cur = {sr_sys, 3'b000, ccr_x};
        case (ex_u.fn[1:0])
          2'd0:    nv = cur & av[15:0];
          2'd1:    nv = cur | av[15:0];
          2'd2:    nv = cur ^ av[15:0];
          default: nv = av[15:0];
        endcase
        res = {16'h0, nv & rd68041_pkg::SR_MASK};
      end
      rd68041_pkg::OP_RTEF: begin
        // UM 8.4: the formats RTE takes, and how far each frame reaches; $4
        // only on the MC68LC040 (UM A.5). The address is A7 + 6.
        logic [31:0] fsz;
        case (bv[15:12])
          4'h0, 4'h1: fsz = 32'd8;
          4'h2, 4'h3: fsz = 32'd12;
          4'h4:       fsz = 32'd16;
          4'h7:       fsz = 32'd60;
          default:    fsz = 32'd0;
        endcase
        rte_bad = (fsz == 32'd0) || (bv[15:12] == 4'h4 && FPU != 0);
        res  = ex_ea - 32'd6 + fsz;
        res2 = bv;
      end
      rd68041_pkg::OP_RTE: res = {16'h0, av[15:0] & rd68041_pkg::SR_MASK};
      rd68041_pkg::OP_IACK: begin
        // UM 7.5.1, Table 7-4: the vector the device put on D7-D0, the
        // autovector 24 + level with AVEC, the spurious interrupt's 24 on a
        // bus error; as its offset.
        logic [7:0] v;
        v   = ex_er ? 8'd24 : ex_av ? (8'd24 + {5'h0, ex_u.fn[2:0]}) : ex_md[7:0];
        res = {22'h0, v, 2'b00};
      end
      default: res = bv;
    endcase
    if (ex_fpres) res = fp_res;
    // The store's data is the result, but for LINK, which stores the register
    // (A7 already decremented, PRM 4's order: LINK A7 pushes the new SP), and
    // CAS, which stores Du if equal.
    st_val = (ex_u.op == rd68041_pkg::OP_LINK) ? ((ex_u.a_reg == 5'd15) ? ex_ea : av) :
             (ex_u.op == rd68041_pkg::OP_CAS)  ? (((bv & m) == (av & m)) ? c_v : bv) : res;
    // UNLK and RTD: the stack pointer past the frame (decode's imm).
    if (ex_u.op == rd68041_pkg::OP_PASSB && ex_u.wr2) res2 = ex_ea + ex_u.imm;

    an = av[msb];
    bn = bv[msb];
    // CAS's flags are its compare's: the difference, not the result.
    rn = (ex_u.op == rd68041_pkg::OP_CAS) ? s[{1'b0, msb}] : res[msb];
    if (ex_u.op == rd68041_pkg::OP_NEG || ex_u.op == rd68041_pkg::OP_NEGX) begin
      an = bv[msb]; bn = 1'b0;
    end
    // The carry or borrow out of the operation's size.
    case (alu_sz)
      rd68041_pkg::SZ_B: cy = sub ? ({1'b0, p[7:0]}  < ({1'b0, q[7:0]} + {8'h0, cin}))
                                  : (({1'b0, p[7:0]}  + {1'b0, q[7:0]}  + {8'h0, cin}) > 9'h0FF);
      rd68041_pkg::SZ_W: cy = sub ? ({1'b0, p[15:0]} < ({1'b0, q[15:0]} + {16'h0, cin}))
                                  : (({1'b0, p[15:0]} + {1'b0, q[15:0]} + {16'h0, cin}) > 17'h0FFFF);
      default:           cy = s[32];
    endcase
    vf = sub ? ((an != bn) && (rn != bn)) : ((an == bn) && (rn != bn));
    z  = (res & m) == 32'h0;

    ccr_n = ccr_x;
    case (ex_u.cc)
      rd68041_pkg::CC_LOGIC: ccr_n = {ccr_x[4], rn, z, 1'b0, 1'b0};
      rd68041_pkg::CC_ADD,
      rd68041_pkg::CC_SUB:   ccr_n = {cy, rn, xop ? (z && ccr_x[2]) : z, vf, cy};
      rd68041_pkg::CC_CMP:   ccr_n = {ccr_x[4], rn, z, vf, cy};
      rd68041_pkg::CC_SET: begin
        case (ex_u.cond)
          4'd0:    ccr_n = ccr_x & av[4:0];
          4'd1:    ccr_n = ccr_x | av[4:0];
          4'd2:    ccr_n = ccr_x ^ av[4:0];
          default: ccr_n = av[4:0];                 // MOVE to CCR, RTR
        endcase
      end
      default: ;
    endcase
    // The operations whose condition codes follow rules of their own.
    case (ex_u.op)
      rd68041_pkg::OP_SHIFT: ccr_n = {sh_xw ? sh_x : ccr_x[4], rn, z, sh_v, sh_c};
      rd68041_pkg::OP_BIT:   ccr_n = {ccr_x[4:3], !bv[bitn], ccr_x[1:0]};
      // PRM 4: X and C the decimal carry, Z cleared if not zero; N and V are
      // undefined, and are left as they were, as UAE040 leaves them
      // (doc/divergences.md).
      rd68041_pkg::OP_BCD:   ccr_n = {bcd_c, ccr_x[3], ccr_x[2] && (res[7:0] == 8'h00), ccr_x[1], bcd_c};
      rd68041_pkg::OP_TAS:   ccr_n = {ccr_x[4], bv[7], bv[7:0] == 8'h00, 1'b0, 1'b0};
      // PRM 4: N the field's top bit, Z the field all zero, V and C cleared;
      // for BFINS, of the field inserted.
      rd68041_pkg::OP_BF: begin
        if (ex_u.fn[2:0] == 3'd7) begin
          logic [31:0] lm;
          lm = (c_v[18:13] == 6'd32) ? 32'hFFFF_FFFF : ((32'h1 << c_v[18:13]) - 32'h1);
          ccr_n = {ccr_x[4], av[c_v[17:13] - 5'd1], (av & lm) == 32'h0, 1'b0, 1'b0};
        end else begin
          ccr_n = {ccr_x[4], bf_msb, bf_zero, 1'b0, 1'b0};
        end
      end
      // CAS: the compare's, of the operand less Dc.
      rd68041_pkg::OP_CAS:   ccr_n = {ccr_x[4], rn, (s[31:0] & m) == 32'h0, vf, cy};
      rd68041_pkg::OP_CMP2: begin
        // Out of bounds when (Rn - lower) > (upper - lower), modulo the size:
        // right for signed and for unsigned bounds alike (PRM 4). An address
        // register is compared whole, the bounds sign-extended.
        logic [31:0] rv, lb, ub;
        logic        wide;
        wide = ex_u.wr2_reg[3] || (ex_u.sz == rd68041_pkg::SZ_L);
        case (ex_u.sz)
          rd68041_pkg::SZ_B: begin
            lb = wide ? {{24{av[7]}}, av[7:0]} : {24'h0, av[7:0]};
            ub = wide ? {{24{bv[7]}}, bv[7:0]} : {24'h0, bv[7:0]};
            rv = wide ? c_v : {24'h0, c_v[7:0]};
          end
          rd68041_pkg::SZ_W: begin
            lb = wide ? {{16{av[15]}}, av[15:0]} : {16'h0, av[15:0]};
            ub = wide ? {{16{bv[15]}}, bv[15:0]} : {16'h0, bv[15:0]};
            rv = wide ? c_v : {16'h0, c_v[15:0]};
          end
          default: begin lb = av; ub = bv; rv = c_v; end
        endcase
        if (wide) cmp2_out = (rv - lb) > (ub - lb);
        else if (ex_u.sz == rd68041_pkg::SZ_B) cmp2_out = (8'(rv - lb)) > (8'(ub - lb));
        else cmp2_out = (16'(rv - lb)) > (16'(ub - lb));
        ccr_n = {ccr_x[4], ccr_x[3], (rv == lb) || (rv == ub), ccr_x[1], cmp2_out};
      end
      // The SR's writers: the CCR is the new SR's.
      rd68041_pkg::OP_SR, rd68041_pkg::OP_RTE: ccr_n = res[4:0];
      rd68041_pkg::OP_MUL: begin
        // PRM 4: N and Z of the whole result; V set when a 32-bit result has
        // lost bits; C cleared.
        if (ex_u.fn[2])
          ccr_n = {ccr_x[4], prod[63], prod == 64'h0, 1'b0, 1'b0};
        else if (ex_u.fn[1])
          ccr_n = {ccr_x[4], prod[31], prod[31:0] == 32'h0,
                   ex_u.fn[0] ? (prod[63:32] != {32{prod[31]}}) : (prod[63:32] != 32'h0), 1'b0};
        else
          ccr_n = {ccr_x[4], prod[31], prod[31:0] == 32'h0, 1'b0, 1'b0};
      end
      rd68041_pkg::OP_DIV: begin
        if (!div_fits)
          ccr_n = {ccr_x[4], ccr_x[3], ccr_x[2], 1'b1, 1'b0};
        else if (ex_u.fn[1])
          ccr_n = {ccr_x[4], div_q[31], div_q == 32'h0, 1'b0, 1'b0};
        else
          ccr_n = {ccr_x[4], div_q[15], div_q[15:0] == 16'h0, 1'b0, 1'b0};
      end
      default: ;
    endcase
    // Traps (M8 takes the exception; until then they halt): TRAPcc and TRAPV on
    // their condition, CHK when the register is out of bounds (PRM 4: "if Dn <
    // 0 or Dn > source", N set for the first and cleared for the second).
    if (ex_u.trap) begin
      if (ex_u.op == rd68041_pkg::OP_CMP2) begin
        ex_trap = cmp2_out;
      end else if (ex_u.op == rd68041_pkg::OP_CMP) begin
        logic lt0, gtb;
        lt0 = bv[msb];
        gtb = !lt0 && ((ex_u.sz == rd68041_pkg::SZ_W) ? ($signed(bv[15:0]) > $signed(av[15:0]))
                                                       : ($signed(bv) > $signed(av)));
        ex_trap = lt0 || gtb;
        ccr_n = {ccr_x[4], lt0 ? 1'b1 : (gtb ? 1'b0 : ccr_x[3]), ccr_x[2:0]};
      end else begin
        ex_trap = ex_ct;
      end
    end
    // A micro-operation only if its condition holds (CAS2): otherwise it
    // writes nothing, stores nothing and leaves the CCR.
    if (ex_u.cx && !cond_true(ex_u.cond, ccr_x[3:0])) begin
      ex_nowr = 1'b1;
      ex_nost = 1'b1;
      ccr_n   = ccr_x;
    end
  end

  // PRM Table 3-19: the conditions, on the CCR as the instructions before left it.
  function automatic logic cond_true(input logic [3:0] c, input logic [3:0] f);
    logic n, z, v, cf;
    n = f[3]; z = f[2]; v = f[1]; cf = f[0];
    case (c)
      4'h0: cond_true = 1'b1;
      4'h1: cond_true = 1'b0;
      4'h2: cond_true = !cf && !z;
      4'h3: cond_true = cf || z;
      4'h4: cond_true = !cf;
      4'h5: cond_true = cf;
      4'h6: cond_true = !z;
      4'h7: cond_true = z;
      4'h8: cond_true = !v;
      4'h9: cond_true = v;
      4'hA: cond_true = !n;
      4'hB: cond_true = n;
      4'hC: cond_true = (n == v);
      4'hD: cond_true = (n != v);
      4'hE: cond_true = !z && (n == v);
      default: cond_true = z || (n != v);
    endcase
  endfunction

  // DBcc branches unless the condition holds or the counter has run out.
  wire dbcc_take = (ex_u.op == rd68041_pkg::OP_DBCC) && !ex_ct && (bv[15:0] != 16'h0000);
  // (An FP micro-operation that takes an exception here does not branch.)
  wire ex_brc   = ex_live && !ex_u.halt && (ex_u.br != rd68041_pkg::BR_NONE) && (fp_xv == 8'd0) &&
                  ((ex_u.op == rd68041_pkg::OP_DBCC) ? dbcc_take :
                   ((ex_u.br != rd68041_pkg::BR_DISP) || ex_ct));
  wire [31:0] ex_target = (ex_u.br == rd68041_pkg::BR_DISP) ? ex_u.tgt :
                          (ex_u.br == rd68041_pkg::BR_MEM)  ? ex_md : bv;
  // UM 8.2.2: a branch to an odd address is an address error, before the
  // prefetch -- and a conditional one with an odd displacement even when it
  // is not taken. The frame's PC is the branch, ADDR the target less bit 0
  // (UM 8.4.6.7).
  wire ex_odd   = ex_live && (ex_u.br != rd68041_pkg::BR_NONE) && ex_target[0] &&
                  (ex_brc || ex_u.br == rd68041_pkg::BR_DISP);
  wire ex_taken = ex_brc && !ex_odd;
  // RTE: the PC from the frame; a throwaway frame runs RTE again (UM 8.4.2).
  wire [31:0] ex_npc    = (ex_u.op == rd68041_pkg::OP_RTE) ?
                            ((c_v[15:12] == 4'h1) ? ex_u.pc : bv) :
                          ex_taken ? ex_target : ex_u.pc + {27'h0, ex_u.len};

  // What EX finds: the traps (UM 8.2.3; vector 5 divide by zero, 6 CHK and
  // CHK2, 7 TRAPcc and TRAPV), taken once the instruction completes; RTE's
  // format error (14, UM 8.2.7) and the address error (3), before.
  logic [7:0] ex_xv;
  logic       ex_xn;
  always_comb begin
    ex_xv = 8'd0;
    ex_xn = 1'b0;
    if (ex_live && ex_trap) begin
      ex_xv = (ex_u.op == rd68041_pkg::OP_DIV) ? 8'd5 :
              (ex_u.op == rd68041_pkg::OP_CMP2 || ex_u.op == rd68041_pkg::OP_CMP) ? 8'd6 : 8'd7;
      ex_xn = 1'b1;
    end
    if (ex_live && ex_u.op == rd68041_pkg::OP_RTEF && rte_bad) ex_xv = 8'd14;
    if (ex_odd) ex_xv = 8'd3;
    // The FPU's (doc/fpu.md, "Exceptions"): first, a pending one.
    if (ex_live && fp_xv != 8'd0) begin
      ex_xv = fp_xv;
      ex_xn = fp_xn;
    end
  end
  rd68041_pkg::uop_t ex_out;                  // the micro-operation as it goes on to W
  always_comb begin
    ex_out = ex_u;
    if (ex_xv != 8'd0) begin
      ex_out.xvec  = ex_xv;
      ex_out.xnext = ex_xn;
      if (ex_odd) ex_out.imm = {ex_target[31:1], 1'b0};
      // An FP exception's frame: its format in tgt[19:16], its address field
      // in imm (rd68041_fpu).
      if (ex_live && fp_xv != 8'd0) begin
        ex_out.imm = fp_xea;
        ex_out.tgt = {11'h0, fp_xc, fp_xf, 16'h0};
      end
    end
    // The MC68LC040's FP instruction: the address it calculated, 0 for a
    // register or an immediate (UM A.5.1), for its format $4 frame.
    if (FPU == 0 && ex_u.valid && ex_u.xvec == 8'd11 && ex_u.xnext)
      ex_out.imm = ex_u.ea_en ? ex_ea : 32'h0;
  end

  assign ex_stall    = w_stall || div_wait || fp_wait || ex_wait;
  assign redirect    = ex_taken && !ex_stall && running && !x_flush;
  assign redirect_pc = ex_target;

  // ===========================================================================
  // Write-back
  // ===========================================================================
  // A micro-operation leaves write-back when its store, if it has one, is
  // taken; it completes then, unless the exception it takes comes before it
  // completes (w_pre), which leaves everything as it was. A halt stays here,
  // and the pipeline behind it with it.
  wire   w_x     = w_u.valid && (w_u.xvec != 8'd0);
  wire   w_pre   = w_x && !w_u.xnext;
  wire   w_st    = w_u.valid && w_u.st && !w_nost && !w_pre;
  // An exception, or a serialising micro-operation, waits here for every
  // queued write to be made before it commits, so that a write that faults
  // meanwhile is taken first (w_wf) -- the micro-operation not done, its
  // instruction run again. While one is pending nothing leaves write-back.
  // Its own store is offered only once they are: offered before, it would be
  // queued every clock it waits, and the queue never empty (an FMOVE out's last
  // store, which carries its post-instruction exception).
  wire   w_drain = w_u.valid && (w_u.xvec != 8'd0 || w_u.ser) && !st_empty;
  assign w_stall = w_u.valid && (w_u.halt || (w_st && !st_ready) || wf_pend || w_drain);
  wire   w_fire  = w_u.valid && !w_stall && running;
  wire   w_go    = w_fire && !w_pre;
  wire   w_wr    = w_go && !w_nowr;
  // A fault while an exception is being processed is a double bus fault, and
  // the processor halts (UM 8.2.1).
  wire   w_dbl   = w_fire && w_x && w_u.xs;
  // An instruction boundary: an instruction done, or exception processing.
  wire   w_end   = w_go && w_u.last && !w_x;
  // UM 8.2.6: traced as T1 and T0 were when the instruction began, which is
  // what the SR still holds (only XS_SR changes it, and it traces the
  // serialising instructions itself); never exception processing.
  wire   w_trace = w_end && !w_u.xs && !w_u.ser && (tr[1] || (tr[0] && w_cof));
  // UM 8.2.9, 8.3: an interrupt at a boundary, after a trace; not at the end
  // of an interrupt's own processing (UM 7.5.1: "at least one instruction in
  // an interrupt exception handler before recognizing another").
  wire   w_int   = w_end && !w_u.ser && int_pend && !(w_u.xs && xs_int) && !w_trace;
  // A write that faulted (UM 8.2.1), not exception processing's: an access
  // fault, taken on whatever is in write-back, which has not completed --
  // its instruction runs again, its registers put back by the undo log. The
  // frame's PC is that instruction's; the write and the two behind it are
  // the frame's write-backs. One of exception processing's is a double bus
  // fault (below).
  wire   w_wf    = w_u.valid && running && wf_pend && !wf_xs;
  // ... and with any of those, or a serialising micro-operation, everything
  // behind it goes and XS_SR takes over.
  assign x_flush = (w_fire && !w_dbl && (w_x || w_trace || w_int || w_u.ser)) || w_wf;

  // The frame each vector has (doc/checkpoint.md).
  function automatic logic [3:0] fmt_of(input logic [7:0] v);
    fmt_of = (v == 8'd2) ? 4'h7 :
             (v == 8'd3 || v == 8'd5 || v == 8'd6 || v == 8'd7 || v == 8'd9) ? 4'h2 : 4'h0;
  endfunction
  wire  [7:0] w_xv  = w_x ? w_u.xvec : w_trace ? 8'd9 : 8'd0;
  // An FP exception's format and address field come with it (rd68041_fpu):
  // the vectors the FPU raises, on an FP micro-operation.
  wire        w_fpx = w_x && (w_u.fpk != rd68041_fpu_pkg::FK_NONE) &&
                      (w_xv == 8'd11 || w_xv == 8'd14 || (w_xv >= 8'd48 && w_xv <= 8'd55));
  // The MC68LC040's FP instruction (UM A.5.1): vector 11 after it, which
  // nothing else is there; its EA rides in imm.
  wire        w_f4  = (FPU == 0) && w_x && (w_xv == 8'd11) && w_u.xnext;
  // The PC the frame holds: the instruction, or the next (UM 8.2).
  wire [31:0] w_xpc = w_pre ? w_u.pc : w_npc;
  // $7's FA and an address error's ADDR ride in imm; the rest's ADDR is the
  // instruction's address.
  wire [31:0] w_xad = (w_xv == 8'd2 || w_xv == 8'd3 || w_fpx || w_f4) ? w_u.imm : w_u.pc;
  // The FPU's record commits as its micro-operation completes -- or, for
  // BSUN, as it takes the exception that comes before it (UM 9.7.1.1).
  wire   fp_wxc    = w_fpx && w_u.tgt[20];
  assign fp_commit = w_fire && (w_u.fpk != rd68041_fpu_pkg::FK_NONE) && (!w_pre || fp_wxc);
  wire [15:0] w_xsr = {sr_sys, 3'b000, w_go ? w_ccr : ccr_w};
  // RTE of a throwaway frame runs again: it is not done, nor traced, yet.
  wire        w_rerun = (w_u.op == rd68041_pkg::OP_RTE) && (w_npc == w_u.pc);

  // XS_SR: the clock the SR changes, once every write has gone (UM 8.1). An
  // exception sets S, clears T1 and T0, and for an interrupt makes the level
  // the mask; its frame goes on the supervisor stack M names, and an
  // interrupt taken with M set clears it and puts the throwaway frame on the
  // interrupt stack (UM 8.2.9). A serialising write of the SR swaps the stack
  // pointers if the active one changes.
  wire        x_apply = running && (xs == XS_SR) && st_empty;
  // An interrupt or a trace, taken at a boundary, waits here for the writes
  // of the instruction before it: if one of them faults, the access fault is
  // taken in its place, at the same PC -- the interrupt is still pending
  // after it; the trace is lost (doc/divergences.md).
  wire        x_conv  = running && (xs == XS_SR) && wf_pend && !wf_xs;
  assign wf_ack = w_wf || x_conv;
  wire        s0      = sr_sys[5];
  wire        m0      = sr_sys[4];
  wire [31:0] sup_sp  = s0 ? q15 : (m0 ? msp : isp);
  wire [31:0] x_fsz   = (x_fmt == 4'h7) ? 32'd60 : (x_fmt == 4'h4) ? 32'd16 :
                        (x_fmt == 4'h2 || x_fmt == 4'h3) ? 32'd12 : 32'd8;
  wire [31:0] x_base  = sup_sp - x_fsz;
  wire        x_two   = x_int && m0;
  wire [31:0] x_base2 = ((s0 && !m0) ? q15 : isp) - 32'd8;
  wire  [7:0] x_sys   = {2'b00, 1'b1, m0 && !x_two, 1'b0, x_int ? x_lvl : sr_sys[2:0]};
  wire  [7:0] n_sys   = x_srn;
  function automatic logic [1:0] act_sp(input logic s, input logic m);
    act_sp = s ? (m ? 2'd2 : 2'd1) : 2'd0;     // USP, ISP, MSP
  endfunction
  wire  [1:0] a_old   = act_sp(s0, m0);
  wire  [1:0] a_new   = act_sp(n_sys[5], n_sys[4]);
  wire [31:0] sh_new  = (a_new == 2'd0) ? usp : (a_new == 2'd1) ? isp : msp;
  logic        xr_we;                         // ... and A7 then
  logic [31:0] xr_v;
  always_comb begin
    xr_we = 1'b0;
    xr_v  = x_two ? x_base2 : x_base;
    if (x_apply && x_exc) begin
      xr_we = 1'b1;
    end else if (x_apply && x_srw && a_old != a_new) begin
      xr_we = 1'b1;
      xr_v  = sh_new;
    end
  end
  // CINV and CPUSH: the instruction cache's part starts as XS_SR applies.
  assign ic_op = x_apply && !x_exc && x_cmo && !x_cmm && x_cmf[3];
  assign dc_op = x_apply && !x_exc && x_cmo && !x_cmm && x_cmf[2];
  // PFLUSH and PTEST, the same way, in the data unit.
  assign mm_op = x_apply && !x_exc && x_cmo && x_cmm;
  wire   cmo_done = (xs == XS_CMO) && !ic_busy && !dc_busy && !mm_busy && !mm_op;
  // Fetching starts again behind a serialising instruction, a RESET, or a
  // CINV or CPUSH.
  wire x_go = (x_apply && !x_exc && !x_rst && !x_cmo && !x_trc && !x_stp) ||
              (((xs == XS_RST && rst_cnt == 9'd511) || cmo_done) && !x_trc);
  assign rsto = (xs == XS_RST);

  // XS_UNDO: one register put back a clock, the lowest first.
  logic [3:0] xu_r;
  always_comb begin
    xu_r = 4'd0;
    for (int r = 15; r >= 0; r--) if (umask[r]) xu_r = 4'(r);
  end
  wire xu_we = (xs == XS_UNDO) && (umask != 16'h0);

  assign we0 = (w_wr && w_u.wr) || (rs == R_SSP && ld_done) || xr_we || xu_we;
  assign wa0 = xu_we ? {1'b0, xu_r} : (rs == R_SSP || xr_we) ? 5'd15 : w_u.wr_reg;
  assign wd0 = xu_we ? ush[xu_r] : (rs == R_SSP) ? ld_data : xr_we ? xr_v : w_res;
  assign wm0 = (rs == R_SSP || xr_we || xu_we) ? 4'hF : lanes_of(w_u.wr_sz);

  // What the log takes this clock: the old value of each architectural
  // register a micro-operation that is not its instruction's last writes.
  wire u_log  = w_go && !w_u.last;
  wire u_log0 = u_log && w_wr && w_u.wr && !w_u.wr_reg[4];
  wire u_log1 = u_log && (w_u.ea_upd != rd68041_pkg::U_NONE);
  wire u_log2 = u_log && w_wr && w_u.wr2 && !w_u.wr2_reg[4];
  // The address register update; the register file lets a result or a second
  // result to the same register win over it.
  assign we1 = w_go && (w_u.ea_upd != rd68041_pkg::U_NONE);
  assign wa1 = {2'b01, w_u.ea_an};
  assign wd1 = w_an;
  assign we2 = w_wr && w_u.wr2;
  assign wa2 = w_u.wr2_reg;
  assign wd2 = w_res2;

  rd68041_exc u_exc (
      .clk(clk), .rst_n(rst_n), .crst(crst),
      .start(x_apply && x_exc), .vec(x_vec), .intr(x_int), .level(x_lvl), .fmt(x_fmt),
      .sr_old(x_sr0), .sr_new(x_sr0 | 16'h2000), .pc(x_pc), .addr(x_addr), .pcf(x_pcf),
      .ssw(x_ssw), .tail(wf_tail), .use_tail(x_wft),
      .base(x_base), .two(x_two), .base2(x_base2), .vbr(vbr),
      .valid(exc_valid), .uop(exc_v), .take(xs_take));

  // <ea> fetch's next address, for the data cache's port A: the one <ea>
  // calculate hands on, else the one it holds.
  wire [31:0] of_ea_nx = ea_stall ? of_ea : agu_ea;

  rd68041_dmu u_dmu (
      .clk(clk), .rst_n(rst_n), .crst(crst), .sup(sup), .sfc(sfc), .dfc(dfc),
      .de(cacr[31]), .dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1),
      .tc_e(tc[15]), .tc_p(tc[14]), .urp(urp), .srp(srp),
      .ld_valid(ld_valid), .ld_addr(ld_addr), .ld_pre(of_ea_nx), .ld_sz(ld_sz),
      .ld_done(ld_done), .ld_data(ld_data), .ld_err(ld_err), .ld_cancel(ld_cancel),
      .ld_lock(of_u.lk && rs == R_RUN), .ld_alt(of_u.alt && rs == R_RUN),
      .ld_ack(of_u.ack && rs == R_RUN), .ld_tm(of_u.fn[2:0]), .ld_avec(ld_avec),
      .ld_noalloc(of_u.xs || rs != R_RUN), .ld_m16(of_u.m16 && of_u.ld && rs == R_RUN),
      .ld_wr((of_u.lk || (of_u.ld && of_u.st)) && rs == R_RUN),
      .ld_probe(of_prb && rs == R_RUN), .ld_oldest(ld_oldest), .ld_atc(ld_atc), .ld_ma(ld_ma),
      .ld_take(ld_take), .ld_rel(of_rel && !of_stall), .ld_fast(ld_fast), .ld_fdata(ld_fdata),
      .st_valid(w_st && running && !w_u.halt && !w_drain), .st_addr(w_ea),
      .st_sz(w_u.sz), .st_data(w_std), .st_lock(w_u.lk), .st_locke(w_u.lke),
      .st_alt(w_u.alt), .st_noalloc(w_u.xs), .st_m16(w_u.m16),
      .st_ready(st_ready), .st_empty(st_empty),
      .wf_pend(wf_pend), .wf_xs(wf_xs), .wf_tail(wf_tail), .wf_ack(wf_ack),
      .lock_open(lock_open),
      .dc_op(dc_op), .dc_scope(x_cmf[1:0]), .dc_push(x_cmp), .dc_addr(x_cma),
      .dc_p8k(tc[14]), .dc_busy(dc_busy),
      .mm_op(mm_op), .mm_kind(x_cmf[2:0]), .mm_addr(x_cma), .mm_busy(mm_busy),
      .mmusr_we(mmusr_we), .mmusr_v(mmusr_v),
      .if_flush(if_flush), .if_fl_kind(if_fl_kind), .if_fl_la(if_fl_la), .if_fl_s(if_fl_s),
      .iw_req(iw_req), .iw_la(iw_la), .iw_s(iw_s), .iw_fill(iw_fill), .iw_fill_la(iw_fill_la),
      .iw_fill_s(iw_fill_s), .iw_entry(iw_entry), .iw_done(iw_done),
      .d_valid(d_valid), .d_ready(d_ready), .d_addr(d_addr), .d_write(d_write),
      .d_siz(d_siz), .d_tt(d_tt), .d_tm(d_tm), .d_tln(d_tln), .d_upa(d_upa), .d_ci(d_ci),
      .d_lock(d_lock), .d_locke(d_locke), .d_wdata(d_wdata128), .d_rsp(d_rsp), .d_lw(d_lw),
      .d_data(rsp_data), .d_tci(d_tci), .d_done(d_done), .d_err(d_err), .d_avec(d_avec),
      .sn_addr(sn_addr), .sn_rd(sn_rd), .sn_dhit(sn_dhit), .sn_ddirty(sn_ddirty),
      .sn_rdata(sn_rdata), .sn_wr(sn_wr), .sn_wbe(sn_wbe), .sn_wdata(sn_wdata),
      .sn_dinv(sn_dinv));

  // A redirect, or an exception, kills <ea> fetch: its read, if one is on the
  // bus, is cancelled.
  // ... or EX's released one, while it waits.
  assign ld_cancel = (redirect || x_flush) && ((of_u.valid && of_mem && !of_got) || ex_wait);

  assign go    = (rs == R_PC && ld_done) || redirect || x_go;
  assign go_pc = (rs == R_PC) ? ld_data : redirect ? redirect_pc : x_npc;

  // An instruction is done at its boundary -- unless exception processing
  // follows, which is then the one done, its next PC the handler's; a
  // serialising one when XS_SR has changed the SR behind it. That is how a
  // trace of instructions sees them (core_cosim_tb).
  wire   ret_w      = w_end && !w_trace && !w_int && !w_u.ser;
  assign retire     = ret_w || (x_go && !x_rerun);
  assign retire_pc  = x_go ? x_ipc : w_u.pc;
  assign retire_npc = x_go ? x_npc : w_npc;

  // ===========================================================================
  // The pipeline's registers
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rs        <= R_SSP;
      halted    <= 1'b0;
      ccr_x     <= 5'h00;
      ccr_w     <= 5'h00;
      sr_sys    <= 8'h27;
      sfc       <= 3'd0;
      dfc       <= 3'd0;
      cacr      <= '0; tc <= '0; itt0 <= '0; itt1 <= '0; dtt0 <= '0; dtt1 <= '0;
      usp       <= '0; isp <= '0; vbr <= '0; msp <= '0; mmusr <= '0; urp <= '0; srp <= '0;
      ipl_s0    <= '0; ipl_s1 <= '0; ipl_s2 <= '0; ipl_lvl <= '0;
      nmi       <= 1'b0;
      xs        <= XS_IDLE;
      x_exc     <= 1'b0; x_int <= 1'b0; x_lvl <= '0; x_vec <= '0; x_fmt <= '0;
      x_pc      <= '0; x_addr <= '0; x_pcf <= '0; x_ssw <= '0; x_sr0 <= '0;
      x_srw     <= 1'b0; x_stp <= 1'b0; x_rst <= 1'b0; x_trc <= 1'b0;
      x_cmo     <= 1'b0; x_cmm <= 1'b0; x_cmf <= '0; x_cmp <= 1'b0; x_cma <= '0;
      x_wft     <= 1'b0;
      x_srn     <= '0; x_npc <= '0; x_ipc <= '0; x_rerun <= 1'b0;
      xs_int    <= 1'b0;
      rst_cnt   <= '0;
      for (int r = 0; r < 16; r++) ush[r] <= '0;
      umask     <= '0;
      seq       <= 5'd0;
      de_ind    <= 2'b00;
      de_len    <= 5'd0;
      de_op     <= '0;
      de_pc     <= '0;
      de_halted <= 1'b0;
      mv_mask   <= '0;
      mv_k      <= '0;
      ea_u      <= '0;
      of_u      <= '0;
      ex_u      <= '0;
      w_u       <= '0;
      of_ea     <= '0; ex_ea <= '0; w_ea <= '0;
      of_an     <= '0; ex_an <= '0; w_an <= '0;
      of_md     <= '0; of_got <= 1'b0; ex_md <= '0;
      ex_mw     <= 1'b0; ex_mok <= 1'b0; of_rel_q <= 1'b0;
      of_er     <= 1'b0; of_av <= 1'b0; ex_er <= 1'b0; ex_av <= 1'b0;
      of_at     <= 1'b0; of_ma <= 1'b0;
      w_res     <= '0; w_res2 <= '0; w_std <= '0; w_npc <= '0; w_ccr <= '0;
      w_nowr    <= 1'b0; w_nost <= 1'b0; w_cof <= 1'b0;
      div_go    <= 1'b0; div_done <= 1'b0;
      reset_v   <= '0;
    end else if (crst) begin
      // UM 8.2.10: the reset exception: S set, M and the trace bits cleared,
      // the mask at 7; VBR and CACR cleared, and TC's and the TTRs' enables
      // (UM 3.6.1).
      rs        <= R_SSP;
      halted    <= 1'b0;
      sr_sys    <= 8'h27;
      vbr       <= '0;
      cacr      <= '0;
      tc        <= '0;
      itt0      <= '0; itt1 <= '0; dtt0 <= '0; dtt1 <= '0;
      nmi       <= 1'b0;
      xs        <= XS_IDLE;
      umask     <= '0;
      seq       <= 5'd0;
      de_halted <= 1'b0;
      ea_u      <= '0;
      of_u      <= '0;
      ex_u      <= '0;
      w_u       <= '0;
      of_got    <= 1'b0;
      div_go    <= 1'b0;
      div_done  <= 1'b0;
    end else begin
      // --- reset ------------------------------------------------------------
      // A bus error or an odd PC while the reset vectors are read is a double
      // bus fault (UM 8.2.10).
      if (rs == R_SSP && ld_done) rs <= R_PC;
      if (rs == R_PC  && ld_done) begin rs <= R_RUN; reset_v <= ld_data; end
      if ((rs == R_SSP || rs == R_PC) && ld_done && ld_err) halted <= 1'b1;
      if (rs == R_PC && ld_done && ld_data[0]) halted <= 1'b1;

      // --- interrupts -------------------------------------------------------
      ipl_s0 <= ~ipl_n;
      ipl_s1 <= ipl_s0;
      ipl_s2 <= ipl_s1;
      if (ipl_s1 == ipl_s2) begin
        ipl_lvl <= ipl_s2;
        if (ipl_s2 == 3'd7 && ipl_lvl != 3'd7) nmi <= 1'b1;
      end

      // --- write-back -------------------------------------------------------
      if (w_u.valid && w_u.halt && running) halted <= 1'b1;
      if (w_dbl) halted <= 1'b1;
      // A write of exception processing's that faulted: a double bus fault too.
      if (wf_pend && wf_xs) halted <= 1'b1;
      if (w_go) begin
        ccr_w <= w_ccr;
        if (w_u.crw) begin
          case (w_u.cr)
            rd68041_pkg::CR_SFC:   sfc   <= w_res[2:0];
            rd68041_pkg::CR_DFC:   dfc   <= w_res[2:0];
            // UM 4.2: CACR has DE (31) and IE (15); the rest reads as zero.
            rd68041_pkg::CR_CACR:  cacr  <= w_res & 32'h8000_8000;
            rd68041_pkg::CR_TC:    tc    <= w_res & 32'h0000_C000;
            rd68041_pkg::CR_ITT0:  itt0  <= w_res & 32'hFFFF_E364;
            rd68041_pkg::CR_ITT1:  itt1  <= w_res & 32'hFFFF_E364;
            rd68041_pkg::CR_DTT0:  dtt0  <= w_res & 32'hFFFF_E364;
            rd68041_pkg::CR_DTT1:  dtt1  <= w_res & 32'hFFFF_E364;
            rd68041_pkg::CR_USP:   usp   <= w_res;
            rd68041_pkg::CR_VBR:   vbr   <= w_res;
            rd68041_pkg::CR_MSP:   msp   <= w_res;
            rd68041_pkg::CR_ISP:   isp   <= w_res;
            rd68041_pkg::CR_MMUSR: mmusr <= w_res;
            // UM 3.1.1: bits 8-0 "must be zero" -- kept as written (a table
            // search uses 31-9), as UAE040 keeps them (doc/oracles.md).
            rd68041_pkg::CR_URP:   urp   <= w_res;
            rd68041_pkg::CR_SRP:   srp   <= w_res;
            default: ;
          endcase
        end
      end
      if (!w_stall) w_u <= '0;

      // --- the undo log ---------------------------------------------------------
      if (u_log0 && !umask[wa0[3:0]]) begin ush[wa0[3:0]] <= rf_o0; umask[wa0[3:0]] <= 1'b1; end
      if (u_log1 && !umask[wa1[3:0]]) begin ush[wa1[3:0]] <= rf_o1; umask[wa1[3:0]] <= 1'b1; end
      if (u_log2 && !umask[wa2[3:0]]) begin ush[wa2[3:0]] <= rf_o2; umask[wa2[3:0]] <= 1'b1; end
      if (w_go && w_u.last) umask <= '0;
      if (xu_we) umask[xu_r] <= 1'b0;
      if (xs == XS_UNDO && umask == 16'h0) xs <= XS_SR;

      // --- exception processing ---------------------------------------------
      if (xs == XS_SEQ && w_end && w_u.xs) xs <= XS_IDLE;
      if (xs == XS_RST) rst_cnt <= rst_cnt + 9'd1;
      if ((xs == XS_RST && rst_cnt == 9'd511) || cmo_done) begin
        begin
          // A traced RESET, CINV or CPUSH: its trace exception once it is done.
          xs <= x_trc ? XS_SR : XS_IDLE;
          if (x_trc) begin
            x_exc  <= 1'b1;
            x_int  <= 1'b0;
            x_vec  <= 8'd9;
            x_fmt  <= 4'h2;
            x_pc   <= x_npc;
            x_addr <= x_ipc;
            x_sr0  <= {sr_sys, 3'b000, ccr_w};
          end
        end
      end
      if (xs == XS_STOP && int_pend) begin
        // UM 8.2.9: STOP ends with an interrupt, its frame's PC the next
        // instruction's.
        xs    <= XS_SR;
        x_exc <= 1'b1;
        x_int <= 1'b1;
        x_lvl <= nmi ? 3'd7 : ipl_lvl;
        x_vec <= 8'd0;
        x_fmt <= 4'h0;
        x_pc  <= x_npc;
        x_sr0 <= {sr_sys, 3'b000, ccr_w};
      end
      if (x_apply) begin
        if (x_exc) begin
          sr_sys <= x_sys;
          if (!s0) usp <= q15;
          if (x_two) msp <= x_base;
          xs     <= XS_SEQ;
          xs_int <= x_int;
          if (x_int && x_lvl == 3'd7) nmi <= 1'b0;
        end else begin
          if (x_srw) begin
            sr_sys <= n_sys;
            if (a_old != a_new) begin
              case (a_old)
                2'd0:    usp <= q15;
                2'd1:    isp <= q15;
                default: msp <= q15;
              endcase
            end
          end
          if (x_rst) begin
            xs      <= XS_RST;
            rst_cnt <= 9'd0;
          end else if (x_cmo) begin
            xs      <= XS_CMO;
          end else if (x_trc) begin
            // A traced serialising instruction: its trace exception now, from
            // the SR it left (UM 8.2.6; STOP so never stops).
            x_exc  <= 1'b1;
            x_int  <= 1'b0;
            x_vec  <= 8'd9;
            x_fmt  <= 4'h2;
            x_pc   <= x_npc;
            x_addr <= x_ipc;
            x_sr0  <= {x_srw ? n_sys : sr_sys, 3'b000, ccr_w};
          end else if (x_stp) begin
            xs <= XS_STOP;
          end else begin
            xs <= XS_IDLE;
          end
        end
      end
      // An exception, an interrupt or a serialising micro-operation, from
      // write-back: XS_SR next.
      if (x_conv) begin
        x_exc  <= 1'b1;
        x_int  <= 1'b0;
        x_vec  <= 8'd2;
        x_fmt  <= 4'h7;
        x_trc  <= 1'b0;
        x_wft  <= 1'b1;
      end
      if (x_flush) begin
        xs     <= ((w_pre || w_wf) && umask != 16'h0) ? XS_UNDO : XS_SR;
        x_exc  <= w_x || w_trace || w_int;
        x_int  <= w_int;
        x_lvl  <= nmi ? 3'd7 : ipl_lvl;
        x_vec  <= w_xv;
        x_fmt  <= w_fpx ? w_u.tgt[19:16] : w_f4 ? 4'h4 : fmt_of(w_xv);
        x_pc   <= w_xpc;
        x_addr <= w_xad;
        x_pcf  <= (FPU == 0) ? w_u.pc : 32'h0;
        x_ssw  <= w_u.tgt[15:0];
        x_sr0  <= w_xsr;
        x_srw  <= w_u.srw && !w_x;
        x_stp  <= w_u.stp && !w_x;
        x_rst  <= w_u.rsto && !w_x;
        x_cmo  <= w_u.cmo && !w_x;
        x_cmm  <= w_u.cond[1];
        x_cmf  <= w_u.fn;
        x_cmp  <= w_u.cond[0];
        x_cma  <= w_res;
        x_srn  <= {w_res[15:12], 1'b0, w_res[10:8]};
        x_npc  <= w_npc;
        x_ipc  <= w_u.pc;
        x_trc  <= (tr[1] || (tr[0] && w_cof)) && !w_rerun;
        x_rerun <= w_rerun;
        x_wft  <= 1'b0;
        if (w_wf) begin
          x_exc  <= 1'b1;
          x_int  <= 1'b0;
          x_vec  <= 8'd2;
          x_fmt  <= 4'h7;
          x_pc   <= w_u.pc;
          x_sr0  <= {sr_sys, 3'b000, ccr_w};
          x_srw  <= 1'b0;
          x_stp  <= 1'b0;
          x_rst  <= 1'b0;
          x_cmo  <= 1'b0;
          x_trc  <= 1'b0;
          x_rerun <= 1'b0;
          x_wft  <= 1'b1;
        end
      end

      // PTEST's result (UM 3.1.4).
      if (mmusr_we) mmusr <= mmusr_v;

      // --- the divider ------------------------------------------------------
      if (div_start) div_go <= 1'b1;
      if (div_go && !div_busy) div_done <= 1'b1;

      // --- execute -> write-back --------------------------------------------
      if (!ex_stall) begin
        w_u    <= ex_out;
        w_ea   <= ex_ea;
        w_an   <= ex_an;
        w_res  <= res;
        w_res2 <= res2;
        w_std  <= st_val;
        w_npc  <= ex_npc;
        w_ccr  <= ccr_n;
        w_nowr <= ex_nowr;
        w_nost <= ex_nost;
        w_cof  <= ex_taken || ex_u.cof;
        if (ex_u.valid) ccr_x <= ccr_n;
        ex_u   <= '0;
        div_go   <= 1'b0;
        div_done <= 1'b0;
      end

      // --- <ea> fetch -> execute --------------------------------------------
      // EX's released read answered: as it would have gone on with it.
      of_rel_q <= 1'b0;
      if (ex_wait && ld_done) begin
        ex_md  <= ld_data;
        ex_er  <= ld_err;
        ex_av  <= ld_avec;
        ex_mok <= 1'b1;
        if (ld_err) ex_u <= ex_flt;
      end
      if (of_u.valid && of_mem && !of_got && of_ans) begin
        of_md  <= ld_data;
        of_er  <= ld_err;
        of_at  <= ld_atc;
        of_ma  <= ld_ma;
        of_av  <= ld_avec;
        of_got <= 1'b1;
      end
      if (!of_stall) begin
        ex_u   <= of_out;
        ex_ea  <= of_ea;
        ex_an  <= of_an;
        ex_md  <= of_got ? of_md : of_rel ? ld_fdata : ld_data;
        ex_er  <= of_got ? of_er : !of_rel && ld_err;
        ex_av  <= of_got ? of_av : !of_rel && ld_avec;
        ex_mw  <= of_rel;
        ex_mok <= ld_fast;
        of_rel_q <= of_rel && !redirect && !x_flush;
        of_u   <= '0;
        of_got <= 1'b0;
      end

      // --- <ea> calculate -> <ea> fetch -------------------------------------
      if (!ea_stall) begin
        of_u  <= ea_out;
        of_ea <= agu_ea;
        of_an <= ea_an_new;
        ea_u  <= '0;
      end

      // --- exception processing -> <ea> calculate ---------------------------
      if (xs_take) ea_u <= exc_u;

      // --- decode -> <ea> calculate ----------------------------------------
      if (de_issue) begin
        ea_u <= de_u;
        if (de_more) begin
          seq <= seq + 5'd1;
          if (!step) begin
            de_op  <= q_win[191:176];
            de_pc  <= q_pc;
            de_ind <= dec_ind;
            de_len <= dec_u.len;
          end
        end else begin
          seq <= 5'd0;
        end
        // MOVEM's registers: the mask, less each one transferred.
        if (!step) begin
          mv_mask <= mv_used ? (mv_src & ~mv_bit) : mv_src;
          mv_k    <= mv_used ? 5'd1 : 5'd0;
        end else if (mv_used) begin
          mv_mask <= mv_mask & ~mv_bit;
          mv_k    <= mv_k + 5'd1;
        end
        // Nothing more is decoded after an instruction that is not built or
        // that takes an exception before it executes, until the pipeline is
        // emptied behind it.
        if (de_u.halt || de_u.xvec != 8'd0) de_halted <= 1'b1;
      end

      // --- a taken branch: everything younger goes ----------------------------
      if (redirect) begin
        ex_u      <= '0;
        ea_u      <= '0;
        of_u      <= '0;
        of_got    <= 1'b0;
        seq       <= 5'd0;
        de_halted <= 1'b0;
      end

      // --- an exception or a serialising instruction: everything behind it --
      if (x_flush) begin
        w_u       <= '0;
        ex_u      <= '0;
        of_u      <= '0;
        ea_u      <= '0;
        of_got    <= 1'b0;
        seq       <= 5'd0;
        de_halted <= 1'b0;
        div_go    <= 1'b0;
        div_done  <= 1'b0;
        ccr_x     <= w_go ? w_ccr : ccr_w;
      end
    end
  end

  logic unused_ok;
  assign unused_ok = &{1'b0, reset_v, cmd_w[13:12], cmd_w[10:8]};

endmodule
