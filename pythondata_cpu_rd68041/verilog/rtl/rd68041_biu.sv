// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The bus unit: the MC68040's bus as a master (UM 7.4-7.6), arbitration (UM
// 7.8), and -- through rd68041_snoop, instantiated here so that every pin has
// one owner -- the snooping of an alternate master's transfers (UM 7.9).
//
// Above this module, a request is ONE bus transfer: a byte, word or long word
// already placed on its byte lanes by the requester (UM 7.2, Table 7-1: the
// MC68040 has no dynamic bus sizing, and a misaligned operand is split by the
// data memory unit before it reaches the bus, UM 7.3), or a line of four long
// words. Nothing above this module knows about TS, TA, TBI or retry:
//
//   req_valid/req_ready  a handshake: the request is taken on the rising edge
//                        where both are high, and that edge starts its C1.
//   rsp_valid            one per long word moved, registered: the edge after TA.
//   rsp_done             with the last response, or alone on a bus error.
//   rsp_err              the transfer ended with TEA (UM 7.6.1), on the beat
//                        rsp_beat names; a line stops there.
//
// A retry (TA with TEA, UM 7.6.2) is run again here and never reported. A line
// the slave burst-inhibits (TBI with the first TA, UM 7.4.2, 7.4.4) is finished
// here as three long-word transfers, still one request and still one bus
// tenure.
//
// Every output is registered on the rising edge of BCLK (UM 7.1: "all timing
// specifications are referenced to this edge"); the bus inputs are sampled on
// the same edge with no synchroniser (UM 7.1: they "are synchronously sampled").
// BB and TIP leave the bus through rd68041_late_release (specification 39).

module rd68041_biu (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         crst,          // internal reset (rd68041_rstctl)

    // The request ------------------------------------------------------------
    input  logic         req_valid,
    output logic         req_ready,
    input  logic [31:0]  req_addr,
    input  logic         req_write,
    input  logic  [1:0]  req_siz,       // rd68041_pkg::SIZ_*
    input  logic  [1:0]  req_tt,
    input  logic  [2:0]  req_tm,
    input  logic  [1:0]  req_tln,
    input  logic  [1:0]  req_upa,
    input  logic         req_ciout,
    input  logic         req_lock,      // part of a locked sequence (UM 7.4.5)
    input  logic         req_locke,     // ... and its last write
    input  logic         req_more,      // another request already waits behind this one
    input  logic [127:0] req_wdata,     // long word k of the transfer in [32k+31:32k]
    input  logic         lock_seq,      // a locked sequence is under way: keep LOCK

    // The response -----------------------------------------------------------
    output logic         rsp_valid,
    output logic  [1:0]  rsp_beat,      // which long word of the transfer, 0 first
    output logic  [1:0]  rsp_lw,        // ... and its A3-A2
    output logic [31:0]  rsp_data,
    output logic         rsp_tci,       // TCI with a line's first TA (UM 7.4.2)
    output logic         rsp_avec,      // AVEC with TA (UM 7.5.1.2)
    output logic         rsp_done,
    output logic         rsp_err,

    output logic         owner,         // the core is, or is about to be, bus master

    // The pins (doc/pinout.md) -----------------------------------------------
    input  logic [31:0]  a_i,
    output logic [31:0]  a_o,
    output logic         a_oe,
    input  logic [31:0]  d_i,
    output logic [31:0]  d_o,
    output logic         d_oe,
    input  logic  [1:0]  tt_i,
    output logic  [1:0]  tt_o,
    output logic  [2:0]  tm_o,
    output logic  [1:0]  tln_o,
    output logic  [1:0]  upa_o,
    input  logic         rw_i,
    output logic         rw_o,
    input  logic  [1:0]  siz_i,
    output logic  [1:0]  siz_o,
    output logic         lock_n_o,
    output logic         locke_n_o,
    output logic         ciout_n_o,
    output logic         attr_oe,
    input  logic         ts_n_i,
    output logic         ts_n_o,
    output logic         tip_n_o,
    output logic         tip_oe,
    input  logic         ta_n_i,
    output logic         ta_n_o,
    output logic         ta_oe,
    input  logic         tea_n_i,
    input  logic         tci_n_i,
    input  logic         tbi_n_i,
    input  logic         avec_n_i,
    output logic         br_n_o,
    input  logic         bg_n_i,
    input  logic         bb_n_i,
    output logic         bb_n_o,
    output logic         bb_oe,
    input  logic  [1:0]  sc_i,
    output logic         mi_n_o,

    // The caches, for the snooper (rd68041_snoop) --------------------------------
    output logic [31:0]  sn_addr,
    input  logic         sn_dhit,
    input  logic         sn_ddirty,
    input  logic         sn_ihit,
    input  logic [31:0]  sn_rdata,
    output logic         sn_wr,
    output logic  [3:0]  sn_wbe,
    output logic [31:0]  sn_wdata,
    output logic         sn_dinv,
    output logic         sn_iinv,
    output logic         sn_rd
);

  // ---------------------------------------------------------------------------
  // States. UM 7.8.1 names five arbitration states -- idle, snoop, implicit
  // ownership, park and active bus cycle -- and Table 7-6 how each looks on BB
  // and BG. The active cycle is split here into its first clock (C1, TS
  // asserted) and the clocks after it, where TA is sampled; REL is the clock in
  // which the core, having lost BG, drives BB and TIP negated before releasing
  // them (Figure 7-32, C8). Snoop is M2's.
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ST_RESET = 3'd0,
    ST_IDLE  = 3'd1,  // not master, not driving
    ST_IMPL  = 3'd2,  // implicit ownership: BG, no cycle, BB three-stated
    ST_PARK  = 3'd3,  // explicit ownership, no cycle: BB asserted
    ST_C1    = 3'd4,  // first clock of a transfer: TS asserted
    ST_DATA  = 3'd5,  // the clocks TA is sampled in
    ST_REL   = 3'd6   // BB and TIP negated, then released
  } st_e;

  st_e st;

  // The transfer under way, latched when it is taken.
  logic [31:0]  x_addr;
  logic         x_write;
  logic  [1:0]  x_siz;
  logic  [1:0]  x_tt;
  logic  [2:0]  x_tm;
  logic  [1:0]  x_tln;
  logic  [1:0]  x_upa;
  logic         x_ciout;
  logic         x_lock;
  logic         x_locke;
  logic [127:0] x_wdata;
  logic  [1:0]  beat;       // long words of the transfer moved so far
  logic         tbi_mode;   // the line was burst-inhibited: long words from here
  logic         rerun;      // the latched transfer is to be run again (retry)

  // ---------------------------------------------------------------------------
  // The inputs, as sampled on this edge (UM 7.1).
  // ---------------------------------------------------------------------------
  wire ta    = !ta_n_i;
  wire tea   = !tea_n_i;
  wire tbi   = !tbi_n_i;
  wire grant = !bg_n_i;
  wire bb_free = bb_n_i;    // nobody, ourselves included, asserts BB

  // UM 7.6, Table 7-5: TA and TEA together are a retry, TEA alone a bus error.
  wire in_data   = (st == ST_DATA);
  wire t_normal  = in_data &&  ta && !tea;
  wire t_error   = in_data && !ta &&  tea;
  wire t_retry   = in_data &&  ta &&  tea;

  wire is_line   = (x_siz == rd68041_pkg::SIZ_LINE);

  // UM 7.6.2: "a retry signaled during the second, third, or fourth cycle of a
  // line" is a bus error, and "a burst-inhibited line transfer can only be
  // retried on the initial transfer".
  wire retry_ok  = t_retry && (beat == 2'd0) && !tbi_mode;
  wire bus_err   = t_error || (t_retry && !retry_ok);

  // A line's first TA with TBI: the rest as three long words (UM 7.4.2).
  wire go_tbi    = t_normal && is_line && !tbi_mode && (beat == 2'd0) && tbi;
  // The burst continues in this same transfer.
  wire burst_on  = t_normal && is_line && !tbi_mode && !tbi && (beat != 2'd3);
  // A burst-inhibited line's next long-word transfer.
  wire tbi_next  = t_normal && is_line && ((tbi_mode && (beat != 2'd3)) || go_tbi);
  // The request is over: normally, or on a bus error.
  wire x_end     = (t_normal && !burst_on && !tbi_next) || bus_err;

  // Is there a transfer to start? A rerun first: it is the same request, and a
  // retry being signalled on this very edge is one too.
  wire retry_now = t_retry && retry_ok;
  wire pending   = rerun || retry_now || req_valid;

  // Where a new transfer can start on this edge. UM 7.8.1: from idle once BG is
  // asserted and BB negated; from implicit ownership or park at once ("the
  // processor must assert BR, TIP, and TS simultaneously to enter an active bus
  // cycle"); and back to back, at the end of one, while BG is still asserted.
  wire may_start = ((st == ST_IDLE) && grant && bb_free) ||
                   ((st == ST_IMPL) && grant) ||
                   ((st == ST_PARK) && grant) ||
                   ((x_end || retry_now) && grant);
  wire start     = pending && may_start;
  // Only a new request is handed back; a rerun is the request already taken.
  assign req_ready = start && !rerun && !retry_now;

  // The A3-A2 of long word `beat` of the transfer: a burst wraps (UM 7.4.2).
  function automatic logic [1:0] lw_of(input logic [1:0] base, input logic [1:0] k);
    lw_of = base + k;
  endfunction

  // The long word of the write data that goes with beat k.
  function automatic logic [31:0] wd_of(input logic [127:0] w, input logic [1:0] k);
    case (k)
      2'd0:    wd_of = w[31:0];
      2'd1:    wd_of = w[63:32];
      2'd2:    wd_of = w[95:64];
      default: wd_of = w[127:96];
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // BB and TIP: the value is set on a rising edge, the enable through
  // rd68041_late_release so that both leave the bus half a clock after they are
  // negated (specification 39).
  // ---------------------------------------------------------------------------
  logic bb_on_d, tip_on_d;

  rd68041_late_release u_bb_rel (
      .clk(clk), .rst_n(rst_n), .on_d(bb_on_d), .oe(bb_oe));
  rd68041_late_release u_tip_rel (
      .clk(clk), .rst_n(rst_n), .on_d(tip_on_d), .oe(tip_oe));

  // ---------------------------------------------------------------------------
  // The snooper, and the pins it shares: D, TA, MI.
  // ---------------------------------------------------------------------------
  logic [31:0] m_d_o, s_d_o;
  logic        m_d_oe, s_d_oe;
  logic        sn_mi_release_d, s_ta_on_d;

  rd68041_snoop u_snoop (
      .clk(clk), .rst_n(rst_n), .crst(crst), .owner(owner),
      .ts_n_i(ts_n_i), .a_i(a_i), .d_i(d_i), .tt_i(tt_i), .rw_i(rw_i),
      .siz_i(siz_i), .sc_i(sc_i), .ta_n_i(ta_n_i), .tea_n_i(tea_n_i),
      .tbi_n_i(tbi_n_i),
      .mi_release_d(sn_mi_release_d), .d_o(s_d_o), .d_oe(s_d_oe),
      .ta_n_o(ta_n_o), .ta_on_d(s_ta_on_d),
      .sn_addr(sn_addr), .sn_dhit(sn_dhit), .sn_ddirty(sn_ddirty),
      .sn_ihit(sn_ihit), .sn_rdata(sn_rdata), .sn_wr(sn_wr), .sn_wbe(sn_wbe),
      .sn_wdata(sn_wdata), .sn_dinv(sn_dinv), .sn_iinv(sn_iinv), .sn_rd(sn_rd)
  );

  rd68041_late_release u_ta_rel (
      .clk(clk), .rst_n(rst_n), .on_d(s_ta_on_d), .oe(ta_oe));

  // The data bus is the bus unit's in a transfer of its own and the snooper's
  // when it answers an alternate master; never both, and a clock apart at the
  // least (the bus changes hands through an idle clock, UM 7.8.1).
  assign d_o  = m_d_oe ? m_d_o : s_d_o;
  assign d_oe = m_d_oe | s_d_oe;

  // The next state, and what each state drives.
  st_e st_d;
  always_comb begin
    st_d = st;
    case (st)
      ST_RESET: st_d = ST_IDLE;
      ST_IDLE:  if (start) st_d = ST_C1;
                else if (grant && bb_free) st_d = ST_IMPL;
      ST_IMPL:  if (start) st_d = ST_C1;
                else if (!grant) st_d = ST_IDLE;
      ST_PARK:  if (start) st_d = ST_C1;
                else if (!grant) st_d = ST_REL;
      ST_C1:    st_d = ST_DATA;
      ST_DATA:  if (burst_on) st_d = ST_DATA;
                else if (tbi_next) st_d = ST_C1;
                else if (start) st_d = ST_C1;
                else if ((x_end || t_retry) && grant) st_d = ST_PARK;
                else if (x_end || t_retry) st_d = ST_REL;
      ST_REL:   st_d = ST_IDLE;
      default:  st_d = ST_IDLE;
    endcase
  end

  // BB is asserted from a transfer's first clock for as long as the core owns
  // the bus explicitly (UM 5.6.3). TIP is driven while the core owns the bus,
  // implicitly or explicitly, and asserted only while a transfer is under way
  // (UM 5.4.2). Neither is enabled for REL: the edge that enters it sets both
  // negated, and rd68041_late_release keeps them driven, negated, for the
  // first half of that clock only.
  // The internal reset negates both on its first edge and releases them half
  // a clock later, as giving the bus up does (UM 7.10, Figure 7-45: "BB and TIP
  // ... are negated before transitioning to a three-state level").
  assign bb_on_d  = !crst && ((st_d == ST_C1) || (st_d == ST_DATA) || (st_d == ST_PARK));
  assign tip_on_d = !crst && ((st_d == ST_C1) || (st_d == ST_DATA) || (st_d == ST_PARK) ||
                              (st_d == ST_IMPL));

  wire owns_d = (st_d == ST_C1) || (st_d == ST_DATA) || (st_d == ST_PARK) ||
                (st_d == ST_IMPL);

  // MI (UM 5.5.2, 7.9.1). Negated while the core is master, "from the same edge
  // that the M68040 asserts the TS signal"; otherwise asserted -- during and
  // after reset until the core's first cycle, "from the last TA of the current
  // bus cycle if the M68040 owns the bus and loses it", and between an
  // alternate master's transfers -- except where the snooper lets memory answer.
  wire master_d = (st_d == ST_C1) || (st_d == ST_DATA) || (st_d == ST_PARK);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mi_n_o <= 1'b0;
    end else if (crst) begin
      mi_n_o <= 1'b0;
    end else begin
      mi_n_o <= master_d || sn_mi_release_d;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st        <= ST_RESET;
      x_addr    <= '0;
      x_write   <= 1'b0;
      x_siz     <= rd68041_pkg::SIZ_LONG;
      x_tt      <= rd68041_pkg::TT_NORMAL;
      x_tm      <= '0;
      x_tln     <= '0;
      x_upa     <= '0;
      x_ciout   <= 1'b0;
      x_lock    <= 1'b0;
      x_locke   <= 1'b0;
      x_wdata   <= '0;
      beat      <= '0;
      tbi_mode  <= 1'b0;
      rerun     <= 1'b0;
      a_o       <= '0;
      a_oe      <= 1'b0;
      m_d_o     <= '0;
      m_d_oe    <= 1'b0;
      tt_o      <= rd68041_pkg::TT_NORMAL;
      tm_o      <= '0;
      tln_o     <= '0;
      upa_o     <= '0;
      rw_o      <= 1'b1;
      siz_o     <= rd68041_pkg::SIZ_LONG;
      lock_n_o  <= 1'b1;
      locke_n_o <= 1'b1;
      ciout_n_o <= 1'b1;
      attr_oe   <= 1'b0;
      ts_n_o    <= 1'b1;
      tip_n_o   <= 1'b1;
      bb_n_o    <= 1'b1;
      br_n_o    <= 1'b1;
      owner     <= 1'b0;
      rsp_valid <= 1'b0;
      rsp_beat  <= '0;
      rsp_lw    <= '0;
      rsp_data  <= '0;
      rsp_tci   <= 1'b0;
      rsp_avec  <= 1'b0;
      rsp_done  <= 1'b0;
      rsp_err   <= 1'b0;
    end else if (crst) begin
      // UM 7.10: "all signals that can be, are three-stated, and the rest are
      // driven to their inactive state", MI asserted; "any bus cycle in progress
      // [terminates] as if TA or TEA had been asserted" -- so nothing is
      // reported and nothing is rerun.
      st        <= ST_RESET;
      beat      <= '0;
      tbi_mode  <= 1'b0;
      rerun     <= 1'b0;
      a_oe      <= 1'b0;
      m_d_oe      <= 1'b0;
      attr_oe   <= 1'b0;
      ts_n_o    <= 1'b1;
      tip_n_o   <= 1'b1;
      bb_n_o    <= 1'b1;
      br_n_o    <= 1'b1;
      lock_n_o  <= 1'b1;
      locke_n_o <= 1'b1;
      owner     <= 1'b0;
      rsp_valid <= 1'b0;
      rsp_done  <= 1'b0;
      rsp_err   <= 1'b0;
    end else begin
      st    <= st_d;
      owner <= owns_d;

      // --- the response, registered --------------------------------------
      rsp_valid <= t_normal;
      rsp_done  <= x_end;
      rsp_err   <= bus_err;
      if (in_data && (ta || tea)) begin
        rsp_beat <= beat;
        rsp_lw   <= lw_of(x_addr[3:2], beat);
        rsp_data <= d_i;
        rsp_avec <= !avec_n_i;
      end
      if (t_normal && (beat == 2'd0) && !tbi_mode) begin
        rsp_tci <= !tci_n_i;
      end

      // --- taking a transfer ---------------------------------------------
      if (req_ready) begin
        x_addr   <= req_addr;
        x_write  <= req_write;
        x_siz    <= req_siz;
        x_tt     <= req_tt;
        x_tm     <= req_tm;
        x_tln    <= req_tln;
        x_upa    <= req_upa;
        x_ciout  <= req_ciout;
        x_lock   <= req_lock;
        x_locke  <= req_locke;
        x_wdata  <= req_wdata;
      end

      // --- where the transfer is ----------------------------------------
      if (start) begin
        beat     <= '0;
        tbi_mode <= 1'b0;
        rerun    <= 1'b0;
      end else begin
        if (burst_on || tbi_next) beat <= beat + 2'd1;
        if (go_tbi) tbi_mode <= 1'b1;
        // UM 7.6.2: a retry with BG negated is a relinquish and retry; the
        // transfer is run again once the bus comes back (Figure 7-31).
        if (retry_now) rerun <= 1'b1;
        if (x_end) tbi_mode <= 1'b0;
      end

      // --- C1 of a transfer: address, attributes, TS -----------------------
      if (start) begin
        // UM 7.4.1-7.4.4, step 1-8 of each flowchart.
        if (req_ready) begin
          a_o       <= req_addr;
          tt_o      <= req_tt;
          tm_o      <= req_tm;
          tln_o     <= req_tln;
          upa_o     <= req_upa;
          rw_o      <= !req_write;
          siz_o     <= req_siz;
          ciout_n_o <= !req_ciout;
          lock_n_o  <= !(req_lock || lock_seq);
          locke_n_o <= !req_locke;
        end else begin
          // A rerun: "the same access information" (UM 7.6.2).
          a_o       <= x_addr;
          tt_o      <= x_tt;
          tm_o      <= x_tm;
          tln_o     <= x_tln;
          upa_o     <= x_upa;
          rw_o      <= !x_write;
          siz_o     <= x_siz;
          ciout_n_o <= !x_ciout;
          lock_n_o  <= !(x_lock || lock_seq);
          locke_n_o <= !x_locke;
        end
        a_oe    <= 1'b1;
        attr_oe <= 1'b1;
        ts_n_o  <= 1'b0;
        tip_n_o <= 1'b0;
        bb_n_o  <= 1'b0;
        m_d_oe    <= 1'b0;
      end else if (tbi_next) begin
        // A burst-inhibited line's next long word: "the processor increments A3
        // and A2 for each read, and the new address is placed on the address
        // bus" (UM 7.4.2); A1-A0 stay those of the access (UM 7.2).
        a_o     <= {x_addr[31:4], lw_of(x_addr[3:2], beat + 2'd1), x_addr[1:0]};
        siz_o   <= rd68041_pkg::SIZ_LONG;
        ts_n_o  <= 1'b0;
        m_d_oe    <= 1'b0;
      end else if (st == ST_C1) begin
        // C2: TS negated; a write drives its data (UM 7.4.3, 7.4.4).
        ts_n_o  <= 1'b1;
        if (x_write) begin
          m_d_o  <= wd_of(x_wdata, beat);
          m_d_oe <= 1'b1;
        end
      end else if (burst_on) begin
        // C3-C5 of a line burst: the next long word of a write (UM 7.4.4).
        if (x_write) m_d_o <= wd_of(x_wdata, beat + 2'd1);
      end else if (x_end || t_retry) begin
        // The transfer is over. UM 7.4.3: "the processor also three-states the
        // data bus during the first half of the next clock following termination
        // of the write transfer"; TIP negates unless another transfer begins
        // (UM 7.4.1). LOCKE negates after the locked sequence's last write
        // (UM 7.4.5), LOCK unless the sequence continues.
        m_d_oe      <= 1'b0;
        tip_n_o   <= 1'b1;
        locke_n_o <= 1'b1;
        lock_n_o  <= !(lock_seq && !x_locke);
        if (!grant) begin
          // Losing the bus: everything but BB and TIP leaves it at this edge
          // (specification 38); BB and TIP are driven negated through REL
          // (specification 39).
          a_oe     <= 1'b0;
          attr_oe  <= 1'b0;
          bb_n_o   <= 1'b1;
          lock_n_o <= 1'b1;
        end
      end else if ((st == ST_PARK || st == ST_IMPL) && grant) begin
        // Owning the bus between transfers: LOCK follows the requester, so a
        // locked sequence's read and write are joined by it and nothing else
        // is (UM 7.4.5).
        lock_n_o <= !lock_seq;
      end else if (st == ST_PARK && !grant) begin
        a_oe     <= 1'b0;
        attr_oe  <= 1'b0;
        bb_n_o   <= 1'b1;
        lock_n_o <= 1'b1;
      end else if (st == ST_IDLE && grant && bb_free) begin
        // Implicit ownership: "the processor drives its output pins with
        // undetermined values and three-states BB, but does not perform a bus
        // cycle" (UM 7.8.1). TS and TIP are driven negated. The data bus is
        // not driven outside a write (doc/divergences.md).
        a_oe    <= 1'b1;
        attr_oe <= 1'b1;
        ts_n_o  <= 1'b1;
        tip_n_o <= 1'b1;
      end else if (st == ST_IMPL && !grant) begin
        a_oe    <= 1'b0;
        attr_oe <= 1'b0;
      end

      // --- BR (UM 5.6.1, 7.8.1) --------------------------------------------
      // Asserted while an access is pending. "BR is negated when the M68040
      // begins an access to the external bus with no other accesses pending";
      // but when the bus is already the core's, "BR is asserted simultaneously
      // with transfer start (TS)", and leaving park takes "the simultaneous
      // assertion of BR, TIP, and TS".
      if (start) begin
        br_n_o <= !(req_more || (st == ST_PARK) || (st == ST_IMPL));
      end else begin
        br_n_o <= !pending;
      end
    end
  end

endmodule
