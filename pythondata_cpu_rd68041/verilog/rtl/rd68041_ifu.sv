// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// Instruction fetch (IF1, IF2; doc/microarch.md) and the instruction cache
// (UM 4): the fetch address, the cache's lookups and fills, the instruction
// queue, and the decode stage's window on it.
//
// The queue is twelve words -- room for the longest instruction, eleven (PRM
// 2.2: MOVE with two full-format <ea>s). Decode sees all of it and how many
// words are there, and says how many it took. A redirect -- a taken branch,
// from EX, or a fetch restarted behind a serialising instruction -- empties
// the queue and fetches from the target.
//
// A fetch is looked up in the cache when the cache is enabled (CACR's IE) and
// the fetch address's mode, from ITT0/ITT1 (rd68041_attr), is cachable: the
// half-line it is in (UM 4.4.3, 4.7.1) is read on one clock and its tags
// compared on the next, a hit putting the half-line's words from the fetch
// address on into the queue. A miss reads the line (UM 4.6.1: a burst, the
// long word wanted first, wrapping), into the way that is invalid or, if none
// is, the one the replacement counter names (UM 4.1); the words in order from
// the fetch address go into the queue as they arrive, as far as there is room,
// and the line is made valid once its last long word is in -- unless the slave
// said TCI with the first, or a transfer ended in a bus error. A bus error on
// the first long word puts its words in the queue marked (q_err), and decode
// turns the instruction that needs one into an access fault (UM 8.2.1); on a
// later one the fill is just abandoned, and done again when the line is next
// wanted (UM 4.6.1). A fetch that is not cachable reads a long word, CIOUT
// asserted if its mode is cache-inhibited (UM 5.3.9).
//
// A redirect to an odd address fetches nothing and says so (q_aerr): the
// address error (UM 8.2.2).
//
// CINV and CPUSH of the instruction cache (ic_op; UM 4.2) invalidate a line, a
// page or everything; the cache holds no dirty data, so a push is an
// invalidation. The snooper (rd68041_snoop) looks up a snooped transfer's line
// on the cache's second port and invalidates it on a hit (UM Table 4-3).
//
// Translation (UM 3): each lookup's fetch address is translated as the
// lookup starts -- ITT0/ITT1, else with translation on the instruction ATC
// (rd68041_atc), else the logical address is the physical one -- and the
// physical page, mode and UPA held for its lookup, fill or long word. A miss
// in the ATC asks the data memory unit for a table search (iw_req), which
// hands back the entry (iw_fill); then the lookup is done again. An entry not
// resident, or a user fetch from a supervisor page, puts the words marked in
// the queue (q_err and q_atc), as a bus error does. PFLUSH and PTEST of a
// program space flush the ATC from the data memory unit (if_flush).

module rd68041_ifu (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          crst,

    input  logic          go,          // start (or restart) fetching at go_pc
    input  logic [31:0]   go_pc,
    input  logic          stop,        // fetch nothing more
    input  logic          sup,         // supervisor program space (TM 110), else user (010)
    input  logic          ic_en,       // CACR's IE
    input  logic [31:0]   itt0,
    input  logic [31:0]   itt1,
    input  logic          tc_e,        // TC's E
    input  logic          tc_p,        // TC's P

    // To decode.
    output logic [191:0]  q_win,       // the queue from its head, word 0 in 191:176
    output logic  [3:0]   q_cnt,       // how many words are there (0-12)
    output logic [31:0]   q_pc,        // the head word's address
    output logic [11:0]   q_err,       // which of them came from a faulted fetch (bit k: word k)
    output logic [11:0]   q_atc,       // ... faulted in the ATC (not resident, or supervisor)
    output logic          q_aerr,      // fetching was redirected to an odd address
    input  logic  [3:0]   q_pop,       // words decode takes this clock (0-12)

    // The bus, through the arbiter.
    output logic          f_valid,
    input  logic          f_ready,
    output logic [31:0]   f_addr,
    output logic  [2:0]   f_tm,
    output logic  [1:0]   f_siz,       // a long word, or a line
    output logic          f_ci,        // CIOUT
    output logic  [1:0]   f_upa,
    input  logic          f_rsp,       // a response to our read
    input  logic  [1:0]   f_lw,        // ... its long word's A3-A2
    input  logic [31:0]   f_data,
    input  logic          f_tci,
    input  logic          f_done,
    input  logic          f_err,

    // CINV and CPUSH (UM 4.2): one clock with the scope; busy until done.
    input  logic          ic_op,
    input  logic  [1:0]   ic_scope,    // 01 a line, 10 a page, 11 all
    input  logic [31:0]   ic_addr,     // the line's or the page's physical address
    input  logic          ic_p8k,      // TC's P: 8 KB pages
    output logic          ic_busy,

    // The ATC's flushes and table searches, from and by the data memory unit.
    input  logic          if_flush,
    input  logic  [1:0]   if_fl_kind,  // 0 page nonglobal, 1 page, 2 all nonglobal, 3 all
    input  logic [31:0]   if_fl_la,
    input  logic          if_fl_s,
    output logic          iw_req,
    output logic [31:0]   iw_la,
    output logic          iw_s,
    input  logic          iw_fill,
    input  logic [31:12]  iw_fill_la,
    input  logic          iw_fill_s,
    input  logic [31:0]   iw_entry,    // {PA31-PA12, G, U1, U0, S, CM, M, W, R, 000}
    input  logic          iw_done,

    // The snooper.
    input  logic [31:0]   sn_addr,
    output logic          sn_ihit,
    input  logic          sn_iinv
);

  localparam int QN = 12;

  logic [15:0] q [0:QN-1];
  logic        qe [0:QN-1];     // ... from a fetch that faulted
  logic        qt [0:QN-1];     // ... in the ATC
  logic  [3:0] head, cnt;       // the head index, the words held
  logic [31:0] hpc;             // the head word's address
  logic [31:0] fa;              // the next word to fetch
  logic        on;              // fetching
  logic        aerr;            // redirected to an odd address: nothing to fetch

  // The window.
  always_comb begin
    for (int k = 0; k < QN; k++) begin
      logic [3:0] i;
      i = 4'((int'(head) + k) % QN);
      q_win[191 - 16*k -: 16] = q[i];
      q_err[k] = qe[i];
      q_atc[k] = qt[i];
    end
  end
  assign q_cnt  = cnt;
  assign q_pc   = hpc;
  assign q_aerr = aerr;

  // --- the fetch address's attributes, and the cache ----------------------------
  // The state of a fetch: a lookup, its result, a line fill, or a long word.
  //   F_WALK  the ATC missed: a table search, by the data memory unit
  typedef enum logic [2:0] { F_IDLE, F_LOOK, F_INV, F_FILL, F_LONG, F_WALK } f_e;
  f_e          fs;
  logic [31:0] la;              // the address looked up or being filled (logical)
  logic [31:12] lp;             // ... its physical page
  logic  [1:0] lcm, lupa;       // ... its mode and UPA

  // The fetch address's translation.
  logic [1:0] tt_cm, tt_upa;
  logic       f_wp, f_tt;       // write protection means nothing to a fetch
  rd68041_attr u_attr (
      .addr(fa[31:24]), .sup(sup), .ttr0(itt0), .ttr1(itt1),
      .cm(tt_cm), .upa(tt_upa), .wp(f_wp), .tt(f_tt));
  logic          lk_hit, lk_sprot, lk_m, lk_w, lk_r;
  logic [31:12]  lk_pa;
  logic  [1:0]   lk_u, lk_cm;
  wire           lk_count;
  rd68041_atc u_iatc (
      .clk(clk), .rst_n(rst_n), .crst(crst), .page8k(tc_p),
      .lk_la(if_flush ? if_fl_la[31:12] : fa[31:12]), .lk_s(if_flush ? if_fl_s : sup),
      .lk_count(lk_count), .lk_hit(lk_hit), .lk_pa(lk_pa),
      .lk_u(lk_u), .lk_sprot(lk_sprot), .lk_cm(lk_cm), .lk_m(lk_m), .lk_w(lk_w), .lk_r(lk_r),
      .fill_we(iw_fill), .fill_la(iw_fill_la), .fill_s(iw_fill_s), .fill_g(iw_entry[11]),
      .fill_pa(iw_entry[31:12]), .fill_u(iw_entry[10:9]), .fill_sprot(iw_entry[8]),
      .fill_cm(iw_entry[7:6]), .fill_m(iw_entry[5]), .fill_w(iw_entry[4]),
      .fill_r(iw_entry[3]),
      .fl_all(if_flush && if_fl_kind[1]), .fl_page(if_flush && !if_fl_kind[1]),
      .fl_nonglobal(if_flush && !if_fl_kind[0]));
  wire          t_ph   = f_tt || !tc_e;
  wire [31:12]  t_pa   = t_ph ? fa[31:12] : lk_pa;
  wire  [1:0]   t_cm   = f_tt ? tt_cm : (tc_e ? lk_cm : 2'b00);
  wire  [1:0]   t_upa  = f_tt ? tt_upa : (tc_e ? lk_u : 2'b00);
  wire          t_walk = !t_ph && !lk_hit;
  wire          t_flt  = !t_ph && lk_hit && (!lk_r || (lk_sprot && !sup));
  wire cachable = ic_en && !t_cm[1];
  assign f_upa  = lupa;
  assign iw_req = (fs == F_WALK);
  assign iw_la  = fa;
  assign iw_s   = sup;
  logic  [1:0] vway;            // the way being filled
  logic        busy;            // the transfer is on the bus
  logic        drop;            // a redirect came: what arrives is not queued
  logic        full;            // the fill has stopped queuing (room, or a gap)
  logic        ftci, ferr;      // the fill was cache-inhibited, or faulted
  logic  [1:0] rnd;             // the replacement counter (UM 4.1)
  logic        ferr_beat;       // a long word of the fill has come already
  logic        cachable_q;      // the lookup was of a cachable address

  logic          p0_hit, p1_hit, p2_valid;
  logic   [1:0]  p0_way, p1_way;
  logic   [3:0]  p0_valid;
  logic  [31:10] p2_tag;
  logic  [255:0] a_rdata;
  logic   [5:0]  p0_set, p2_set, t_set;
  logic  [31:10] p0_ptag, t_tag;
  logic   [1:0]  p2_way, t_way;
  logic          t_we, t_valid, inv_all;
  logic   [6:0]  a_addr, b_addr;
  logic   [7:0]  b_we;
  logic   [3:0]  nd0, nd1, nd2;     // no dirty bits in this cache
  logic  [255:0] b_rdata;           // nothing is read back from it

  // --- CINV and CPUSH --------------------------------------------------------
  // A line: its set looked up on port 0. A page: every set and way, on port 2.
  typedef enum logic [1:0] { M_IDLE, M_LINE, M_PAGE } m_e;
  m_e          ms;
  logic  [7:0] mk;              // the page walk: set and way
  logic [31:0] ma;
  logic        mp8k;
  assign ic_busy = (ms != M_IDLE);

  wire page_hit = p2_valid && (mp8k ? (p2_tag[31:13] == ma[31:13]) : (p2_tag[31:12] == ma[31:12]));

  // Port 0: the fetch's lookup, or CINV's line once no lookup is in flight.
  wire   m_p0    = (ms == M_LINE) && (fs != F_LOOK);
  assign p0_set  = m_p0 ? ma[9:4] : la[9:4];
  assign p0_ptag = m_p0 ? ma[31:10] : {lp, la[11:10]};
  assign p2_set  = mk[7:2];
  assign p2_way  = mk[1:0];

  // The tag port: the snooper first, then CINV, then the fill's own.
  wire        sn_t    = sn_iinv && p1_hit;
  wire        m_t     = (m_p0 && p0_hit) || (ms == M_PAGE && page_hit);
  wire        fill_t  = (fs == F_INV) || (fs == F_FILL && busy && f_done && !ferr && !f_err && !ftci);
  always_comb begin
    t_we = 1'b0; t_set = la[9:4]; t_way = vway; t_tag = {lp, la[11:10]}; t_valid = 1'b0;
    if (sn_t) begin
      t_we = 1'b1; t_set = sn_addr[9:4]; t_way = p1_way;
    end else if (m_t) begin
      t_we = 1'b1;
      t_set = (ms == M_LINE) ? ma[9:4] : mk[7:2];
      t_way = (ms == M_LINE) ? p0_way : mk[1:0];
    end else if (fill_t) begin
      t_we = 1'b1; t_valid = (fs == F_FILL);
    end
  end
  wire t_blocked = sn_t || m_t;     // the fill's tag write must wait
  assign inv_all = ic_op && (ic_scope == 2'b11);
  assign sn_ihit = p1_hit;

  // Data: port A reads the half-line the fetch address is in; port B writes
  // the fill's long words.
  assign a_addr = fa[9:3];
  assign b_addr = {la[9:4], f_lw[1]};
  assign b_we   = (fs == F_FILL && f_rsp && busy) ? (f_lw[0] ? 8'h0F : 8'hF0) : 8'h00;

  rd68041_cache_arr #(.DIRTY(1'b0)) u_arr (
      .clk(clk), .rst_n(rst_n), .crst(crst),
      .p0_set(p0_set), .p0_ptag(p0_ptag), .p0_hit(p0_hit), .p0_way(p0_way),
      .p0_dirty(nd0), .p0_valid(p0_valid),
      .p1_set(sn_addr[9:4]), .p1_ptag(sn_addr[31:10]), .p1_hit(p1_hit), .p1_way(p1_way),
      .p1_dirty(nd1),
      .p2_set(p2_set), .p2_way(p2_way), .p2_tag(p2_tag), .p2_valid(p2_valid), .p2_dirty(nd2),
      .t_we(t_we), .t_set(t_set), .t_way(t_way), .t_tag(t_tag), .t_valid(t_valid),
      .d_we(1'b0), .d_set(6'd0), .d_way(2'd0), .d_bits(4'd0), .inv_all(inv_all),
      .a_addr(a_addr), .a_rdata(a_rdata),
      .b_addr(b_addr), .b_way(vway), .b_we(b_we), .b_wdata({f_data, f_data}), .b_rdata(b_rdata));

  // --- what goes into the queue this clock ---------------------------------------
  // Up to four words, from a hit's half-line or a response's long word.
  wire [63:0] hl     = a_rdata[64*p0_way +: 64];
  wire  [3:0] room   = 4'(QN) - cnt + q_pop;
  // A hit: the half-line's words from la's on.
  wire  [2:0] hit_n  = 3'd4 - {1'b0, la[2:1]};
  wire        hit_in = (fs == F_LOOK) && !drop && cachable_q && p0_hit && ({1'b0, room} >= {2'b00, hit_n});
  // A long word: two words, or one when the fetch address is its second half.
  wire        lw_seq = !drop && (f_lw == fa[3:2]) &&
                       (fs == F_LONG || (!full && fa[31:4] == la[31:4]));
  wire  [2:0] lw_n   = fa[1] ? 3'd1 : 3'd2;
  wire        lw_in  = f_rsp && busy && lw_seq && ({1'b0, room} >= {2'b00, lw_n});
  // A fault on a fill's first long word, or on a long word's fetch: its words,
  // marked.
  wire        err_in = busy && f_done && f_err && !drop && !full &&
                       (fs == F_LONG || (!ferr_beat && fa[31:2] == la[31:2])) &&
                       ({1'b0, room} >= {2'b00, lw_n});
  // ... or an ATC fault: the words, marked, without a transfer.
  wire        want_lk;
  wire        atc_in = want_lk && t_flt;
  assign lk_count = want_lk && !t_ph;

  logic [2:0]  in_n;
  logic [15:0] in_w [0:3];
  logic        in_e, in_t;
  always_comb begin
    in_n = 3'd0; in_e = 1'b0; in_t = 1'b0;
    for (int k = 0; k < 4; k++) in_w[k] = 16'h0;
    if (hit_in) begin
      in_n = hit_n;
      for (int k = 0; k < 4; k++)
        in_w[k] = hl[63 - 16*(int'(la[2:1]) + k) -: 16];
    end else if (atc_in) begin
      in_n = lw_n;
      in_e = 1'b1;
      in_t = 1'b1;
    end else if (lw_in || err_in) begin
      in_n = lw_n;
      in_e = err_in;
      in_w[0] = fa[1] ? f_data[15:0] : f_data[31:16];
      in_w[1] = f_data[15:0];
    end
  end

  // Computed here, not as variables inside the clocked process: yosys keeps
  // such a variable as a register (doc/coding-standard.md).
  wire [3:0] tail    = 4'((int'(head) + int'(cnt)) % QN);
  // Where the k-th word goes, unsigned: a size cast of an int is signed in
  // iverilog, and index 8 as a signed four bits is -8.
  logic [3:0] wi [0:3];
  always_comb for (int k = 0; k < 4; k++) wi[k] = 4'((int'(tail) + k) % QN);
  wire [3:0] cnt_nxt = cnt - q_pop + {1'b0, in_n};
  wire [31:0] fa_nxt = fa + {28'h0, in_n, 1'b0};

  // The way a fill takes: the first invalid, else the counter's.
  function automatic logic [1:0] victim(input logic [3:0] v, input logic [1:0] r);
    victim = r;
    for (int w = 3; w >= 0; w--) if (!v[w]) victim = 2'(w);
  endfunction
  wire [1:0] pick = victim(p0_valid, rnd);

  // --- the bus ---------------------------------------------------------------------
  assign want_lk = on && !stop && !aerr && !go && (fs == F_IDLE) && (room >= 4'd4);
  assign f_valid = ((fs == F_FILL) || (fs == F_LONG)) && !busy;
  assign f_addr  = (fs == F_FILL) ? {lp, la[11:2], 2'b00} : {lp, fa[11:2], 2'b00};
  assign f_siz   = (fs == F_FILL) ? rd68041_pkg::SIZ_LINE : rd68041_pkg::SIZ_LONG;
  assign f_ci    = (fs == F_LONG) && lcm[1];
  assign f_tm    = sup ? rd68041_pkg::TM_SUP_CODE : rd68041_pkg::TM_USER_CODE;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < QN; i++) begin q[i] <= 16'h0000; qe[i] <= 1'b0; qt[i] <= 1'b0; end
      lp    <= '0;
      lcm   <= '0;
      lupa  <= '0;
      aerr  <= 1'b0;
      head  <= '0;
      cnt   <= '0;
      hpc   <= '0;
      fa    <= '0;
      on    <= 1'b0;
      fs    <= F_IDLE;
      la    <= '0;
      vway  <= '0;
      busy  <= 1'b0;
      drop  <= 1'b0;
      full  <= 1'b0;
      ftci  <= 1'b0;
      ferr  <= 1'b0;
      ferr_beat  <= 1'b0;
      cachable_q <= 1'b0;
      rnd   <= '0;
      ms    <= M_IDLE;
      mk    <= '0;
      ma    <= '0;
      mp8k  <= 1'b0;
    end else if (crst) begin
      head  <= '0;
      cnt   <= '0;
      on    <= 1'b0;
      aerr  <= 1'b0;
      fs    <= F_IDLE;
      busy  <= 1'b0;
      drop  <= 1'b0;
      ms    <= M_IDLE;
    end else begin
      // --- the queue ---------------------------------------------------------
      if (q_pop != 4'd0) begin
        head <= 4'((int'(head) + int'(q_pop)) % QN);
        hpc  <= hpc + {27'h0, q_pop, 1'b0};
      end
      // Written out: a loop variable in this process would be a register to
      // yosys (doc/coding-standard.md).
      if (in_n > 3'd0) begin q[wi[0]] <= in_w[0]; qe[wi[0]] <= in_e; qt[wi[0]] <= in_t; end
      if (in_n > 3'd1) begin q[wi[1]] <= in_w[1]; qe[wi[1]] <= in_e; qt[wi[1]] <= in_t; end
      if (in_n > 3'd2) begin q[wi[2]] <= in_w[2]; qe[wi[2]] <= in_e; qt[wi[2]] <= in_t; end
      if (in_n > 3'd3) begin q[wi[3]] <= in_w[3]; qe[wi[3]] <= in_e; qt[wi[3]] <= in_t; end
      cnt <= cnt_nxt;
      if (in_n != 3'd0) fa <= fa_nxt;

      // --- the fetch -----------------------------------------------------------
      case (fs)
        F_IDLE: begin
          if (want_lk && t_walk) begin
            fs <= F_WALK;
          end else if (want_lk && !t_flt) begin
            lp   <= t_pa;
            lcm  <= t_cm;
            lupa <= t_upa;
            if (cachable) begin
              // The half-line at fa is being read; its tags are compared on
              // the next clock.
              fs <= F_LOOK;
              la <= fa;
              cachable_q <= 1'b1;
              rnd <= rnd + 2'd1;
            end else begin
              fs <= F_LONG;
            end
          end
        end
        F_LOOK: begin
          if (drop || hit_in) begin
            fs <= F_IDLE;
          end else if (p0_hit) begin
            fs <= F_IDLE;                    // a hit with no room: look again
          end else begin
            // A miss: the line is filled into a way made invalid first.
            fs   <= F_INV;
            vway <= pick;
          end
        end
        F_INV: begin
          if (!t_blocked) begin
            fs    <= F_FILL;
            full  <= 1'b0;
            ftci  <= 1'b0;
            ferr  <= 1'b0;
            ferr_beat <= 1'b0;
          end
        end
        F_FILL, F_LONG: begin
          if (f_valid && f_ready) busy <= 1'b1;
          if (busy && f_rsp) begin
            ferr_beat <= 1'b1;
            if (fs == F_FILL && !ferr_beat && f_tci) ftci <= 1'b1;
            // The words in order stop at the first that does not fit, or at
            // the wrap.
            if (fs == F_FILL && !lw_in) full <= 1'b1;
          end
          if (busy && f_done) begin
            busy <= 1'b0;
            drop <= 1'b0;
            if (f_err) ferr <= 1'b1;
            // A fill whose tag write lost to the snooper or CINV leaves the
            // line invalid, to be fetched again when it is next wanted.
            fs <= F_IDLE;
          end
        end
        F_WALK: if (iw_done) fs <= F_IDLE;
        default: fs <= F_IDLE;
      endcase
      if (fs == F_LOOK) drop <= 1'b0;

      if (go) begin
        // A redirect: everything queued goes; a transfer on the bus finishes,
        // its words not queued (a fill still fills the cache).
        head <= '0;
        cnt  <= '0;
        hpc  <= go_pc;
        fa   <= go_pc;
        on   <= 1'b1;
        aerr <= go_pc[0];
        if (fs == F_LOOK) drop <= 1'b1;
        if (busy && !f_done) drop <= 1'b1;
        if (f_valid && f_ready) drop <= 1'b1;
        // A long word not yet asked for is not wanted.
        if (fs == F_LONG && !busy && !(f_valid && f_ready)) fs <= F_IDLE;
      end

      // --- CINV and CPUSH ------------------------------------------------------
      case (ms)
        M_IDLE: begin
          if (ic_op && ic_scope == 2'b01) begin ms <= M_LINE; ma <= ic_addr; end
          if (ic_op && ic_scope == 2'b10) begin ms <= M_PAGE; ma <= ic_addr; mk <= '0; mp8k <= ic_p8k; end
        end
        M_LINE: if (m_p0 && !sn_t) ms <= M_IDLE;
        M_PAGE: if (!sn_t) begin
          mk <= mk + 8'd1;
          if (mk == 8'hFF) ms <= M_IDLE;
        end
        default: ms <= M_IDLE;
      endcase
    end
  end

  logic unused_ok;
  assign unused_ok = &{1'b0, f_wp, nd0, nd1, nd2, b_rdata, sn_addr[3:0], la[0], ma[3:0],
                       lcm[0], p2_tag[11:10], lk_m, lk_w, if_fl_la[11:0],
                       iw_entry[2:0]};

endmodule
