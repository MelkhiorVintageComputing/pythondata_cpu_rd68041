// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// Top level: the MC68040's pins (doc/pinout.md), split into _i/_o/_oe.
//
// M6: the integer unit's walking skeleton (rd68041_core), the bus unit with its
// snooper, and the reset; no caches yet, so no cache answers a snoop.

module rd68041_top #(
    // Whether the floating-point unit is built: 1 is the MC68040, 0 the
    // MC68LC040, which takes the F-line exception on every floating-point
    // instruction (UM Appendix A). doc/plan.md, M11 and M12.
    parameter int FPU = 1
) (
    // Clocks and reset (UM 5.7, 5.9) -------------------------------------------
    input  logic        clk,        // BCLK; every register is on its rising edge
    input  logic        pclk,       // PCLK, not used: doc/pinout.md
    input  logic        rst_n,      // not an MC68040 pin: async init, doc/pinout.md
    input  logic        rsti_n_i,
    output logic        rsto_n_o,

    // Address bus (UM 5.1) -----------------------------------------------------
    input  logic [31:0] a_i,
    output logic [31:0] a_o,
    output logic        a_oe,

    // Data bus (UM 5.2) --------------------------------------------------------
    input  logic [31:0] d_i,
    output logic [31:0] d_o,
    output logic        d_oe,

    // Transfer attributes (UM 5.3) ---------------------------------------------
    input  logic  [1:0] tt_i,
    output logic  [1:0] tt_o,
    output logic  [2:0] tm_o,
    output logic  [1:0] tln_o,
    output logic  [1:0] upa_o,
    input  logic        rw_i,
    output logic        rw_o,
    input  logic  [1:0] siz_i,
    output logic  [1:0] siz_o,
    output logic        lock_n_o,
    output logic        locke_n_o,
    output logic        ciout_n_o,
    output logic        attr_oe,

    // Transfer control (UM 5.4) ------------------------------------------------
    input  logic        ts_n_i,
    output logic        ts_n_o,
    output logic        ts_oe,
    output logic        tip_n_o,
    output logic        tip_oe,
    input  logic        ta_n_i,
    output logic        ta_n_o,
    output logic        ta_oe,
    input  logic        tea_n_i,
    input  logic        tci_n_i,
    input  logic        tbi_n_i,
    input  logic        dle_i,      // DLE (JS0 on the MC68LC040): not used

    // Snoop control (UM 5.5) ---------------------------------------------------
    input  logic  [1:0] sc_i,
    output logic        mi_n_o,

    // Arbitration (UM 5.6) -----------------------------------------------------
    output logic        br_n_o,
    input  logic        bg_n_i,
    input  logic        bb_n_i,
    output logic        bb_n_o,
    output logic        bb_oe,

    // Processor control (UM 5.7, 5.10) -----------------------------------------
    input  logic        cdis_n_i,
    input  logic        mdis_n_i,

    // Interrupts (UM 5.8) ------------------------------------------------------
    input  logic  [2:0] ipl_n_i,
    output logic        ipend_n_o,
    input  logic        avec_n_i,

    // Status (UM 5.9.1), stubbed -----------------------------------------------
    output logic  [3:0] pst_o,

    // Test (UM 6), stubbed -----------------------------------------------------
    input  logic        tck_i,
    input  logic        tms_i,
    input  logic        tdi_i,
    input  logic        trst_n_i,
    output logic        tdo_o,
    output logic        tdo_oe
);

  // The processor reset (UM 7.10): RSTI synchronised, the 128-clock hold, the
  // modes sampled at reset.
  logic       crst;
  logic       mode_mux, mode_dle;
  logic [2:0] mode_ipl;

  rd68041_rstctl u_rstctl (
      .clk      (clk),
      .rst_n    (rst_n),
      .rsti_n_i (rsti_n_i),
      .cdis_n_i (cdis_n_i),
      .mdis_n_i (mdis_n_i),
      .ipl_n_i  (ipl_n_i),
      .crst     (crst),
      .mode_mux (mode_mux),
      .mode_dle (mode_dle),
      .mode_ipl (mode_ipl)
  );

  // The integer unit, with instruction fetch and the data memory unit (M6).
  logic         req_valid, req_ready, req_write, req_lock, req_locke, lock_seq, req_ciout;
  logic  [1:0]  req_tln, req_upa;
  logic [31:0]  req_addr;
  logic  [1:0]  req_siz, req_tt;
  logic  [2:0]  req_tm;
  logic [127:0] req_wdata;
  logic         retire, halted;
  logic [31:0]  retire_pc, retire_npc;
  logic [15:0]  sr;
  logic         rsp_valid, rsp_tci, rsp_avec, rsp_done, rsp_err;
  logic  [1:0]  rsp_beat, rsp_lw;
  logic [31:0]  rsp_data;
  logic         owner;
  // The snooper's port to the caches.
  logic [31:0]  sn_addr, sn_wdata, sn_rdata;
  logic  [3:0]  sn_wbe;
  logic         sn_wr, sn_dinv, sn_iinv, sn_dhit, sn_ddirty, sn_ihit, sn_rd;

  logic         ipend, rsto;

  rd68041_core #(.FPU(FPU)) u_core (
      .clk(clk), .rst_n(rst_n), .crst(crst),
      .req_valid(req_valid), .req_ready(req_ready), .req_addr(req_addr),
      .req_write(req_write), .req_siz(req_siz), .req_tt(req_tt), .req_tm(req_tm),
      .req_wdata(req_wdata), .req_lock(req_lock), .req_locke(req_locke),
      .req_tln(req_tln), .req_upa(req_upa), .req_ciout(req_ciout),
      .lock_seq(lock_seq), .rsp_valid(rsp_valid), .rsp_data(rsp_data),
      .rsp_lw(rsp_lw), .rsp_tci(rsp_tci),
      .rsp_done(rsp_done), .rsp_err(rsp_err), .rsp_avec(rsp_avec),
      .ipl_n(ipl_n_i), .ipend(ipend), .rsto(rsto),
      .sn_addr(sn_addr), .sn_dhit(sn_dhit), .sn_ddirty(sn_ddirty), .sn_ihit(sn_ihit),
      .sn_rdata(sn_rdata), .sn_wr(sn_wr), .sn_wbe(sn_wbe), .sn_wdata(sn_wdata),
      .sn_dinv(sn_dinv), .sn_iinv(sn_iinv), .sn_rd(sn_rd),
      .retire(retire), .retire_pc(retire_pc), .retire_npc(retire_npc),
      .halted(halted), .sr(sr));

  rd68041_biu u_biu (
      .clk       (clk),
      .rst_n     (rst_n),
      .crst      (crst),
      .req_valid (req_valid),
      .req_ready (req_ready),
      .req_addr  (req_addr),
      .req_write (req_write),
      .req_siz   (req_siz),
      .req_tt    (req_tt),
      .req_tm    (req_tm),
      .req_tln   (req_tln),
      .req_upa   (req_upa),
      .req_ciout (req_ciout),
      .req_lock  (req_lock),
      .req_locke (req_locke),
      .req_more  (1'b0),
      .req_wdata (req_wdata),
      .lock_seq  (lock_seq),
      .rsp_valid (rsp_valid),
      .rsp_beat  (rsp_beat),
      .rsp_lw    (rsp_lw),
      .rsp_data  (rsp_data),
      .rsp_tci   (rsp_tci),
      .rsp_avec  (rsp_avec),
      .rsp_done  (rsp_done),
      .rsp_err   (rsp_err),
      .owner     (owner),
      .a_i       (a_i),
      .a_o       (a_o),
      .a_oe      (a_oe),
      .d_i       (d_i),
      .d_o       (d_o),
      .d_oe      (d_oe),
      .tt_i      (tt_i),
      .tt_o      (tt_o),
      .tm_o      (tm_o),
      .tln_o     (tln_o),
      .upa_o     (upa_o),
      .rw_i      (rw_i),
      .rw_o      (rw_o),
      .siz_i     (siz_i),
      .siz_o     (siz_o),
      .lock_n_o  (lock_n_o),
      .locke_n_o (locke_n_o),
      .ciout_n_o (ciout_n_o),
      .attr_oe   (attr_oe),
      .ts_n_i    (ts_n_i),
      .ts_n_o    (ts_n_o),
      .tip_n_o   (tip_n_o),
      .tip_oe    (tip_oe),
      .ta_n_i    (ta_n_i),
      .ta_n_o    (ta_n_o),
      .ta_oe     (ta_oe),
      .tea_n_i   (tea_n_i),
      .tci_n_i   (tci_n_i),
      .tbi_n_i   (tbi_n_i),
      .avec_n_i  (avec_n_i),
      .br_n_o    (br_n_o),
      .bg_n_i    (bg_n_i),
      .bb_n_i    (bb_n_i),
      .bb_n_o    (bb_n_o),
      .bb_oe     (bb_oe),
      .sc_i      (sc_i),
      .mi_n_o    (mi_n_o),
      .sn_addr   (sn_addr),
      .sn_dhit   (sn_dhit),
      .sn_ddirty (sn_ddirty),
      .sn_ihit   (sn_ihit),
      .sn_rdata  (sn_rdata),
      .sn_wr     (sn_wr),
      .sn_wbe    (sn_wbe),
      .sn_wdata  (sn_wdata),
      .sn_dinv   (sn_dinv),
      .sn_iinv   (sn_iinv),
      .sn_rd     (sn_rd)
  );

  // TS shares the address group's enable (specification 38 names it with
  // them).
  assign ts_oe     = attr_oe;
  assign rsto_n_o  = !rsto;
  assign ipend_n_o = !ipend;
  assign pst_o     = rd68041_pkg::PST_USER_RUN;
  assign tdo_o     = 1'b0;
  assign tdo_oe    = 1'b0;

  // What M2 deliberately does not read. Each goes when the unit that needs it
  // arrives: the caches (M9), the core (M6), the FPU (M11).
  logic unused_ok;
  assign unused_ok = &{1'b0, pclk, dle_i,
                       tck_i, tms_i, tdi_i, trst_n_i, mode_mux,
                       mode_dle, mode_ipl, rsp_beat,
                       owner, retire, retire_pc, retire_npc, halted, sr};

endmodule
