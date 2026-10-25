// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// One address translation cache: 64 entries, four-way set-associative, 16 sets
// (UM 3.3, Figure 3-20). The instruction and data MMUs each have one.
//
// UM 3.3: "the MMU uses the four bits of the logical address located just above
// the page offset (LA16-LA13 for 8K pages, LA15-LA12 for 4K pages) to index into
// the ATC. The tags are compared with the remaining upper bits of the logical
// address and FC2." The tag holds LA31-LA16 and FC2; for 8 KB pages its least
// significant bit is ignored, and PA12 comes from LA12 rather than the entry.
//
// The lookup is combinational on lk_la, which the caller holds in a register:
// the entries are LUT RAM, read without a clock, so a lookup and the cache
// lookup it runs beside both start from that one register (doc/microarch.md).
// The valid and global bits are flip-flops, so that PFLUSHA and PFLUSHAN clear
// them in one clock, and so that reset clears them (doc/divergences.md: the
// part's reset does not).
//
// Replacement (UM 3.3): the entry the fill's own page already has, if it has
// one -- a table search that sets M "writes over the old ATC entry" -- else an
// invalid entry of the set, else the entry a 2-bit counter names,
// "incremented for each ATC access".
//
// Flushing (UM 3.7.2): all entries, or all non-global ones, at once; or the
// entries a page address matches, given on the lookup port -- PFLUSH is
// serialising, so the lookup port is free -- optionally sparing global ones.

module rd68041_atc (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        crst,

    input  logic        page8k,       // TC P bit: 8 KB pages

    // Lookup ---------------------------------------------------------------
    input  logic [31:12] lk_la,
    input  logic        lk_s,         // FC2 of the access
    input  logic        lk_count,     // an access: advance the replacement counter
    output logic        lk_hit,
    output logic [31:12] lk_pa,
    output logic  [1:0] lk_u,         // U1-U0, to UPA1-UPA0
    output logic        lk_sprot,     // S: supervisor only
    output logic  [1:0] lk_cm,        // cache mode
    output logic        lk_m,         // modified
    output logic        lk_w,         // write protected
    output logic        lk_r,         // resident

    // Fill: an entry for fill_la's set, in the way the replacement rule picks
    input  logic        fill_we,
    input  logic [31:12] fill_la,
    input  logic        fill_s,
    input  logic        fill_g,
    input  logic [31:12] fill_pa,
    input  logic  [1:0] fill_u,
    input  logic        fill_sprot,
    input  logic  [1:0] fill_cm,
    input  logic        fill_m,
    input  logic        fill_w,
    input  logic        fill_r,

    // Flush ------------------------------------------------------------------
    input  logic        fl_all,       // every entry (PFLUSHA)
    input  logic        fl_page,      // the entries lk_la, lk_s match (PFLUSH)
    input  logic        fl_nonglobal  // ... sparing global ones (PFLUSHN, PFLUSHAN)
);

  // The stored entry: tag, then the page descriptor's image (Figure 3-21).
  localparam int TW = 16 + 1;                    // LA31-LA16, FC2
  localparam int EW = 20 + 2 + 1 + 2 + 1 + 1 + 1; // PA31-PA12, U, S, CM, M, W, R
  localparam int W  = TW + EW;

  function automatic logic [3:0] set_of(input logic [16:12] la, input logic p8);
    set_of = p8 ? la[16:13] : la[15:12];
  endfunction

  wire [3:0] lk_set   = set_of(lk_la[16:12], page8k);
  wire [3:0] fill_set = set_of(fill_la[16:12], page8k);

  logic [15:0] v_flat [0:3];  // valid, per way, a bit per set
  logic [15:0] g_flat [0:3];  // global

  logic [W-1:0] rd [0:3];      // way w's entry at lk_set
  logic [W-1:0] rdf [0:3];     // ... and at fill_set
  logic  [3:0]  hit_w;
  logic  [1:0]  ctr;
  logic  [1:0]  vic_way;      // the way a fill goes to

  // --- the entries: one LUT RAM per way, 16 x W, written on a clock edge ----
  for (genvar w = 0; w < 4; w++) begin : g_way
    logic [W-1:0] ram [0:15];
    logic         we;
    assign we = fill_we && (vic_way == w[1:0]);
    always_ff @(posedge clk) begin
      if (we) ram[fill_set] <= {fill_la[31:16], fill_s,
                                fill_pa, fill_u, fill_sprot, fill_cm, fill_m,
                                fill_w, fill_r};
    end
    assign rd[w] = ram[lk_set];
    assign rdf[w] = ram[fill_set];

    // UM 3.3: "all 16 bits of this field are used in the comparison ... when
    // the page size is 4 Kbytes. For 8-Kbytes pages, the least significant bit
    // of this field is ignored."
    wire [15:0] tag  = rd[w][W-1 -: 16];
    wire        tfc2 = rd[w][W-17];
    assign hit_w[w] = v_flat[w][lk_set] && (tfc2 == lk_s) &&
                      (tag[15:1] == lk_la[31:17]) &&
                      (page8k || (tag[0] == lk_la[16]));
  end

  // --- the hit way's entry -------------------------------------------------
  logic [EW-1:0] e;
  always_comb begin
    e = '0;
    for (int w = 0; w < 4; w++) if (hit_w[w]) e = e | rd[w][EW-1:0];
  end

  // The entry as the fill packs it: PA31-PA12 in 27:8, U1-U0 7:6, S 5, CM 4:3,
  // M 2, W 1, R 0.
  assign lk_hit   = |hit_w;
  // UM 3.3: with 8 KB pages "physical address bit 12 is driven by logical
  // address bit 12".
  assign lk_pa    = {e[27:9], page8k ? lk_la[12] : e[8]};
  assign lk_u     = e[7:6];
  assign lk_sprot = e[5];
  assign lk_cm    = e[4:3];
  assign lk_m     = e[2];
  assign lk_w     = e[1];
  assign lk_r     = e[0];

  // --- replacement -----------------------------------------------------------
  // The fill's page's own entry, compared at the fill's set.
  logic [3:0] own_w;
  for (genvar w = 0; w < 4; w++) begin : g_own
    wire [15:0]  ftag = rdf[w][W-1 -: 16];
    assign own_w[w] = v_flat[w][fill_set] && (rdf[w][W-17] == fill_s) &&
                      (ftag[15:1] == fill_la[31:17]) && (page8k || (ftag[0] == fill_la[16]));
  end
  always_comb begin
    vic_way = ctr;
    for (int w = 3; w >= 0; w--) if (!v_flat[w][fill_set]) vic_way = w[1:0];
    for (int w = 3; w >= 0; w--) if (own_w[w]) vic_way = w[1:0];
  end

  // --- valid and global bits, the counter ----------------------------------
  for (genvar w = 0; w < 4; w++) begin : g_vg
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        v_flat[w] <= '0;
        g_flat[w] <= '0;
      end else if (crst) begin
        v_flat[w] <= '0;
        g_flat[w] <= '0;
      end else begin
        if (fl_all) begin
          v_flat[w] <= fl_nonglobal ? (v_flat[w] & g_flat[w]) : 16'h0000;
        end else if (fl_page && hit_w[w] && !(fl_nonglobal && g_flat[w][lk_set])) begin
          v_flat[w][lk_set] <= 1'b0;
        end
        if (fill_we && vic_way == w[1:0]) begin
          v_flat[w][fill_set] <= 1'b1;
          g_flat[w][fill_set] <= fill_g;
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ctr <= 2'd0;
    end else if (lk_count) begin
      ctr <= ctr + 2'd1;
    end
  end

endmodule
