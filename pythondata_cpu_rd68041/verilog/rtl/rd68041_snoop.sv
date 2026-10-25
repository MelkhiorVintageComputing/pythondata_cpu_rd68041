// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The bus snooper: an alternate master's transfers, watched and answered (UM
// 7.9, UM 4.4, 4.5, Tables 4-1, 4-3, 4-4).
//
// When the core is not bus master and TS is asserted, the snooper latches the
// address, SIZ, TT, R/W and SC on that edge (UM 7.9: "the processor latches
// the level on the byte offset, SIZx, TMx, and R/W signals during the rising
// edge of BCLK for which TS is first asserted"). Then:
//
//   - not snoopable (TT not normal or MOVE16, or SC = 00 or 11): MI is negated
//     at once and memory answers (UM 7.9.1);
//   - snoopable: MI stays asserted while the caches are looked up -- one clock
//     here, "the equivalent of two wait states" for memory, UM 7.9.2's best
//     case. A dirty data-cache line that the snoop operation says to source
//     (a read with SC = 01 or 10) or sink (a write with SC = 01 that is not a
//     line) is answered by the core as a slave: MI held, TA driven, the data
//     from or into the cache (UM 7.9.3, 7.9.4). Everything else negates MI and
//     lets memory answer, with the cache actions of Tables 4-3 and 4-4: a
//     snooped write that hits invalidates the line, a snooped read with SC =
//     10 invalidates it, and an instruction-cache line is invalidated by both.
//
// The snooper then counts the transfer's terminations -- TA, TEA, TA with TEA,
// and TBI on a line's first long word -- to know when it is over; while MI is
// asserted it ignores them (UM 5.5.2). MI is asserted again between transfers
// ("no matter what the values are on the SCx and TTx signals, MI is asserted
// between bus cycles", UM 7.9.1).
//
// The caches answer through the sn_* port. Its lookup is combinational on the
// address the snooper holds (the tags are in LUT RAM), and its data one clock
// behind the address, as a block RAM is:
//
//   sn_addr   the line and long word looked at, held from the edge TS is seen
//   sn_dhit, sn_ddirty, sn_ihit   for sn_addr's line, combinationally
//   sn_rdata  the long word sn_addr named on the clock before, from the way
//             that hit then
//   sn_wr, sn_wbe, sn_wdata       sink: write the hit line, set its D bit
//   sn_dinv, sn_iinv              invalidate the hit line
//
// The push buffer is snooped as part of the data cache (M9).

module rd68041_snoop (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        crst,
    input  logic        owner,       // the core is master: nothing to snoop

    // The bus, as sampled.
    input  logic        ts_n_i,
    input  logic [31:0] a_i,
    input  logic [31:0] d_i,
    input  logic  [1:0] tt_i,
    input  logic        rw_i,
    input  logic  [1:0] siz_i,
    input  logic  [1:0] sc_i,
    input  logic        ta_n_i,
    input  logic        tea_n_i,
    input  logic        tbi_n_i,

    // To the bus unit's pins.
    output logic        mi_release_d,  // MI to be negated next clock
    output logic [31:0] d_o,
    output logic        d_oe,
    output logic        ta_n_o,
    output logic        ta_on_d,       // TA to be driven next clock (rd68041_late_release)

    // The caches.
    output logic [31:0] sn_addr,
    input  logic        sn_dhit,
    input  logic        sn_ddirty,
    input  logic        sn_ihit,
    input  logic [31:0] sn_rdata,
    output logic        sn_wr,
    output logic  [3:0] sn_wbe,
    output logic [31:0] sn_wdata,
    output logic        sn_dinv,
    output logic        sn_iinv,
    output logic        sn_rd          // it reads the data cache at sn_addr this clock
);

  typedef enum logic [2:0] {
    SN_IDLE = 3'd0,   // waiting for an alternate master's TS
    SN_LOOK = 3'd1,   // the caches are being looked up; MI asserted
    SN_PASS = 3'd2,   // memory answers; counting the terminations
    SN_SRC  = 3'd3,   // intervention: the core sources the data
    SN_SINK = 3'd4    // intervention: the core sinks the data
  } sn_e;

  sn_e st;

  logic [31:0] x_a;
  logic  [1:0] x_siz, x_tt, x_sc;
  logic        x_rw;
  logic  [1:0] beat;          // terminations counted, or long words sourced
  logic  [1:0] rd_k;          // the long word whose address is presented
  logic        inv_after;     // invalidate once the sourced transfer is over

  wire ts      = !ts_n_i && !owner;
  wire ta      = !ta_n_i;
  wire tea     = !tea_n_i;
  wire tbi     = !tbi_n_i;
  wire is_line = (x_siz == rd68041_pkg::SIZ_LINE);

  // UM 5.3.1: "only normal and MOVE16 accesses can be snooped"; Table 4-1: SC
  // = 00 inhibits snooping and 11 is reserved, also inhibited.
  wire snoopable_now = !tt_i[1] && (sc_i == 2'b01 || sc_i == 2'b10);

  // The decision on the lookup (SN_LOOK), Table 4-4.
  wire src_now  = sn_dhit && sn_ddirty &&  x_rw;                      // D9, D10
  wire sink_now = sn_dhit && sn_ddirty && !x_rw && (x_sc == 2'b01) && !is_line;  // D12

  // A memory-answered transfer is over (UM 7.9: "the processor monitors the
  // levels of TA, TEA, and TBI to detect normal, bus error, retry, and
  // burst-inhibited terminations"; an alternate master's burst-inhibited line
  // is then snooped as four long-word transfers).
  wire pass_end = (st == SN_PASS) &&
                  (tea || (ta && (!is_line || tbi || (beat == 2'd3))));

  // The sourced transfer's last TA is on the bus at this edge.
  wire src_end  = (st == SN_SRC) && !ta_n_o &&
                  (!is_line || (beat == 2'd3));

  // The data cache's port B is read at sn_addr while the lines are looked up
  // and while a dirty line is sourced.
  assign sn_rd = (st == SN_LOOK) || (st == SN_SRC);

  // The lookup: the latched address, the long word being read.
  assign sn_addr = {x_a[31:4], (st == SN_SRC) ? (x_a[3:2] + rd_k) : x_a[3:2], 2'b00};

  // UM Table 7-1: the bytes a byte, word or long-word write occupies.
  function automatic logic [3:0] lanes(input logic [1:0] s, input logic [1:0] a10);
    case (s)
      rd68041_pkg::SIZ_BYTE: lanes = 4'b1000 >> a10;
      rd68041_pkg::SIZ_WORD: lanes = a10[1] ? 4'b0011 : 4'b1100;
      default:               lanes = 4'b1111;
    endcase
  endfunction

  // The next value of MI's release and TA's drive, for the bus unit's pins.
  always_comb begin
    mi_release_d = 1'b0;
    ta_on_d      = 1'b0;
    case (st)
      SN_IDLE: mi_release_d = ts && !snoopable_now;
      SN_LOOK: begin
                 mi_release_d = !(src_now || sink_now);
                 ta_on_d      = sink_now;
               end
      SN_PASS: mi_release_d = !pass_end;
      SN_SRC:  ta_on_d      = !src_end;
      SN_SINK: ta_on_d      = (ta_n_o == 1'b1);     // TA for one clock, then released
      default: ;
    endcase
    if (crst || owner) begin
      mi_release_d = 1'b0;
      ta_on_d      = 1'b0;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st        <= SN_IDLE;
      x_a       <= '0;
      x_siz     <= rd68041_pkg::SIZ_LONG;
      x_tt      <= rd68041_pkg::TT_NORMAL;
      x_sc      <= 2'b00;
      x_rw      <= 1'b1;
      beat      <= '0;
      rd_k      <= '0;
      inv_after <= 1'b0;
      d_o       <= '0;
      d_oe      <= 1'b0;
      ta_n_o    <= 1'b1;
      sn_wr     <= 1'b0;
      sn_wbe    <= '0;
      sn_wdata  <= '0;
      sn_dinv   <= 1'b0;
      sn_iinv   <= 1'b0;
    end else if (crst || owner) begin
      st        <= SN_IDLE;
      beat      <= '0;
      rd_k      <= '0;
      inv_after <= 1'b0;
      d_oe      <= 1'b0;
      ta_n_o    <= 1'b1;
      sn_wr     <= 1'b0;
      sn_dinv   <= 1'b0;
      sn_iinv   <= 1'b0;
    end else begin
      sn_wr   <= 1'b0;
      sn_dinv <= 1'b0;
      sn_iinv <= 1'b0;
      case (st)
        SN_IDLE: begin
          if (ts) begin
            x_a   <= a_i;
            x_siz <= siz_i;
            x_tt  <= tt_i;
            x_rw  <= rw_i;
            x_sc  <= sc_i;
            beat  <= '0;
            rd_k  <= '0;
            if (snoopable_now) st <= SN_LOOK;
            else               st <= SN_PASS;
          end
        end

        SN_LOOK: begin
          if (src_now) begin
            // UM 7.9.3: inhibit memory and supply the data. The long word at
            // the transfer's own A3-A2 was presented this clock; the next one
            // is presented from here on.
            st        <= SN_SRC;
            rd_k      <= 2'd1;
            inv_after <= (x_sc == 2'b10);                // D10
          end else if (sink_now) begin
            // UM 7.9.4: inhibit memory and take the data. TA from the next
            // clock; the master's data is on the bus from C2 (UM 7.4.3).
            st     <= SN_SINK;
            ta_n_o <= 1'b0;
          end else begin
            st <= SN_PASS;
            // Table 4-4: V10, V11, V12, V13, D11, D13 invalidate the line;
            // Table 4-3: V5, V6 the instruction-cache line.
            sn_dinv <= sn_dhit && (!x_rw || (x_sc == 2'b10));
            sn_iinv <= sn_ihit && (!x_rw || (x_sc == 2'b10));
          end
        end

        SN_PASS: begin
          if (ta || tea) beat <= beat + 2'd1;
          if (pass_end) st <= SN_IDLE;
        end

        SN_SRC: begin
          // One long word a clock. The edge that enters SN_SRC presented the
          // transfer's own long word; each edge here puts the one presented on
          // the clock before (sn_rdata) on the bus with TA, and presents the
          // next. A TA that was on the bus at this edge was sampled by the
          // master: that long word is moved.
          //
          //   edge   E2 (decide)  E3       E4       E5       E6       E7
          //   addr   lw1          lw2      lw3      ...
          //   D, TA  --           lw0      lw1      lw2      lw3      released
          if (src_end) begin
            st        <= SN_IDLE;
            d_oe      <= 1'b0;
            ta_n_o    <= 1'b1;
            // D10: invalidated once the line has been supplied (UM 4.4.3:
            // "after the access, even if it is dirty").
            sn_dinv   <= inv_after;
            sn_iinv   <= inv_after && sn_ihit;
            inv_after <= 1'b0;
          end else begin
            d_o    <= sn_rdata;
            d_oe   <= 1'b1;
            ta_n_o <= 1'b0;
            rd_k   <= rd_k + 2'd1;
            if (!ta_n_o) beat <= beat + 2'd1;
          end
        end

        SN_SINK: begin
          if (!ta_n_o) begin
            // The edge the master sampled our TA on: its data is on the bus.
            sn_wdata <= d_i;
            sn_wbe   <= lanes(x_siz, x_a[1:0]);
            sn_wr    <= 1'b1;
            ta_n_o   <= 1'b1;
            st       <= SN_IDLE;
            // Table 4-3: a snooped write invalidates an instruction-cache line.
            sn_iinv  <= sn_ihit;
          end
        end

        default: st <= SN_IDLE;
      endcase
    end
  end

  logic unused_ok;
  assign unused_ok = &{1'b0, x_tt};

endmodule
