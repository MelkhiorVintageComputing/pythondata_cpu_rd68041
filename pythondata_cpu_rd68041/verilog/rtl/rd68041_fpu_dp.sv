// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- SystemVerilog MC68040
//
// The floating-point data path (doc/fpu.md): one operation at a time, a few
// clocks for an add, a multiply in four, a divide or a square root a bit a
// clock, every one ending in the same rounding and range check (UM 9.4, 9.5.1).
//
// The operands come unpacked (rd68041_fpu_pkg::fpn_t): a class, a sign, an
// exponent biased by 16383 and signed, a 64-bit mantissa with its integer bit.
// A denormalised or unnormalised operand never gets here -- it is an
// unsupported data type (UM 9.6.2) -- and the data path never makes one: a
// result too small for the rounding precision is an underflow, a nonmaskable
// trap on the MC68040 (UM 9.7.5.2) whose IEEE result the FPSP makes.
//
// Special operands (zeros, infinities, NaNs) are decided when the operation
// starts, from the operation tables (PRM 5) and the NaN rules (PRM 1.6.5; the
// destination's NaN when both are, as the MC68881 does, UM30 4.5.4):
// a signalling NaN is returned with its signalling bit set, and SNAN raised; an
// operand error makes the default NaN, all ones.
//
// The intermediate result is normalised into 67 bits -- the 64-bit mantissa,
// then guard, round and sticky (UM Figure 9-7) -- and rounded to the mantissa
// precision (24, 53 or 64 bits) in the mode asked for; underflow is the
// unrounded exponent below the range precision's smallest normal (an extended
// result may have the minimum exponent with its integer bit set, UM 9.7.5),
// overflow the rounded one above its largest. The unrounded intermediate is
// also an output: the access the FSAVE frame gives it (WBTEMP, FPTEMP).

module rd68041_fpu_dp
(
    input  logic         clk,
    input  logic         rst_n,
    input  logic         crst,

    input  logic         kill,       // abandon the operation under way
    input  logic         start_i,    // the inputs below are taken on this clock
    input  logic  [3:0]  op_i,       // D_*
    input  rd68041_fpu_pkg::fpn_t         a_i,        // the destination operand
    input  rd68041_fpu_pkg::fpn_t         b_i,        // the source operand
    input  logic  [1:0]  prec_i,     // the rounding precision: P_X, P_S, P_D
    input  logic  [1:0]  rmode_i,    // R_N, R_Z, R_M, R_P
    input  logic  [1:0]  isz_i,      // D_INT: 0 a byte, 1 a word, 2 a long

    output logic         busy,
    output logic         done,       // one clock: the outputs hold until the next start

    output rd68041_fpu_pkg::fpn_t         r,          // the result, rounded
    output logic         f_snan,     // the FPSR EXC bits it raises (UM 9.2.3.3)
    output logic         f_operr,
    output logic         f_ovfl,
    output logic         f_unfl,
    output logic         f_dz,
    output logic         f_inex,
    output logic         f_spec,     // the result came from the special-operand rules
    output logic         f_dnz,      // an underflow's denormalised result is a zero
    output logic [17:0]  i_exp,      // the intermediate: its exponent ...
    output logic [66:0]  i_man,      // ... and mantissa with guard, round, sticky, unrounded
    output logic [31:0]  r_int,      // D_INT: the integer, sign-extended from its size
    output logic         c_lt,       // D_CMP: dest < src, equal, unordered
    output logic         c_eq,
    output logic         c_un
);

  // ===========================================================================
  // The inputs, registered as the operation is started: it works from flops,
  // not from the operand selection in front of the unit (which, with the
  // alignment of an add behind it, was M11's critical path,
  // doc/size-and-speed.md). The operation proper starts a clock later.
  // ===========================================================================
  logic                  start;
  logic  [3:0]           op;
  rd68041_fpu_pkg::fpn_t a, b;
  logic  [1:0]           prec, rmode, isz;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start <= 1'b0; op <= '0; a <= '0; b <= '0; prec <= '0; rmode <= '0; isz <= '0;
    end else if (crst || kill) begin
      start <= 1'b0;
    end else begin
      start <= start_i;
      if (start_i) begin
        op <= op_i; a <= a_i; b <= b_i; prec <= prec_i; rmode <= rmode_i; isz <= isz_i;
      end
    end
  end

  // ===========================================================================
  // The operation
  // ===========================================================================
  typedef enum logic [3:0] { S_IDLE, S_ADD1, S_ADD2, S_MUL, S_DIV, S_SQRT, S_NORM, S_RND,
                             S_INT, S_DONE } s_e;
  s_e          s;
  logic  [3:0] q_op;
  logic  [1:0] q_prec, q_rm, q_isz;
  logic        q_sign;          // the result's sign
  logic [18:0] q_exp;           // the intermediate's exponent, before normalising
  logic [67:0] q_man;           // ... its mantissa: carry, integer, 63 fraction, G R S
  logic  [6:0] cnt;
  logic [63:0] q_a, q_b;        // the mantissas being worked on
  logic [127:0] acc;            // the product; the divide's and root's remainder
  logic [69:0] rem;
  logic [67:0] quo;
  logic        q_op_sub;        // add or subtract: the magnitudes are subtracted

  // ===========================================================================
  // The special operands
  // ===========================================================================
  // The default NaN, and a NaN with its signalling bit set.
  localparam logic [63:0] DNAN_M = 64'hFFFF_FFFF_FFFF_FFFF;
  function automatic rd68041_fpu_pkg::fpn_t mk(input logic [2:0] c, input logic sg,
                                               input logic [17:0] e, input logic [63:0] m);
    mk = {c, 1'b0, sg, e, m};
  endfunction
  function automatic rd68041_fpu_pkg::fpn_t quiet(input rd68041_fpu_pkg::fpn_t x);
    quiet = {x.cls, 1'b0, x.sign, x.exp, x.man | 64'h4000_0000_0000_0000};
  endfunction

  wire dyadic = (op == rd68041_fpu_pkg::D_ADD) || (op == rd68041_fpu_pkg::D_SUB) || (op == rd68041_fpu_pkg::D_MUL) || (op == rd68041_fpu_pkg::D_DIV) ||
                (op == rd68041_fpu_pkg::D_SMUL) || (op == rd68041_fpu_pkg::D_SDIV) || (op == rd68041_fpu_pkg::D_CMP);
  wire a_nan  = dyadic && (a.cls == rd68041_fpu_pkg::C_NAN);
  wire b_nan  = (b.cls == rd68041_fpu_pkg::C_NAN);
  wire any_sn = (a_nan && a.snan) || (b_nan && b.snan);
  wire bs_eff = (op == rd68041_fpu_pkg::D_SUB || op == rd68041_fpu_pkg::D_CMP) ? !b.sign : b.sign;   // the source's sign as added
  wire sx     = a.sign ^ b.sign;                                   // a product's, a quotient's

  logic        sp;              // the result is special
  rd68041_fpu_pkg::fpn_t        sp_r;
  logic        sp_operr, sp_dz;
  always_comb begin
    sp = 1'b1; sp_operr = 1'b0; sp_dz = 1'b0;
    sp_r = mk(rd68041_fpu_pkg::C_ZERO, 1'b0, 18'd0, 64'h0);
    if (a_nan || b_nan) begin
      sp_r = quiet(a_nan ? a : b);
    end else begin
      case (op)
        rd68041_fpu_pkg::D_ADD, rd68041_fpu_pkg::D_SUB: begin
          // A zero and a number is the number, rounded: the add does that.
          if (a.cls == rd68041_fpu_pkg::C_INF && b.cls == rd68041_fpu_pkg::C_INF) begin
            if (a.sign != bs_eff) begin
              sp_operr = 1'b1; sp_r = mk(rd68041_fpu_pkg::C_NAN, 1'b0, 18'h7FFF, DNAN_M);
            end else sp_r = mk(rd68041_fpu_pkg::C_INF, a.sign, 18'h7FFF, 64'h0);
          end else if (a.cls == rd68041_fpu_pkg::C_INF) sp_r = mk(rd68041_fpu_pkg::C_INF, a.sign, 18'h7FFF, 64'h0);
          else if (b.cls == rd68041_fpu_pkg::C_INF)     sp_r = mk(rd68041_fpu_pkg::C_INF, bs_eff, 18'h7FFF, 64'h0);
          else if (a.cls == rd68041_fpu_pkg::C_ZERO && b.cls == rd68041_fpu_pkg::C_ZERO)
            // PRM 5 FADD, note 2: opposite zeros are +0, -0 in RM.
            sp_r = mk(rd68041_fpu_pkg::C_ZERO, (a.sign == bs_eff) ? a.sign : (rmode == rd68041_fpu_pkg::R_M), 18'd0, 64'h0);
          else sp = 1'b0;
        end
        rd68041_fpu_pkg::D_CMP: sp = 1'b0;                                // (decided by the order, below)
        rd68041_fpu_pkg::D_MUL, rd68041_fpu_pkg::D_SMUL: begin
          if ((a.cls == rd68041_fpu_pkg::C_INF && b.cls == rd68041_fpu_pkg::C_ZERO) || (a.cls == rd68041_fpu_pkg::C_ZERO && b.cls == rd68041_fpu_pkg::C_INF)) begin
            sp_operr = 1'b1; sp_r = mk(rd68041_fpu_pkg::C_NAN, 1'b0, 18'h7FFF, DNAN_M);
          end else if (a.cls == rd68041_fpu_pkg::C_INF || b.cls == rd68041_fpu_pkg::C_INF) sp_r = mk(rd68041_fpu_pkg::C_INF, sx, 18'h7FFF, 64'h0);
          else if (a.cls == rd68041_fpu_pkg::C_ZERO || b.cls == rd68041_fpu_pkg::C_ZERO)   sp_r = mk(rd68041_fpu_pkg::C_ZERO, sx, 18'd0, 64'h0);
          else sp = 1'b0;
        end
        rd68041_fpu_pkg::D_DIV, rd68041_fpu_pkg::D_SDIV: begin
          if ((a.cls == rd68041_fpu_pkg::C_ZERO && b.cls == rd68041_fpu_pkg::C_ZERO) || (a.cls == rd68041_fpu_pkg::C_INF && b.cls == rd68041_fpu_pkg::C_INF)) begin
            sp_operr = 1'b1; sp_r = mk(rd68041_fpu_pkg::C_NAN, 1'b0, 18'h7FFF, DNAN_M);
          end else if (b.cls == rd68041_fpu_pkg::C_ZERO) begin
            sp_dz = (a.cls != rd68041_fpu_pkg::C_INF); sp_r = mk(rd68041_fpu_pkg::C_INF, sx, 18'h7FFF, 64'h0);
          end else if (a.cls == rd68041_fpu_pkg::C_INF)  sp_r = mk(rd68041_fpu_pkg::C_INF, sx, 18'h7FFF, 64'h0);
          else if (a.cls == rd68041_fpu_pkg::C_ZERO || b.cls == rd68041_fpu_pkg::C_INF) sp_r = mk(rd68041_fpu_pkg::C_ZERO, sx, 18'd0, 64'h0);
          else sp = 1'b0;
        end
        rd68041_fpu_pkg::D_SQRT: begin
          if (b.cls == rd68041_fpu_pkg::C_ZERO) sp_r = mk(rd68041_fpu_pkg::C_ZERO, b.sign, 18'd0, 64'h0);
          else if (b.sign) begin
            sp_operr = 1'b1; sp_r = mk(rd68041_fpu_pkg::C_NAN, 1'b0, 18'h7FFF, DNAN_M);
          end else if (b.cls == rd68041_fpu_pkg::C_INF) sp_r = mk(rd68041_fpu_pkg::C_INF, 1'b0, 18'h7FFF, 64'h0);
          else sp = 1'b0;
        end
        rd68041_fpu_pkg::D_INT: begin
          // UM 9.7.3.2: an infinity to an integer is an operand error (the
          // FPSP stores the largest integer); a zero is zero.
          if (b.cls == rd68041_fpu_pkg::C_INF) sp_operr = 1'b1;
          if (b.cls == rd68041_fpu_pkg::C_ZERO || b.cls == rd68041_fpu_pkg::C_INF) sp_r = mk(b.cls, b.sign, b.exp, b.man);
          else sp = 1'b0;
        end
        default: begin                                   // D_MOVE
          if (b.cls == rd68041_fpu_pkg::C_ZERO || b.cls == rd68041_fpu_pkg::C_INF) sp_r = b;
          else sp = 1'b0;
        end
      endcase
    end
  end
  // A NaN to an integer is an operand error too (UM 9.7.3.2).
  wire sp_nanint = (op == rd68041_fpu_pkg::D_INT) && b_nan;

  // FCMP's ordering of the special cases (PRM 5, FCMP's table).
  function automatic logic [1:0] cmp_sp(input rd68041_fpu_pkg::fpn_t x, input rd68041_fpu_pkg::fpn_t y);
    // {less, equal}: zeros of either sign are equal; an infinity is beyond
    // every number, equal to one of its own sign.
    logic xz, yz, xi, yi;
    xz = (x.cls == rd68041_fpu_pkg::C_ZERO); yz = (y.cls == rd68041_fpu_pkg::C_ZERO);
    xi = (x.cls == rd68041_fpu_pkg::C_INF);  yi = (y.cls == rd68041_fpu_pkg::C_INF);
    if (xz && yz)               cmp_sp = 2'b01;
    else if (xi && yi)          cmp_sp = (x.sign == y.sign) ? 2'b01 : {x.sign, 1'b0};
    else if (xi)                cmp_sp = {x.sign, 1'b0};
    else if (yi)                cmp_sp = {!y.sign, 1'b0};
    else if (xz)                cmp_sp = {!y.sign, 1'b0};
    else                        cmp_sp = {x.sign, 1'b0};             // yz
  endfunction
  // Two numbers, normal.
  function automatic logic [1:0] cmp_n(input rd68041_fpu_pkg::fpn_t x, input rd68041_fpu_pkg::fpn_t y);
    logic gt_mag, eq_mag;
    eq_mag = (x.exp == y.exp) && (x.man == y.man);
    gt_mag = ($signed(x.exp) > $signed(y.exp)) || ((x.exp == y.exp) && (x.man > y.man));
    if (x.sign != y.sign)  cmp_n = {x.sign, 1'b0};
    else if (eq_mag)       cmp_n = 2'b01;
    else                   cmp_n = {x.sign ? gt_mag : !gt_mag, 1'b0};
  endfunction

  // ===========================================================================
  // Add and subtract: align, add, normalise
  // ===========================================================================
  // The larger in magnitude first.
  wire        a_big  = ($signed(a.exp) > $signed(b.exp)) ||
                       ((a.exp == b.exp) && (a.man >= b.man));
  wire [17:0] ed     = a_big ? (a.exp - b.exp) : (b.exp - a.exp);
  wire [6:0]  ash    = (ed > 18'd70) ? 7'd70 : ed[6:0];
  wire [63:0] big_m  = a_big ? a.man : b.man;
  wire [63:0] sml_m  = a_big ? b.man : a.man;
  // The smaller, shifted right with the bits shifted out folded into sticky.
  wire [136:0] sml_w = {sml_m, 3'b000, 70'h0} >> ash;
  wire [66:0]  sml_a = {sml_w[136:71], sml_w[70] | (|sml_w[69:0])};

  // ===========================================================================
  // The rounding (UM 9.4.2, Figure 9-8)
  // ===========================================================================
  // The mantissa precision: FSGLMUL and FSGLDIV round to single whatever the
  // rounding precision (PRM 5), with the extended exponent range.
  wire        q_sgl  = (q_op == rd68041_fpu_pkg::D_SMUL) || (q_op == rd68041_fpu_pkg::D_SDIV);
  wire [1:0]  mprec  = q_sgl ? rd68041_fpu_pkg::P_S : q_prec;
  wire [1:0]  rprec  = q_sgl ? rd68041_fpu_pkg::P_X : q_prec;
  // The normalised intermediate (S_RND's input): integer bit at 66.
  logic [66:0] n_man;
  logic [18:0] n_exp;
  logic        g, rb, st, lsb;
  logic [64:0] t_man;            // the kept bits, as an integer, before the increment
  always_comb begin
    case (mprec)
      rd68041_fpu_pkg::P_S: begin
        t_man = {41'h0, n_man[66:43]};
        lsb = n_man[43]; g = n_man[42]; rb = n_man[41]; st = |n_man[40:0];
      end
      rd68041_fpu_pkg::P_D: begin
        t_man = {12'h0, n_man[66:14]};
        lsb = n_man[14]; g = n_man[13]; rb = n_man[12]; st = |n_man[11:0];
      end
      default: begin
        t_man = {1'b0, n_man[66:3]};
        lsb = n_man[3]; g = n_man[2]; rb = n_man[1]; st = n_man[0];
      end
    endcase
  end
  wire        inexact = g | rb | st;
  wire        inc     = (q_rm == rd68041_fpu_pkg::R_N) ? (g & (rb | st | lsb)) :
                        (q_rm == rd68041_fpu_pkg::R_M) ? (q_sign & inexact) :
                        (q_rm == rd68041_fpu_pkg::R_P) ? (!q_sign & inexact) : 1'b0;
  wire [64:0] t_inc   = t_man + {64'h0, inc};
  // The carry out of the kept bits: the mantissa is a power of two again.
  logic        r_cy;
  logic [63:0] r_m;
  always_comb begin
    case (mprec)
      rd68041_fpu_pkg::P_S:     begin r_cy = t_inc[24]; r_m = r_cy ? 64'h8000_0000_0000_0000 : {t_inc[23:0], 40'h0}; end
      rd68041_fpu_pkg::P_D:     begin r_cy = t_inc[53]; r_m = r_cy ? 64'h8000_0000_0000_0000 : {t_inc[52:0], 11'h0}; end
      default: begin r_cy = t_inc[64]; r_m = r_cy ? 64'h8000_0000_0000_0000 : t_inc[63:0]; end
    endcase
  end
  wire [18:0] r_e = n_exp + {18'h0, r_cy};
  // The range of the precision (UM 9.4.2, 9.7.4, 9.7.5).
  wire [18:0] emin = (rprec == rd68041_fpu_pkg::P_S) ? 19'd16257 : (rprec == rd68041_fpu_pkg::P_D) ? 19'd15361 : 19'd0;
  wire [18:0] emax = (rprec == rd68041_fpu_pkg::P_S) ? 19'd16510 : (rprec == rd68041_fpu_pkg::P_D) ? 19'd17406 : 19'd32766;
  wire        r_unfl = $signed(n_exp) < $signed(emin);
  // An underflow's inexactness is the denormalised result's (the one the
  // FPSP stores): the top p - (emin - exp) bits are kept, as UAE040 has it.
  wire [18:0] dn_sh  = emin - n_exp;
  wire [18:0] dn_k19 = 19'((mprec == rd68041_fpu_pkg::P_S) ? 24 :
                           (mprec == rd68041_fpu_pkg::P_D) ? 53 : 64) - dn_sh;
  wire [6:0]  dn_k   = ($signed(dn_k19) <= 0) ? 7'd0 : dn_k19[6:0];
  wire [66:0] dn_lo  = n_man << dn_k;
  wire        dn_inex = (dn_k == 7'd0) || (dn_lo != 67'h0);
  // ... and whether it is a zero: no bit kept, and not rounded up to the
  // smallest denormalised number -- away from zero in RM and RP, or in RN
  // with the leading bit the guard bit and one below it.
  wire        dn_zero = (dn_k == 7'd0) &&
                        !((q_rm == rd68041_fpu_pkg::R_M && q_sign) ||
                          (q_rm == rd68041_fpu_pkg::R_P && !q_sign) ||
                          (q_rm == rd68041_fpu_pkg::R_N && dn_k19 == 19'd0 && n_man[65:0] != 66'h0));
  wire        r_ovfl = !r_unfl && ($signed(r_e) > $signed(emax));

  // ===========================================================================
  // To an integer (FMOVE to B, W, L): round in FPCR's mode, then the range
  // ===========================================================================
  // The unbiased exponent; the integer part and guard, round, sticky: the
  // mantissa shifted right by 63 less the exponent (by 70 at most: all of it
  // below the round bit).
  wire [18:0]  ie   = q_exp - 19'd16383;
  wire [18:0]  ish19 = 19'd63 - ie;
  wire [6:0]   ish  = ($signed(ie) > 19'sd63) ? 7'd0 :
                      ($signed(ie) < -19'sd7) ? 7'd70 : ish19[6:0];
  wire [133:0] iw   = {q_a, 70'h0} >> ish;
  wire [63:0]  ipart = iw[133:70];
  wire         ig    = iw[69];
  wire         ir    = iw[68];
  wire         is_   = |iw[67:0];
  wire         iinex = ig | ir | is_;
  wire         iinc  = (q_rm == rd68041_fpu_pkg::R_N) ? (ig & (ir | is_ | ipart[0])) :
                       (q_rm == rd68041_fpu_pkg::R_M) ? (q_sign & iinex) :
                       (q_rm == rd68041_fpu_pkg::R_P) ? (!q_sign & iinex) : 1'b0;
  wire [64:0]  imag  = {1'b0, ipart} + {64'h0, iinc};
  // The magnitude's limit: 2^(n-1) - 1 positive; 2^(n-1) negative, but the
  // MC68040 calls that an operand error too (UM 9.7.3.2, "erroneous MC68040
  // condition"), and the FPSP stores it.
  wire [64:0]  ilim  = (q_isz == 2'd0) ? 65'h80 : (q_isz == 2'd1) ? 65'h8000 : 65'h8000_0000;
  wire         iovf  = ($signed(ie) > 19'sd63) || (imag >= ilim);
  wire [31:0]  ival  = q_sign ? (32'h0 - imag[31:0]) : imag[31:0];

  // ===========================================================================
  // Normalise: a carry shifts right, leading zeros left (UM Figure 9-7)
  // ===========================================================================
  function automatic logic [6:0] lzc67(input logic [66:0] v);
    lzc67 = 7'd67;
    for (int k = 0; k < 67; k++) if (v[k]) lzc67 = 7'(66 - k);
  endfunction
  logic [6:0] lz;
  assign lz = lzc67(q_man[66:0]);
  wire [66:0] nl_man = q_man[66:0] << lz;

  // ===========================================================================
  // The square root: two bits of the radicand a clock (acc holds them)
  // ===========================================================================
  wire [69:0] sq_r = {rem[67:0], acc[127:126]};
  wire [69:0] sq_t = {quo[67:0], 2'b01};
  wire        sq_ge = sq_r >= sq_t;
  // The divide: a quotient bit a clock.
  wire [69:0] dv_t  = {6'h0, q_b};
  wire        dv_ge = rem >= dv_t;

  // ===========================================================================
  // The state machine
  // ===========================================================================
  assign busy = start || ((s != S_IDLE) && (s != S_DONE));

  // The multiplier: a 64 x 16 partial product a clock.
  wire [79:0] pp = q_a * q_b[16*cnt[1:0] +: 16];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s <= S_IDLE; q_op <= '0; q_prec <= '0; q_rm <= '0; q_isz <= '0; q_sign <= 1'b0;
      q_exp <= '0; q_man <= '0; cnt <= '0; q_a <= '0; q_b <= '0; acc <= '0; rem <= '0;
      quo <= '0; n_man <= '0; n_exp <= '0; q_op_sub <= 1'b0;
      done <= 1'b0; r <= '0; f_snan <= 1'b0; f_operr <= 1'b0; f_ovfl <= 1'b0;
      f_unfl <= 1'b0; f_dz <= 1'b0; f_inex <= 1'b0; f_spec <= 1'b0; i_exp <= '0; i_man <= '0;
      f_dnz <= 1'b0; r_int <= '0; c_lt <= 1'b0; c_eq <= 1'b0; c_un <= 1'b0;
    end else if (crst || kill) begin
      s <= S_IDLE; done <= 1'b0;
    end else begin
      done <= 1'b0;
      case (s)
        S_IDLE, S_DONE: begin
          s <= S_IDLE;
          if (start) begin
            q_op <= op; q_prec <= prec; q_rm <= rmode; q_isz <= isz;
            f_snan <= any_sn; f_operr <= sp_operr || sp_nanint; f_dz <= sp_dz;
            f_ovfl <= 1'b0; f_unfl <= 1'b0; f_inex <= 1'b0; f_dnz <= 1'b0;
            f_spec <= sp;
            c_un <= a_nan || b_nan;
            c_lt <= 1'b0; c_eq <= 1'b0;
            r_int <= 32'h0;
            i_exp <= '0; i_man <= '0;
            if (op == rd68041_fpu_pkg::D_CMP) begin
              // FCMP decides at once: the condition codes come from the order.
              if (!(a_nan || b_nan)) begin
                {c_lt, c_eq} <= (a.cls == rd68041_fpu_pkg::C_NORM && b.cls == rd68041_fpu_pkg::C_NORM) ? cmp_n(a, b) : cmp_sp(a, b);
              end
              r <= sp ? sp_r : a;
              s <= S_DONE; done <= 1'b1;
            end else if (sp || sp_nanint) begin
              r <= sp_r;
              s <= S_DONE; done <= 1'b1;
            end else begin
              q_a <= b.man; q_b <= a.man;
              case (op)
                rd68041_fpu_pkg::D_ADD, rd68041_fpu_pkg::D_SUB: begin
                  q_sign <= a_big ? a.sign : bs_eff;
                  q_op_sub <= (a.sign != bs_eff);
                  q_exp  <= {1'b0, a_big ? a.exp : b.exp};
                  // The aligned operands; the sum on the next clock.
                  q_man  <= {1'b0, big_m, 3'b000};
                  acc    <= {61'h0, sml_a};
                  s <= S_ADD1;
                end
                rd68041_fpu_pkg::D_MUL, rd68041_fpu_pkg::D_SMUL: begin
                  q_sign <= sx;
                  q_exp  <= {a.exp[17], a.exp} + {b.exp[17], b.exp} - 19'd16383;
                  // FSGLMUL: the operands' mantissas truncated to 24 bits.
                  q_a <= (op == rd68041_fpu_pkg::D_SMUL) ? {b.man[63:40], 40'h0} : b.man;
                  q_b <= (op == rd68041_fpu_pkg::D_SMUL) ? {a.man[63:40], 40'h0} : a.man;
                  acc <= '0; cnt <= 7'd0;
                  s <= S_MUL;
                end
                rd68041_fpu_pkg::D_DIV, rd68041_fpu_pkg::D_SDIV: begin
                  q_sign <= sx;
                  q_exp  <= {a.exp[17], a.exp} - {b.exp[17], b.exp} + 19'd16383;
                  // FSGLDIV: the operands whole. PRM 5 has them truncated to
                  // 24 bits and, in the next paragraph, "just change to
                  // extended precision"; UAE040 divides them whole (and
                  // truncates FSGLMUL's), doc/manual-contradictions.md.
                  q_a <= a.man;
                  q_b <= b.man;
                  rem <= {6'h0, a.man};
                  quo <= '0; cnt <= 7'd0;
                  s <= S_DIV;
                end
                rd68041_fpu_pkg::D_SQRT: begin
                  // The radicand in [1, 4): the mantissa, shifted once more
                  // for an odd exponent; the root's exponent half of it.
                  q_sign <= 1'b0;
                  q_exp  <= 19'($signed({b.exp[17], b.exp} - 19'd16383) >>> 1) + 19'd16383;
                  acc    <= b.exp[0] ? {1'b0, b.man, 63'h0} : {b.man, 64'h0};
                  rem <= '0; quo <= '0; cnt <= 7'd0;
                  s <= S_SQRT;
                end
                rd68041_fpu_pkg::D_INT: begin
                  q_sign <= b.sign;
                  q_exp  <= {b.exp[17], b.exp};
                  q_a    <= b.man;
                  s <= S_INT;
                end
                default: begin                           // D_MOVE: round the source
                  q_sign <= b.sign;
                  q_exp  <= {b.exp[17], b.exp};
                  q_man  <= {1'b0, b.man, 3'b000};
                  s <= S_NORM;
                end
              endcase
            end
          end
        end

        S_ADD1: begin
          // The sum, or the difference of the larger and the smaller.
          q_man <= (q_op_sub) ? ({1'b0, q_man[66:0]} - {1'b0, acc[66:0]})
                              : ({1'b0, q_man[66:0]} + {1'b0, acc[66:0]});
          s <= S_ADD2;
        end
        S_ADD2: begin
          // A difference of exactly zero: +0, or -0 in RM (PRM 5, FADD).
          if (q_man == 68'h0) begin
            r <= mk(rd68041_fpu_pkg::C_ZERO, q_rm == rd68041_fpu_pkg::R_M, 18'd0, 64'h0);
            s <= S_DONE; done <= 1'b1;
          end else begin
            s <= S_NORM;
          end
        end

        S_MUL: begin
          acc <= acc + ({48'h0, pp} << (16 * cnt[1:0]));
          cnt <= cnt + 7'd1;
          if (cnt[1:0] == 2'd3) begin
            cnt <= 7'd0;
            s <= S_NORM;
          end
        end

        S_DIV: begin
          // One quotient bit: 68 of them, the first the integer one.
          quo <= {quo[66:0], dv_ge};
          rem <= (dv_ge ? (rem - dv_t) : rem) << 1;
          cnt <= cnt + 7'd1;
          if (cnt == 7'd67) s <= S_NORM;
        end

        S_SQRT: begin
          // One root bit: 67 of them, the first the integer one.
          quo <= {quo[66:0], sq_ge};
          rem <= sq_ge ? (sq_r - sq_t) : sq_r;
          acc <= acc << 2;
          cnt <= cnt + 7'd1;
          if (cnt == 7'd66) s <= S_NORM;
        end

        S_NORM: begin
          // Into n_man/n_exp: integer bit at 66, sticky folded in.
          case (q_op)
            rd68041_fpu_pkg::D_MUL, rd68041_fpu_pkg::D_SMUL: begin
              if (acc[127]) begin
                n_man <= {acc[127:62], |acc[61:0]};
                n_exp <= q_exp + 19'd1;
              end else begin
                n_man <= {acc[126:61], |acc[60:0]};
                n_exp <= q_exp;
              end
            end
            rd68041_fpu_pkg::D_DIV, rd68041_fpu_pkg::D_SDIV: begin
              if (quo[67]) begin
                n_man <= {quo[67:2], quo[1] | quo[0] | (rem != 70'h0)};
                n_exp <= q_exp;
              end else begin
                n_man <= {quo[66:1], quo[0] | (rem != 70'h0)};
                n_exp <= q_exp - 19'd1;
              end
            end
            rd68041_fpu_pkg::D_SQRT: begin
              n_man <= {quo[66:1], quo[0] | (rem != 70'h0)};
              n_exp <= q_exp;
            end
            default: begin                         // add, subtract, move
              if (q_man[67]) begin
                n_man <= {q_man[67:2], q_man[1] | q_man[0]};
                n_exp <= q_exp + 19'd1;
              end else begin
                n_man <= nl_man;
                n_exp <= q_exp - {12'h0, lz};
              end
            end
          endcase
          s <= S_RND;
        end

        S_RND: begin
          i_exp <= n_exp[17:0];
          i_man <= n_man;
          f_unfl <= r_unfl;
          f_dnz  <= dn_zero;
          f_ovfl <= r_ovfl;
          f_inex <= r_unfl ? dn_inex : inexact;
          r <= mk(rd68041_fpu_pkg::C_NORM, q_sign, r_e[17:0], r_m);
          s <= S_DONE; done <= 1'b1;
        end

        S_INT: begin
          f_operr <= iovf;
          f_inex  <= iinex && !iovf;
          // Out of range, the value is undefined (UM 9.7.3.2: the FPSP stores
          // it); this stores the largest of the sign, as UAE040 does.
          if (iovf)
            r_int <= (q_isz == 2'd0) ? (q_sign ? 32'hFFFF_FF80 : 32'h0000_007F) :
                     (q_isz == 2'd1) ? (q_sign ? 32'hFFFF_8000 : 32'h0000_7FFF) :
                                       (q_sign ? 32'h8000_0000 : 32'h7FFF_FFFF);
          else
            r_int <= (q_isz == 2'd0) ? {{24{ival[7]}}, ival[7:0]} :
                     (q_isz == 2'd1) ? {{16{ival[15]}}, ival[15:0]} : ival;
          // The rounded integer, for the frame's WBTEMP (UM Table 9-16).
          i_man   <= {imag[63:0], 3'b000};
          r <= mk(rd68041_fpu_pkg::C_NORM, q_sign, q_exp[17:0], q_a);
          s <= S_DONE; done <= 1'b1;
        end

        default: s <= S_IDLE;
      endcase
    end
  end

  logic unused_ok;
  assign unused_ok = &{1'b0, ish19[18:7], dn_k19[18:7]};

endmodule
