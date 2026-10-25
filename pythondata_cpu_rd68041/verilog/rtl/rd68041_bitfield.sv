// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68041

// RD68041 -- the bit-field unit. PRM 4, the eight BFxxx instructions.
//
// A bit field is `width` bits, 1 to 32 of them, starting `offset` bits after a
// base. Everything here is built around ONE intermediate value: the field
// LEFT-JUSTIFIED in thirty-two bits, with the bits below it zero. Every
// condition code and every result the eight instructions want is a function of
// that and the width, and the two addressing cases differ only in how they
// produce it and how they put it back.
//
//   a data register   the offset is taken modulo 32 and the field WRAPS from
//                     bit 0 round to bit 31, so left-justifying it is a rotate
//                     and nothing else.
//   memory            the field starts at bit 7 - (offset mod 8) of the byte
//                     at base + offset/8 and runs downward through as many as
//                     five bytes -- PRM 4, "the possible accesses are byte,
//                     word, 3-byte, long word, and long word with byte".
//
// Bit numbering is the manual's, not the register's: offset 0 is the MOST
// significant bit of the base byte, and increasing offset moves toward less
// significant bits.

`default_nettype none

module rd68041_bitfield (
    // Which case, and the field's position and size.
    input  wire         is_reg,
    input  wire  [31:0] reg_data,   // the data register, when is_reg
    input  wire  [39:0] mem_data,   // the bytes read, FIRST byte in 39:32
    input  wire   [4:0] roff,       // offset mod 32, for the register case
    input  wire   [2:0] boff,       // offset mod 8, for the memory case
    input  wire   [5:0] width,      // 1 to 32

    // What to put back, right-justified in its low `width` bits.
    input  wire  [31:0] ins,

    // The field, and what the instructions make of it.
    output wire  [31:0] field,      // right-justified, zero extended
    output wire  [31:0] sxfield,    // ... and sign extended
    output wire         msb,        // PRM 4: N is the field's most significant bit
    output wire         zero,       // ... and Z is all of them being clear
    output wire   [5:0] ffo,        // bits from the top of the field to the
                                    // first one, or `width` if there is none

    // The field replaced, in whichever form it came.
    output wire  [31:0] merged_reg,
    output wire  [39:0] merged_mem
);

  // A run of `width` ones at the top. Width is never zero -- the microcode
  // turns the encoded zero into 32 before it gets here -- so this never has to
  // produce an empty mask, and `32 - width` is never a shift of 32.
  wire [31:0] mask32 = 32'hFFFF_FFFF << (6'd32 - width);

  // ------------------------------------------------------------------------
  // The field, left justified
  // ------------------------------------------------------------------------
  // A register's field is a rotate: the wrap the manual describes IS what a
  // rotate does, so there is no special case for a field that runs off the end.
  wire [63:0] doubled  = {reg_data, reg_data};
  wire [31:0] reg_left = doubled[63 - 32'(roff) -: 32];

  // A memory field is a shift of the window the bytes were read into. The
  // window holds them first-byte-first, so the field begins `boff` bits in.
  // Only the top thirty-two bits of the shifted window can be part of the
  // field: the field is at most thirty-two bits and starts at the top.
  wire [39:0] mem_shift = mem_data << boff;
  wire [31:0] mem_left  = mem_shift[39:8];
  wire  [7:0] mem_unused = mem_shift[7:0];   // read by nothing, and named so

  wire [31:0] fieldl = (is_reg ? reg_left : mem_left) & mask32;

  assign field   = fieldl >> (6'd32 - width);
  wire [63:0] sxwide = {{32{fieldl[31]}}, fieldl};
  assign sxfield = 32'(sxwide >> (6'd32 - width));
  assign msb     = fieldl[31];
  assign zero    = (fieldl == 32'd0);

  // ------------------------------------------------------------------------
  // BFFFO. PRM 4: "the bit offset of that bit ... is placed in Dn. If no bit in
  // the bit field is set to one, the value in Dn is the field offset plus the
  // field width."
  //
  // Counting the leading zeros of the left-justified field answers both: the
  // bits below the field are already zero, so a field of all zeros counts 32,
  // which is at least the width, and the width is what the manual asks for.
  // ------------------------------------------------------------------------
  // Counting UPWARD so that the last assignment is the highest set bit, which
  // is the first one from the top. Counting downward finds the lowest set bit
  // instead, which is a different instruction entirely.
  logic [5:0] clz;
  int i;
  always_comb begin
    clz = 6'd32;
    for (i = 0; i < 32; i = i + 1)
      if (fieldl[i]) clz = 6'd31 - 6'(i);
  end
  assign ffo = (clz > width) ? width : clz;

  // ------------------------------------------------------------------------
  // Putting it back
  // ------------------------------------------------------------------------
  wire [31:0] insl = ins << (6'd32 - width);

  // The register case is the rotate undone.
  wire [31:0] reg_new  = (reg_left & ~mask32) | (insl & mask32);
  wire [63:0] redouble = {reg_new, reg_new};
  assign merged_reg = redouble[32'(roff) + 31 -: 32];

  // The memory case puts the window back where it came from.
  wire [39:0] mask40 = {mask32, 8'd0} >> boff;
  wire [39:0] insm   = {insl, 8'd0} >> boff;
  assign merged_mem = (mem_data & ~mask40) | (insm & mask40);

endmodule

`default_nettype wire
