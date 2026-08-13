/******************************************************************************/
/* Mednafen NEC PC-FX Emulation Module                                        */
/******************************************************************************/
/* rainbow.cpp:
**  Copyright (C) 2006-2019 Mednafen Team
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/*
** MJPEG-like decoder based on the algorithm and Huffman data tables provided by David Michel
** of MagicEngine and MagicEngine-FX.
*/

#include "pcfx.h"
#include "rainbow.h"

// ---- build-time toggle, declared here so the logging below can read it ----
// RB_NULLRUN_HOLD: candidate fix for the rectangular blocks in the Disc B
// scenario-mode intro. Null-run columns keep their previous contents instead of
// being painted with the null-run colour. See the null-run note further down.
//
// DEFAULT OFF. The confetti repair is the priority and is well validated; this
// fires on every null run rather than a few times per session, and the reading
// of null-run semantics behind it is inferred from behaviour rather than
// documentation. Leaving it off lets the confetti fix be judged on its own.
static const bool RB_NULLRUN_HOLD = false;
// ---------------------------------------------------------------------------

#include "king.h"
#include "interrupt.h"
#include "idct.h"

#include <mednafen/FileStream.h>

namespace MDFN_IEN_PCFX
{

static bool ChromaIP;	// Bilinearly interpolate chroma channel

/* Y = luminance/luma, UV = chrominance/chroma */

struct HuffmanQuickLUTPair
{
	uint8 val;
	uint8 bitc;	// Bit count for the code.
};

static const uint8 zigzag[63] =
{
 0x01, 0x08, 0x10, 0x09, 0x02, 0x03, 0x0A, 0x11,
 0x18, 0x20, 0x19, 0x12, 0x0B, 0x04, 0x05, 0x0C,
 0x13, 0x1A, 0x21, 0x28, 0x30, 0x29, 0x22, 0x1B,
 0x14, 0x0D, 0x06, 0x07, 0x0E, 0x15, 0x1C, 0x23,
 0x2A, 0x31, 0x38, 0x39, 0x32, 0x2B, 0x24, 0x1D,
 0x16, 0x0F, 0x17, 0x1E, 0x25, 0x2C, 0x33, 0x3A,
 0x3B, 0x34, 0x2D, 0x26, 0x1F, 0x27, 0x2E, 0x35,
 0x3C, 0x3D, 0x36, 0x2F, 0x37, 0x3E, 0x3F
};

static const HuffmanQuickLUTPair ac_y_qlut[1 << 12] =
{
 #include "rainbow_acy.inc"
};

static const HuffmanQuickLUTPair ac_uv_qlut[1 << 12] =
{
 #include "rainbow_acuv.inc"
};

static const HuffmanQuickLUTPair dc_y_qlut[1 << 9] =
{
 #include "rainbow_dcy.inc"
};

static const HuffmanQuickLUTPair dc_uv_qlut[1 << 8] =
{
 #include "rainbow_dcuv.inc"
};

static uint8 *DecodeBuffer[2] = { NULL, NULL };
static int32 DecodeFormat[2]; // The format each buffer is in(-1 = invalid, 0 = palettized 8-bit, 1 = YUV)
static uint32 QuantTables[2][64], QuantTablesBase[2][64];	// 0 = Y, 1 = UV

static uint32 DecodeBufferWhichRead;

static int32 RasterReadPos;
static uint16 Control;
static uint16 NullRunY, NullRunU, NullRunV, HSync;
static uint16 HScroll;

static uint32 bits_buffer;
static uint32 bits_buffered_bits;
static int32 bits_bytes_left;
static int32 rb_starve_bytes = 0;   // how far past the block the decode ran

// ===========================================================================
// PC-FX RAINBOW: fix the AJW Queen of Queens FMV "garbage bar".
//
// Symptom: part of the width of one 16-line strip renders as coloured noise
// for one to three frames. Clean picture on the left, noise from some column
// rightward. Not present on real hardware.
//
// Cause: some FMV blocks do not carry usable entropy data for all 16 macroblock
// columns. Per-column byte-consumption logging from real gameplay:
//
//   block_size 730:  3 118 72 51 | 150 136 160 | 39 0 0 0 0
//                    \_ normal _/  \_ desynced _/  \_ no data _/
//
// The decode runs normally for a few columns, loses bit alignment (visible as
// uniformly inflated cost), and eventually runs the block dry, after which
// FetchWidgywabbit substitutes 0x00 and the remaining columns decode from
// zeros. Everything from the desync onward renders as noise.
//
// Detecting this from the bitstream proved unreliable: the desync begins
// before starvation, and some affected strips never starve at all. So this
// works on the DECODED RESULT instead - it measures how noisy each column
// actually looks and replaces the ones that are clearly not picture.
//
// The measure is mean absolute horizontal luma difference within a column.
// Calibrated over 460,000 decoded strips of real gameplay: ordinary detailed
// video reached 35, genuinely corrupt strips scored 43 to 201. The test here
// is also self-calibrating per strip - a column must stand far above the
// median of its own strip - so detailed footage does not trip it.
//
// Replacement uses HappyColor, the fill the format itself uses for null-run
// columns. Stateless: no frame history, no row tracking, nothing read past the
// declared block size.
//
// Set RB_FIX_NOISY_COLUMNS false for stock behaviour.
// DETECTION ONLY in this build. The metric cannot be calibrated outside the
// emulator (the captured byte dumps do not carry the quantisation tables that
// were live, so an offline decode produces wrong pixel values). So this build
// measures and REPORTS what it would replace, and changes nothing on screen.
// Once the log shows the detector separating real corruption from ordinary
// detailed video, the replacement gets switched on.
static const bool   RB_FIX_NOISY_COLUMNS  = true;
// Calibrated in-emulator over 350,000 strips (~5.6 million column readings).
// The two populations are cleanly separated:
//
//   0-9: 4,968,381   10s: 534,420   20s: 92,565   30s: 4,456
//   40s: 43          50s: 0         60s: 3        70s: 25   80s: 3   90+: 104
//
// Ordinary picture stops in the 30s. Confirmed corruption measured 63, 76, 79,
// 83, 90, 95, 96, 124, 142, 151 and 166. A floor of 40 caught columns scoring
// 41 next to neighbours at 32 - ordinary detail, repaired unnecessarily for 32
// consecutive strips. 60 sits inside the empty band between the populations.
static const uint32 RB_NOISE_FLOOR        = 60;
static const uint32 RB_NOISE_RATIO        = 3;

// Chroma floor. The artifact is a COLOURED checkerboard, and corruption can
// swing U/V hard while luma stays flat - a luma-only measure scores that near
// zero and misses it entirely. Set equal to the luma floor as a starting point;
// real video generally varies less in chroma than in luma, so this is
// conservative. The log reports chroma alongside luma so it can be tuned.
static const uint32 RB_CHROMA_FLOOR       = 60;

// Second, independent trigger: the block ran dry.
//
// Measuring the decoded picture can only catch corruption that some metric
// happens to see. A block running substantially short of its declared size is
// direct evidence the strip is bad, needs no metric at all, and cannot be
// fooled by corruption that looks flat.
//
// The threshold excludes trivial shortfalls: of 497 observed starvations, 454
// were exactly one byte, which is just the bit reader's lookahead topping up at
// the end of a strip. Suppressing on those replaced hundreds of good strips per
// session and caused visible jitter. Real desyncs ran short by 208, 503 and 626
// bytes.
static const int32  RB_STARVE_THRESHOLD   = 8;

// A corrupt strip is replaced with the same strip row from the previous frame.
// FMV runs at roughly 15fps and changes little between frames, so holding a
// strip for the one to three frames an artifact lasts is close to invisible -
// far less conspicuous than substituting neighbouring picture, which puts a
// spatial discontinuity on screen that the eye catches in a single frame.
//
// A frame is 15 strips (240 lines / 16). Measured over 16,000 frames:
// 12,402 frames of exactly 15 strips, plus 3,353 back-to-back FirstDecode
// calls with no strips between (harmless) and 24 frames of other lengths.
//
// Only clean strips are cached, so corruption is never propagated forward.
// Until a row has been seen clean at least once there is nothing to hold, and
// the column repair below is used instead.
static const int32 RB_STRIPS_PER_FRAME = 15;

static uint32 rb_row_cache[16][256 * 16];
static bool   rb_row_cache_valid[16] = { false };
static int32  rb_row = 0;

// Null-run columns.
//
// The format can mark a run of macroblock columns as carrying no data. Stock
// behaviour paints them with HappyColor, a solid colour derived from the
// NullRunY/U/V registers. In AJW Queen of Queens' Disc B scenario-mode intro
// that produces visible rectangular blocks which appear ONLY under emulation:
// they are absent on real hardware, and absent from an independent decoder that
// leaves such columns holding their previous contents.
//
// That points at the meaning of a null run: the region is unchanged, so it
// should keep what was already there rather than being repainted. With
// RB_NULLRUN_HOLD the columns are restored from the previous frame's copy of
// this strip row - the same cache used to repair corrupt strips. Where no
// cached row exists yet, the stock fill is used.
//
// Set false for stock behaviour.
// Declared near the top of the file - see the toggle block above.

// Corrupt columns are repaired as contiguous runs. A short gap of clean-looking
// columns inside a corrupt run is still suspect, so gaps up to this length are
// absorbed - but no more, so that a single distant column cannot drag the
// repair across good picture between them.
static const int    RB_MAX_GAP            = 2;
// ===========================================================================

static void InitBits(int32 bcount)
{
 bits_bytes_left = bcount;
 bits_buffer = 0;
 bits_buffered_bits = 0;
 rb_starve_bytes = 0;
}

static INLINE uint8 FetchWidgywabbit(void)
{
 if(bits_bytes_left <= 0)
 {
  // Block exhausted. Stock behaviour (return 0) is preserved; we only count how
  // far short it ran, as direct evidence this strip's decode is untrustworthy.
  rb_starve_bytes++;
  return(0);
 }

 uint8 ret = KING_RB_Fetch();
 if(ret == 0xFF) 
  KING_RB_Fetch();

 bits_bytes_left--;

 return(ret);
}

enum
{
 MDFNBITS_PEEK = 1,
 MDFNBITS_FUNNYSIGN = 2,
};

static INLINE uint32 GetBits(const unsigned int count, const unsigned int how = 0)
{
 uint32 ret;

 while(bits_buffered_bits < count)
 {
  bits_buffer <<= 8;
  bits_buffer |= FetchWidgywabbit();
  bits_buffered_bits += 8;
 }

 ret = (bits_buffer >> (bits_buffered_bits - count)) & ((1 << count) - 1);

 if(!(how & MDFNBITS_PEEK))
  bits_buffered_bits -= count;

 if((how & MDFNBITS_FUNNYSIGN) && count)
 {
  if(ret < (1U << (count - 1)))
   ret += 1 - (1 << count);
 }

 return(ret);
}

// Note: SkipBits is intended to be called right after GetBits() with "how" MDFNBITS_PEEK,
// and the count pass to SkipBits must be less than or equal to the count passed to GetBits().
static INLINE void SkipBits(const unsigned int count)
{
 bits_buffered_bits -= count;
}

static uint32 HappyColor; // Cached, calculated from null run yuv registers;
static void CalcHappyColor(void)
{
 uint8 y_c = (int16)NullRunY + 0x80;
 uint8 u_c = (int16)NullRunU + 0x80;
 uint8 v_c = (int16)NullRunV + 0x80;

 HappyColor = (y_c << 16) | (u_c << 8) | (v_c << 0);
}

static uint32 get_ac_coeff(const HuffmanQuickLUTPair *table, int32 *zeroes)
{
 unsigned int numbits;
 uint32 rawbits;
 uint32 code;

 rawbits = GetBits(12, MDFNBITS_PEEK);
 code = table[rawbits].val;
 SkipBits(table[rawbits].bitc);

 numbits = code & 0xF;
 *zeroes = code >> 4;

 return(GetBits(numbits, MDFNBITS_FUNNYSIGN));
}

static uint32 get_dc_coeff(const HuffmanQuickLUTPair *table, int32 *zeroes, int maxbits)
{
 uint32 code;

 for(;;)
 {
  uint32 rawbits;

  rawbits = GetBits(maxbits, MDFNBITS_PEEK);
  code = table[rawbits].val;
  SkipBits(table[rawbits].bitc);

  if(code < 0xF)
  {
   *zeroes = 0;
   return(GetBits(code, MDFNBITS_FUNNYSIGN));
  }
  else if(code == 0xF)
  {
   get_ac_coeff(ac_y_qlut, zeroes);
   (*zeroes)++;
   return(0);
  }
  else if(code >= 0x10)
  {
   code -= 0x10;

   for(int i = 0; i < 64; i++)
   {
    // Y
    uint32 coeff = (QuantTablesBase[0][i] * code) >> 2;

    if(coeff < 1)
     coeff = 1;
    else if(coeff > 0xFE)
     coeff = 0xFE;

    QuantTables[0][i] = coeff;

    // UV
    if(i)
     coeff = (QuantTablesBase[1][i] * code) >> 2;
    else
     coeff = (QuantTablesBase[1][i]) >> 2;

    if(coeff < 1)
     coeff = 1;
    else if(coeff > 0xFE)
     coeff = 0xFE;

    QuantTables[1][i] = coeff;
   }

  }
 }

}

static INLINE uint32 get_dc_y_coeff(int32 *zeroes)
{
 return(get_dc_coeff(dc_y_qlut, zeroes, 9));
}

static uint32 get_dc_uv_coeff(void)
{
 const HuffmanQuickLUTPair *table = dc_uv_qlut;
 uint32 code;
 uint32 rawbits = GetBits(8, MDFNBITS_PEEK);

 code = table[rawbits].val;
 SkipBits(table[rawbits].bitc);

 return(GetBits(code, MDFNBITS_FUNNYSIGN));
}

static void decode(int32 *dct, const uint32 *QuantTable, const int32 dc, const HuffmanQuickLUTPair *table)
{
 int32 coeff;
 int zeroes;
 int count;
 int index;

 dct[0] = (int16)(QuantTable[0] * dc);
 count = 0;

 do
 {
  coeff = get_ac_coeff(table, &zeroes);
  if(!coeff)
  {
   if(!zeroes)
   {
    while(count < 63)
    {
     dct[zigzag[count++]] = 0;
    }
    break;
   }
   else if(zeroes == 1)
    zeroes = 0xF;
  }
  
  while(zeroes-- && count < 63)
  {
   dct[zigzag[count++]] = 0;
  }
  zeroes = 0;
  if(count < 63)
  {
   index = zigzag[count++];
   dct[index] = (int16)(QuantTable[index] * coeff);
  }
 } while(count < 63);

}

#ifdef WANT_DEBUGGER
uint32 RAINBOW_GetRegister(const unsigned int id, char* special, const uint32 special_len)
{
 uint32 value = 0xDEADBEEF;

 if(id == RAINBOW_GSREG_RSCRLL)
  value = HScroll;
 else if(id == RAINBOW_GSREG_RCTRL)
  value = Control;
 else if(id == RAINBOW_GSREG_RNRY)
  value = NullRunY;
 else if(id == RAINBOW_GSREG_RNRU)
  value = NullRunU;
 else if(id == RAINBOW_GSREG_RNRV)
  value = NullRunV;
 else if(id == RAINBOW_GSREG_RHSYNC)
  value = HSync;

 return(value);
}

void RAINBOW_SetRegister(const unsigned int id, uint32 value)
{
 if(id == RAINBOW_GSREG_RSCRLL)
  HScroll = value & 0x1FF;
 else if(id == RAINBOW_GSREG_RCTRL)
  Control = value;
 else if(id == RAINBOW_GSREG_RNRY)
  NullRunY = value;
 else if(id == RAINBOW_GSREG_RNRU)
  NullRunU = value;
 else if(id == RAINBOW_GSREG_RNRV)
  NullRunV = value;
}
#endif

static uint32 LastLine[256];
static bool FirstDecode;
static bool GarbageData;

static void Cleanup(void)
{
 for(int i = 0; i < 2; i++)
 {
  if(DecodeBuffer[i])
  {
   delete[] DecodeBuffer[i];
   DecodeBuffer[i] = NULL;
  }
 }
}

void RAINBOW_Init(bool arg_ChromaIP)
{
 try
 {
  ChromaIP = arg_ChromaIP;

  for(int i = 0; i < 2; i++)
  {
   DecodeBuffer[i] = new uint8[0x2000 * 4];
   memset(DecodeBuffer[i], 0, 0x2000 * 4);
  }

  DecodeFormat[0] = DecodeFormat[1] = -1;
  DecodeBufferWhichRead = 0;
  GarbageData = false;
  FirstDecode = true;
  RasterReadPos = 0;
 }
 catch(...)
 {
  Cleanup();
  throw;
 }
}

void RAINBOW_Close(void)
{
 Cleanup();
}

// RAINBOW base I/O port address: 0x200

#define REGSETBOFW(_reg, _data, _wb) { (_reg) &= ~(0xFF << ((_wb) * 8)); (_reg) |= (_data) << ((_wb) * 8); }
#define REGSETBOFH REGSETBOFW

// The horizontal scroll register is set up kind of weird...D0-D7 of writes to $200 set the lower byte, D0-D7 of writes to $202 set the upper byte

void RAINBOW_Write8(uint32 A, uint8 V)
{
 //printf("RAINBOW Wr8: %08x %02x\n", A, V);
 switch(A & 0x1C)
 {
  case 0x00: REGSETBOFH(HScroll, V, (A & 0x2) >> 1); HScroll &= 0x1FF; break;
  case 0x04: REGSETBOFH(Control, V, A & 0x3); break;
  case 0x08: REGSETBOFH(NullRunY, V, A & 0x3); CalcHappyColor(); break;
  case 0x0C: REGSETBOFH(NullRunU, V, A & 0x3); CalcHappyColor(); break;
  case 0x10: REGSETBOFH(NullRunV, V, A & 0x3); CalcHappyColor(); break;
  case 0x14: REGSETBOFH(HSync, V, A & 0x3); break;
 }
}

void RAINBOW_Write16(uint32 A, uint16 V)
{
 int msh = A & 0x2;

 //printf("RAINBOW Wr16: %08x %04x\n", A, V);
 switch(A & 0x1C)
 {
  case 0x00: REGSETBOFH(HScroll, V & 0xFF, (A & 0x2) >> 1); HScroll &= 0x1FF; break;
  case 0x04: REGSETHW(Control, V, msh); break;
  case 0x08: REGSETHW(NullRunY, V, msh); CalcHappyColor(); break;
  case 0x0C: REGSETHW(NullRunU, V, msh); CalcHappyColor(); break;
  case 0x10: REGSETHW(NullRunV, V, msh); CalcHappyColor(); break;
  case 0x14: REGSETHW(HSync, V, msh); break;
 }
}

void RAINBOW_ForceTransferReset(void)
{
 RasterReadPos = 0;
 DecodeFormat[0] = DecodeFormat[1] = -1;
}

void RAINBOW_SwapBuffers(void)
{
 DecodeBufferWhichRead ^= 1;
 RasterReadPos = 0;
}

void RAINBOW_DecodeBlock(bool arg_FirstDecode, bool Skip)
{
   uint8 block_type;
   int32 block_size;
   int icount;
   int which_buffer = DecodeBufferWhichRead ^ 1;

   if(!(Control & 0x01))
   {
    puts("Rainbow decode when disabled!!");
    return;
   }

   if(arg_FirstDecode)
   {
    FirstDecode = true;
    GarbageData = false;
    rb_row = 0;              // new transfer: back to the top strip row
   }

   if(GarbageData)
    icount = 0;
   else
    icount = 0x200;

   do
   {
    do
    {
     while(KING_RB_Fetch() != 0xFF && icount > 0)
      icount--;

     block_type = KING_RB_Fetch();
     //if(icount > 0 && block_type != 0xF0 && block_type != 0xF1 && block_type != 0xF2 && block_type != 0xF3 && block_type != 0xF8 && block_type != 0xFF)
     //if(icount > 0 && block_type == 0x11)
     // printf("%02x\n", block_type);
     icount--;
    } while(block_type != 0xF0 && block_type != 0xF1 && block_type != 0xF2 && block_type != 0xF3 && block_type != 0xF8 && block_type != 0xFF && icount > 0);

    {
     uint16 tmp;
     
     tmp = KING_RB_Fetch() << 8;
     tmp |= KING_RB_Fetch() << 0;

     block_size = (int16)tmp;
    }

    block_size -= 2;
    if(block_type == 0xFF && block_size <= 0)
     for(int i = 0; i < 128; i++,icount--) KING_RB_Fetch();

    //fprintf(stderr, "Block: %d\n", block_size);
   } while(block_size <= 0 && icount > 0);

   //if(!GarbageData && icount < 500)
   //{
   // FXDBG("Partial garbage data. %d", icount);
   //}

   //printf("%d\n", icount);
   if(icount <= 0)
   {
    FXDBG("Garbage data.");
    GarbageData = true;
    //printf("Dooom: %d\n");
    DecodeFormat[which_buffer] = 0;
    memset(DecodeBuffer[which_buffer], 0, 0x2000);
    goto BufferNoDecode;
   }

   if(block_type == 0xf8 || block_type == 0xff)
    DecodeFormat[which_buffer] = 1;
   else
    DecodeFormat[which_buffer] = 0;

   if(block_type == 0xF8 || block_type == 0xFF)
   {
    if(block_type == 0xFF)
    {
     for(int q = 0; q < 2; q++)
      for(int i = 0; i < 64; i++)
      {
       uint8 meow = KING_RB_Fetch();

       QuantTables[q][i] = meow; 
       QuantTablesBase[q][i] = meow;
      }
     block_size -= 128;
    }

    InitBits(block_size);

    int32 dc_y = 0, dc_u = 0, dc_v = 0;
    uint32 *dest_base = (uint32 *)DecodeBuffer[which_buffer];
    for(int column = 0; column < 16; column++)
    {
     uint32 *dest_base_column = &dest_base[column * 16];
     int zeroes = 0;

     dc_y += get_dc_y_coeff(&zeroes);

     if(zeroes) // If set, clear the number of columns
     {
      do
      {
       if(column < 16)
       {
	dest_base_column = &dest_base[column * 16];

        const int32 nr_row = rb_row % RB_STRIPS_PER_FRAME;

        if(RB_NULLRUN_HOLD && rb_row_cache_valid[nr_row])
        {
         // Unchanged region: keep what was here in the previous frame.
         const uint32 *prev = rb_row_cache[nr_row];

         for(int y = 0; y < 16; y++)
          for(int x = 0; x < 16; x++)
           dest_base_column[y * 256 + x] = prev[y * 256 + column * 16 + x];

        }
        else
        {
         for(int y = 0; y < 16; y++)
          for(int x = 0; x < 16; x++)
           dest_base_column[y * 256 + x] = HappyColor;
        }
       }
       column++;
       zeroes--;
      } while(zeroes);
      column--; // Fix for the column autoincrement in the while(zeroes) loop
      dc_y = dc_u = dc_v = 0;
     }
     else
     {
      int32 dct_y[256];
      int32 dct_u[64];
      int32 dct_v[64];

      // Y/Luma, 16x16 components
      // ---------
      // | A | C |
      // |-------|
      // | B | D |
      // ---------
      // A (0, 0)
      decode(&dct_y[0x00], QuantTables[0], dc_y, ac_y_qlut);

      // B (0, 1)
      dc_y += get_dc_y_coeff(&zeroes);
      decode(&dct_y[0x40], QuantTables[0], dc_y, ac_y_qlut);

      // C (1, 0)
      dc_y += get_dc_y_coeff(&zeroes);
      decode(&dct_y[0x80], QuantTables[0], dc_y, ac_y_qlut);

      // D (1, 1)
      dc_y += get_dc_y_coeff(&zeroes);
      decode(&dct_y[0xC0], QuantTables[0], dc_y, ac_y_qlut);

      // U, 8x8 components
      dc_u += get_dc_uv_coeff();
      decode(&dct_u[0x00], QuantTables[1], dc_u, ac_uv_qlut);

      // V, 8x8 components
      dc_v += get_dc_uv_coeff();
      decode(&dct_v[0x00], QuantTables[1], dc_v, ac_uv_qlut);

      if(Skip)
       continue;

      IDCT(&dct_y[0x00]);
      IDCT(&dct_y[0x40]);
      IDCT(&dct_y[0x80]);
      IDCT(&dct_y[0xC0]);
      IDCT(&dct_u[0x00]);
      IDCT(&dct_v[0x00]);

      for(int y = 0; y < 16; y++)
       for(int x = 0; x < 16; x++)
        dest_base_column[y * 256 + x] = clamp_to_u8(dct_y[y * 8 + (x & 0x7) + ((x & 0x8) << 4)] + 0x80) << 16;

      if(!ChromaIP)
      {
       for(int y = 0; y < 8; y++)
       {
        for(int x = 0; x < 8; x++)
        {
         uint32 component_uv = (clamp_to_u8(dct_u[y * 8 + x] + 0x80) << 8) | clamp_to_u8(dct_v[y * 8 + x] + 0x80);
         dest_base_column[y * 512 + (256 * 0) + x * 2 + 0] |= component_uv;
         dest_base_column[y * 512 + (256 * 0) + x * 2 + 1] |= component_uv;
         dest_base_column[y * 512 + (256 * 1) + x * 2 + 0] |= component_uv;
         dest_base_column[y * 512 + (256 * 1) + x * 2 + 1] |= component_uv;
        }
       }
      }
      else
      {
       for(int y = 0; y < 8; y++)
       {
        for(int x = 0; x < 8; x++)
        {
         uint32 component_uv = (clamp_to_u8(dct_u[y * 8 + x] + 0x80) << 8) | clamp_to_u8(dct_v[y * 8 + x] + 0x80);
 	 dest_base_column[y * 512 + (256 * 1) + x * 2 + 0] |= component_uv;
        }
       }
      }
     }
    }

    // BLOCKS v32: drain trailing quantizer rescales.
    //
    // The encoder emits rescale opcodes AFTER the last macroblock column, to set
    // up the quantizer for the NEXT strip. The 16-column loop above exits as soon
    // as column reaches 16, so those opcodes are never read and the following
    // strip decodes with a stale quantizer. Measured on QoQ Disc B: strips ending
    // on q0=2 carry a trailing RESCALE back to q0=3 that Mednafen discards, which
    // is why the leading columns of the next strip decode one quantizer step too
    // weak and render dark flat blocks as grey instead of black.
    //
    // Only rescale opcodes are consumed here; the first non-rescale code is the
    // zero padding that fills out the block, and we stop there.
    while(bits_bytes_left > 0)
    {
     uint32 rawbits = GetBits(9, MDFNBITS_PEEK);
     uint32 code = dc_y_qlut[rawbits].val;

     if(code < 0x10)
      break;

     SkipBits(dc_y_qlut[rawbits].bitc);
     code -= 0x10;

     for(int i = 0; i < 64; i++)
     {
      uint32 coeff = (QuantTablesBase[0][i] * code) >> 2;
      if(coeff < 1) coeff = 1; else if(coeff > 0xFE) coeff = 0xFE;
      QuantTables[0][i] = coeff;

      if(i)
       coeff = (QuantTablesBase[1][i] * code) >> 2;
      else
       coeff = (QuantTablesBase[1][i]) >> 2;

      if(coeff < 1) coeff = 1; else if(coeff > 0xFE) coeff = 0xFE;
      QuantTables[1][i] = coeff;
     }
    }

    // Do bilinear interpolation on the chroma channels:
    if(!Skip && ChromaIP)
    {
     for(int y = 0; y < 16; y+= 2)
     {
      uint32 *linebase = &dest_base[y * 256];
      uint32 *linebase1 = &dest_base[(y + 1) * 256];

      for(int x = 0; x < 254; x += 2)
      {
       unsigned int u, v;

       u = (((linebase1[x] >> 8) & 0xFF) + ((linebase1[x + 2] >> 8) & 0xFF)) >> 1;
       v = (((linebase1[x] >> 0) & 0xFF) + ((linebase1[x + 2] >> 0) & 0xFF)) >> 1;

       linebase1[x + 1] = (linebase1[x + 1] & ~ 0xFFFF) | (u << 8) | v;
      }

      linebase1[0xFF] = (linebase1[0xFF] & ~ 0xFFFF) | (linebase1[0xFE] & 0xFFFF);

      if(FirstDecode)
      {
       for(int x = 0; x < 256; x++) linebase[x] = (linebase[x] & ~ 0xFFFF) | (linebase1[x] & 0xFFFF);
       FirstDecode = 0;
      }
      else
       for(int x = 0; x < 256; x++)
       {
        unsigned int u, v;
 
        u = (((LastLine[x] >> 8) & 0xFF) + ((linebase1[x] >> 8) & 0xFF)) >> 1;
        v = (((LastLine[x] >> 0) & 0xFF) + ((linebase1[x] >> 0) & 0xFF)) >> 1;

        linebase[x] = (linebase[x] & ~ 0xFFFF) | (u << 8) | v;
       }

      memcpy(LastLine, linebase1, 256 * 4);
     }
    } // End chroma interpolation

    // ---- replace columns whose decoded result is noise, not picture -------
    if(RB_FIX_NOISY_COLUMNS && !Skip)
    {
     uint32 colnoise[16];
     uint32 colchroma[16];

     for(int c = 0; c < 16; c++)
     {
      const int bx = c * 16;
      uint32 acc = 0;
      uint32 acc_c = 0;

      for(int y = 0; y < 16; y++)
      {
       const uint32 *line = &dest_base[y * 256 + bx];

       for(int x = 0; x < 15; x++)
       {
        const uint32 p0 = line[x];
        const uint32 p1 = line[x + 1];

        const int32 a = (int32)((p0 >> 16) & 0xFF);
        const int32 b = (int32)((p1 >> 16) & 0xFF);
        acc += (uint32)(a > b ? a - b : b - a);

        const int32 u0 = (int32)((p0 >> 8) & 0xFF);
        const int32 u1 = (int32)((p1 >> 8) & 0xFF);
        const int32 v0 = (int32)(p0 & 0xFF);
        const int32 v1 = (int32)(p1 & 0xFF);

        acc_c += (uint32)(u0 > u1 ? u0 - u1 : u1 - u0);
        acc_c += (uint32)(v0 > v1 ? v0 - v1 : v1 - v0);
       }
      }
      colnoise[c]  = acc / (16 * 15);
      colchroma[c] = acc_c / (16 * 15 * 2);
     }

     // median of the strip's own columns, so the test adapts to the content
     uint32 sorted[16];
     for(int i = 0; i < 16; i++) sorted[i] = colnoise[i];
     for(int i = 1; i < 16; i++)
     {
      const uint32 key = sorted[i];
      int j = i - 1;
      while(j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
      sorted[j + 1] = key;
     }
     const uint32 median = (sorted[7] + sorted[8]) / 2;

     uint32 sorted_c[16];
     for(int i = 0; i < 16; i++) sorted_c[i] = colchroma[i];
     for(int i = 1; i < 16; i++)
     {
      const uint32 key = sorted_c[i];
      int j = i - 1;
      while(j >= 0 && sorted_c[j] > key) { sorted_c[j + 1] = sorted_c[j]; j--; }
      sorted_c[j + 1] = key;
     }
     const uint32 median_c = (sorted_c[7] + sorted_c[8]) / 2;

     int  first_bad = -1;
     bool bad[16];

     for(int c = 0; c < 16; c++)
     {
      const uint32 v  = colnoise[c];
      const uint32 vc = colchroma[c];

      // A column is corrupt if either luma or chroma is both above an absolute
      // floor and well above the median of its own strip, so the test adapts to
      // the content rather than assuming a fixed level of detail.
      bad[c] = (v  >= RB_NOISE_FLOOR  && v  >= median   * RB_NOISE_RATIO) ||
               (vc >= RB_CHROMA_FLOOR && vc >= median_c * RB_NOISE_RATIO);

      if(bad[c] && first_bad < 0)
       first_bad = c;
     }

     // Absorb short gaps, but only short ones.
     {
      bool grown[16];
      for(int c = 0; c < 16; c++) grown[c] = bad[c];

      for(int c = 0; c < 16; c++)
      {
       if(!bad[c]) continue;

       for(int g = 2; g <= RB_MAX_GAP + 1; g++)
       {
        if(c + g > 15 || !bad[c + g]) continue;

        for(int k = c + 1; k < c + g; k++) grown[k] = true;
        break;
       }
      }
      for(int c = 0; c < 16; c++) bad[c] = grown[c];
     }

     // Repair the whole span between the first and last bad column. Once the
     // decode has gone wrong, columns in between are unreliable even if their
     // own noise happens to fall below the threshold - patching only the
     // flagged ones would leave visible gaps.
     const int32 row = rb_row % RB_STRIPS_PER_FRAME;

     // Either trigger condemns the strip: the decoded picture looks like noise,
     // or the block ran substantially short of its declared size.
     const bool starved = (rb_starve_bytes > RB_STARVE_THRESHOLD);
     const bool condemned = (first_bad >= 0) || starved;

     if(RB_FIX_NOISY_COLUMNS && condemned && rb_row_cache_valid[row])
     {
      // Preferred repair: hold this strip row from the previous frame.
      memcpy(dest_base, rb_row_cache[row], 256 * 16 * sizeof(uint32));
     }
     else if(RB_FIX_NOISY_COLUMNS && first_bad >= 0)
     {
      int c = 0;

      while(c < 16)
      {
       if(!bad[c]) { c++; continue; }

       const int run_start = c;
       while(c < 16 && bad[c]) c++;
       const int run_end = c - 1;

       // Source: the nearest clean column to the left of this run, copied as a
       // whole 16x16 block so its texture is preserved.
       //
       // NOTE: copy the matching x within the column, not a single pixel from
       // it. Reading one fixed x and writing it across the whole width makes
       // every row a flat colour, which renders as conspicuous horizontal bars
       // - worse than the artifact being repaired.
       //
       // With no clean column to the left, fall back to the null-run colour the
       // format itself uses for skipped columns.
       const int src_col = run_start - 1;

       for(int rc = run_start; rc <= run_end; rc++)
       {
        uint32 *dst = &dest_base[rc * 16];

        for(int y = 0; y < 16; y++)
         for(int x = 0; x < 16; x++)
          dst[y * 256 + x] = (src_col >= 0)
                           ? dest_base[src_col * 16 + y * 256 + x]
                           : HappyColor;
       }
      }

     }

     if(first_bad < 0 && !starved)
     {
      // Clean strip: remember it as this row's fallback for later frames.
      // A strip that starved is never cached, so corruption cannot be held
      // forward and re-displayed as if it were good picture.
      memcpy(rb_row_cache[row], dest_base, 256 * 16 * sizeof(uint32));
      rb_row_cache_valid[row] = true;
     }

    }
    // ----------------------------------------------------------------------
   } // end jpeg-like decoding
   else 
   {
    // Earlier code confines the set to F0,F1,F2, and F3.
    // F0 = (4, 0xF), F1 = (3, 0x7), F2 = (0x2, 0x3), F3 = 0x1, 0x1)
    const unsigned int plt_shift = 4 - (block_type & 0x3);
    const unsigned int crl_mask = (1 << plt_shift) - 1;
    int x = 0;
    
    while(block_size > 0)
    {
     uint8 boot;
     unsigned int rle_count;

     boot = KING_RB_Fetch();
     block_size--;

     if(boot == 0xFF)
     {
      KING_RB_Fetch();
      block_size--;
     }

     if(!(boot & crl_mask)) // Expand mode?
     {
      rle_count = KING_RB_Fetch();
      block_size--;
      if(rle_count == 0xFF) 
      {
       KING_RB_Fetch();
       block_size--;
      }
      rle_count++;
     }
     else
      rle_count = boot & crl_mask;

     for(unsigned int i = 0; i < rle_count; i++)
     {
      if(x >= 0x2000) 
      {
       //puts("Oops");
       break; // Don't overflow our decode buffer!
      }
      DecodeBuffer[which_buffer][x] = (boot >> plt_shift);
      x++;
     }
    }
   } // end RLE decoding

   //for(int i = 0; i < 8 + block_size; i++)
   // KING_RB_Fetch();

  BufferNoDecode: ;

 // Advance the strip row for EVERY block, not just decoded YUV ones.
 //
 // The row index is what the strip cache and the null-run hold are keyed on, so
 // it has to track the actual sequence of strips. Advancing it only inside the
 // YUV path would leave it stalled on palette/RLE blocks (types F0-F3) and on
 // blocks decoded with Skip set, after which every subsequent lookup would read
 // the wrong row. QoQ appears to use YUV blocks throughout, but the index must
 // not depend on that.
 rb_row++;
 if(rb_row >= RB_STRIPS_PER_FRAME)
  rb_row = 0;
}

void KING_Moo(void);

// NOTE:  layer_or and palette_ptr are optimizations, the real RAINBOW chip knows not of such things.
int RAINBOW_FetchRaster(uint32 *linebuffer, uint32 layer_or, uint32 *palette_ptr)
{
 int ret;

 ret = DecodeFormat[DecodeBufferWhichRead];

 if(linebuffer)
 {
  linebuffer = MDFN_ASSUME_ALIGNED(linebuffer, 8);
  //
  if(DecodeFormat[DecodeBufferWhichRead] == -1) // None
  {
   MDFN_FastArraySet(linebuffer, 0, 256);
  }
  else if(DecodeFormat[DecodeBufferWhichRead] == 1)	// YUV
  {
   uint32 *in_ptr = (uint32*)&DecodeBuffer[DecodeBufferWhichRead][RasterReadPos * 256 * 4];

   if(Control & 0x2)	// Endless scroll mode:
   {
    uint8 tmpss = HScroll;

    for(int x = 0; x < 256; x++)
    {
     linebuffer[x] = in_ptr[tmpss] | layer_or;
     tmpss++;
    }
   }
   else // Non-endless
   {
    uint16 tmpss = HScroll & 0x1FF;

    for(int x = 0; x < 256; x++)
    {
     linebuffer[x] = (tmpss < 256) ? (in_ptr[tmpss] | layer_or) : 0;
     tmpss = (tmpss + 1) & 0x1FF;
    }
   }
   MDFN_FastArraySet(in_ptr, 0, 256);
  }
  else if(DecodeFormat[DecodeBufferWhichRead] == 0)	// Palette
  {
   uint8 *in_ptr = &DecodeBuffer[DecodeBufferWhichRead][RasterReadPos * 256];

   if(Control & 0x2)    // Endless scroll mode:
   {
    uint8 tmpss = HScroll;

    for(int x = 0; x < 256; x++)
    {
     linebuffer[x] = in_ptr[tmpss] ? (palette_ptr[in_ptr[tmpss]] | layer_or) : 0;
     tmpss++;
    }
   }
   else // Non-endless
   {
    uint16 tmpss = HScroll & 0x1FF;

    for(int x = 0; x < 256; x++)
    {
     linebuffer[x] = (tmpss < 256 && in_ptr[tmpss]) ? (palette_ptr[in_ptr[tmpss]] | layer_or) : 0;
     tmpss = (tmpss + 1) & 0x1FF;
    }
   }
   //MDFN_FastU32MemsetM8((uint32 *)in_ptr, 0, 256 / 4);
  }
 }

 RasterReadPos = (RasterReadPos + 1) & 0xF;

 if(!RasterReadPos)
  DecodeFormat[DecodeBufferWhichRead] = -1;     // Invalidate this buffer.

 //printf("Fetch: %d, buffer: %d\n", RasterReadPos, DecodeBufferWhichRead);
 return(ret);
}

void RAINBOW_Reset(void)
{
 Control = 0;
 NullRunY = NullRunU = NullRunV = 0;
 HScroll = 0;
 RasterReadPos = 0;
 DecodeBufferWhichRead = 0;

 memset(QuantTables, 0, sizeof(QuantTables));
 memset(QuantTablesBase, 0, sizeof(QuantTablesBase));
 DecodeFormat[0] = DecodeFormat[1] = -1;

 CalcHappyColor();
}

void RAINBOW_StateAction(StateMem *sm, const unsigned load, const bool data_only)
{
 SFORMAT StateRegs[] =
 {
   SFVAR(HScroll),
   SFVAR(Control),
   SFVAR(RasterReadPos),
   SFVAR(DecodeBufferWhichRead),
   SFVAR(NullRunY),
   SFVAR(NullRunU),
   SFVAR(NullRunV),
   SFVAR(HSync),
   SFVAR(DecodeFormat),
   SFPTR8(DecodeBuffer[0], 0x2000 * 4),
   SFPTR8(DecodeBuffer[1], 0x2000 * 4),
   SFEND
 };

 MDFNSS_StateAction(sm, load, data_only, StateRegs, "RBOW");

 if(load)
 {
  RasterReadPos &= 0xF;
  DecodeBufferWhichRead &= 0x1;

  CalcHappyColor();
 }
}

}
