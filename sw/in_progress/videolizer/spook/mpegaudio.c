/*
 * The simplest mpeg audio layer 2 encoder
 * Copyright (c) 2000, 2001 Fabrice Bellard.
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * alternative bitstream reader & writer by Michael Niedermayer <michaelni@gmx.at>
 *
 * MPEG1 L2 implementation adapted from FFMPEG by Nathan Lutchansky
 */
 
#include "mpegaudio.h"

const uint16_t mpa_bitrate_tab[2][3][15] = {
    { {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448 },
      {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384 },
      {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320 } },
    { {0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256},
      {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160},
      {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160}
    }
};


const uint16_t mpa_freq_tab[3] = { 44100, 48000, 32000 };

/*******************************************************/
/* half mpeg encoding window (full precision) */
const int32_t mpa_enwindow[257] = {
     0,    -1,    -1,    -1,    -1,    -1,    -1,    -2,
    -2,    -2,    -2,    -3,    -3,    -4,    -4,    -5,
    -5,    -6,    -7,    -7,    -8,    -9,   -10,   -11,
   -13,   -14,   -16,   -17,   -19,   -21,   -24,   -26,
   -29,   -31,   -35,   -38,   -41,   -45,   -49,   -53,
   -58,   -63,   -68,   -73,   -79,   -85,   -91,   -97,
  -104,  -111,  -117,  -125,  -132,  -139,  -147,  -154,
  -161,  -169,  -176,  -183,  -190,  -196,  -202,  -208,
   213,   218,   222,   225,   227,   228,   228,   227,
   224,   221,   215,   208,   200,   189,   177,   163,
   146,   127,   106,    83,    57,    29,    -2,   -36,
   -72,  -111,  -153,  -197,  -244,  -294,  -347,  -401,
  -459,  -519,  -581,  -645,  -711,  -779,  -848,  -919,
  -991, -1064, -1137, -1210, -1283, -1356, -1428, -1498,
 -1567, -1634, -1698, -1759, -1817, -1870, -1919, -1962,
 -2001, -2032, -2057, -2075, -2085, -2087, -2080, -2063,
  2037,  2000,  1952,  1893,  1822,  1739,  1644,  1535,
  1414,  1280,  1131,   970,   794,   605,   402,   185,
   -45,  -288,  -545,  -814, -1095, -1388, -1692, -2006,
 -2330, -2663, -3004, -3351, -3705, -4063, -4425, -4788,
 -5153, -5517, -5879, -6237, -6589, -6935, -7271, -7597,
 -7910, -8209, -8491, -8755, -8998, -9219, -9416, -9585,
 -9727, -9838, -9916, -9959, -9966, -9935, -9863, -9750,
 -9592, -9389, -9139, -8840, -8492, -8092, -7640, -7134,
  6574,  5959,  5288,  4561,  3776,  2935,  2037,  1082,
    70,  -998, -2122, -3300, -4533, -5818, -7154, -8540,
 -9975,-11455,-12980,-14548,-16155,-17799,-19478,-21189,
-22929,-24694,-26482,-28289,-30112,-31947,-33791,-35640,
-37489,-39336,-41176,-43006,-44821,-46617,-48390,-50137,
-51853,-53534,-55178,-56778,-58333,-59838,-61289,-62684,
-64019,-65290,-66494,-67629,-68692,-69679,-70590,-71420,
-72169,-72835,-73415,-73908,-74313,-74630,-74856,-74992,
 75038,
};


/*******************************************************/
/* layer 2 tables */

const int sblimit_table[5] = { 27 , 30 , 8, 12 , 30 };

const int quant_steps[17] = {
    3,     5,    7,    9,    15,
    31,    63,  127,  255,   511,
    1023,  2047, 4095, 8191, 16383,
    32767, 65535
};

/* we use a negative value if grouped */
const int quant_bits[17] = {
    -5,  -7,  3, -10, 4, 
     5,  6,  7,  8,  9,
    10, 11, 12, 13, 14,
    15, 16 
};

/* encoding tables which give the quantization index. Note how it is
   possible to store them efficiently ! */
static const unsigned char alloc_table_0[] = {
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
};

static const unsigned char alloc_table_1[] = {
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  2,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 3,  0,  1,  2,  3,  4,  5, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
 2,  0,  1, 16, 
};

static const unsigned char alloc_table_2[] = {
 4,  0,  1,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 
 4,  0,  1,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
};

static const unsigned char alloc_table_3[] = {
 4,  0,  1,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 
 4,  0,  1,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
};

static const unsigned char alloc_table_4[] = {
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 
 4,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 3,  0,  1,  3,  4,  5,  6,  7, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
 2,  0,  1,  3, 
};


const uint8_t ff_sqrt_tab[128]={
        0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
        9, 9, 9, 9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11
};

const uint8_t ff_log2_tab[256]={
        0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
        5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
        6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
        6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};

void init_put_bits(PutBitContext *s, uint8_t *buffer, int buffer_size)
{
    s->buf = buffer;
    s->buf_end = s->buf + buffer_size;
#ifdef ALT_BITSTREAM_WRITER
    s->index=0;
    ((uint32_t*)(s->buf))[0]=0;
//    memset(buffer, 0, buffer_size);
#else
    s->buf_ptr = s->buf;
    s->bit_left=32;
    s->bit_buf=0;
#endif
}

/* return the number of bits output */
int get_bit_count(PutBitContext *s)
{
#ifdef ALT_BITSTREAM_WRITER
    return s->index;
#else
    return (s->buf_ptr - s->buf) * 8 + 32 - s->bit_left;
#endif
}

void align_put_bits(PutBitContext *s)
{
#ifdef ALT_BITSTREAM_WRITER
    put_bits(s,(  - s->index) & 7,0);
#else
    put_bits(s,s->bit_left & 7,0);
#endif
}

/* pad the end of the output stream with zeros */
void flush_put_bits(PutBitContext *s)
{
#ifdef ALT_BITSTREAM_WRITER
    align_put_bits(s);
#else
    s->bit_buf<<= s->bit_left;
    while (s->bit_left < 32) {
        /* XXX: should test end of buffer */
        *s->buf_ptr++=s->bit_buf >> 24;
        s->bit_buf<<=8;
        s->bit_left+=8;
    }
    s->bit_left=32;
    s->bit_buf=0;
#endif
}

void put_string(PutBitContext * pbc, char *s)
{
    while(*s){
        put_bits(pbc, 8, *s);
        s++;
    }
    put_bits(pbc, 8, 0);
}


/* bitrate is in kb/s */
int l2_select_table(int bitrate, int nb_channels, int freq, int lsf)
{
    int ch_bitrate, table;
    
    ch_bitrate = bitrate / nb_channels;
    if (!lsf) {
        if ((freq == 48000 && ch_bitrate >= 56) ||
            (ch_bitrate >= 56 && ch_bitrate <= 80)) 
            table = 0;
        else if (freq != 48000 && ch_bitrate >= 96) 
            table = 1;
        else if (freq != 32000 && ch_bitrate <= 48) 
            table = 2;
        else 
            table = 3;
    } else {
        table = 4;
    }
    return table;
}


const unsigned char *alloc_tables[5] = 
{ alloc_table_0, alloc_table_1, alloc_table_2, alloc_table_3, alloc_table_4, };






/* define it to use floats in quantization (I don't like floats !) */
//#define USE_FLOATS

int MPA_encode_init(MpegAudioContext *s, int freq, int bitrate, int channels)
{
    int i, v, table;
    float a;

    if (channels > 2)
        return -1;
    bitrate = bitrate / 1000;
    s->nb_channels = channels;
    s->freq = freq;
    s->bit_rate = bitrate;
    //avctx->frame_size = MPA_FRAME_SIZE;

    /* encoding freq */
    s->lsf = 0;
    for(i=0;i<3;i++) {
        if (mpa_freq_tab[i] == freq) 
            break;
        if ((mpa_freq_tab[i] / 2) == freq) {
            s->lsf = 1;
            break;
        }
    }
    if (i == 3)
        return -1;
    s->freq_index = i;

    /* encoding bitrate & frequency */
    for(i=0;i<15;i++) {
        if (mpa_bitrate_tab[s->lsf][1][i] == bitrate) 
            break;
    }
    if (i == 15)
        return -1;
    s->bitrate_index = i;

    /* compute total header size & pad bit */
    
    a = (float)(bitrate * 1000 * MPA_FRAME_SIZE) / (freq * 8.0);
    s->frame_size = ((int)a) * 8;

    /* frame fractional size to compute padding */
    s->frame_frac = 0;
    s->frame_frac_incr = (int)((a - floor(a)) * 65536.0);
    
    /* select the right allocation table */
    table = l2_select_table(bitrate, s->nb_channels, freq, s->lsf);

    /* number of used subbands */
    s->sblimit = sblimit_table[table];
    s->alloc_table = alloc_tables[table];

#ifdef DEBUG
    av_log(avctx, AV_LOG_DEBUG, "%d kb/s, %d Hz, frame_size=%d bits, table=%d, padincr=%x\n", 
           bitrate, freq, s->frame_size, table, s->frame_frac_incr);
#endif

    for(i=0;i<s->nb_channels;i++)
        s->samples_offset[i] = 0;

    for(i=0;i<257;i++) {
        int v;
        v = mpa_enwindow[i];
#if WFRAC_BITS != 16
        v = (v + (1 << (16 - WFRAC_BITS - 1))) >> (16 - WFRAC_BITS);
#endif
        filter_bank[i] = v;
        if ((i & 63) != 0)
            v = -v;
        if (i != 0)
            filter_bank[512 - i] = v;
    }

    for(i=0;i<64;i++) {
        v = (int)(pow(2.0, (3 - i) / 3.0) * (1 << 20));
        if (v <= 0)
            v = 1;
        scale_factor_table[i] = v;
#ifdef USE_FLOATS
        scale_factor_inv_table[i] = pow(2.0, -(3 - i) / 3.0) / (float)(1 << 20);
#else
#define P 15
        scale_factor_shift[i] = 21 - P - (i / 3);
        scale_factor_mult[i] = (1 << P) * pow(2.0, (i % 3) / 3.0);
#endif
    }
    for(i=0;i<128;i++) {
        v = i - 64;
        if (v <= -3)
            v = 0;
        else if (v < 0)
            v = 1;
        else if (v == 0)
            v = 2;
        else if (v < 3)
            v = 3;
        else 
            v = 4;
        scale_diff_table[i] = v;
    }

    for(i=0;i<17;i++) {
        v = quant_bits[i];
        if (v < 0) 
            v = -v;
        else
            v = v * 3;
        total_quant_bits[i] = 12 * v;
    }

//    avctx->coded_frame= avcodec_alloc_frame();
//    avctx->coded_frame->key_frame= 1;

    return 0;
}

/* 32 point floating point IDCT without 1/sqrt(2) coef zero scaling */
static void idct32(int *out, int *tab)
{
    int i, j;
    int *t, *t1, xr;
    const int *xp = costab32;

    for(j=31;j>=3;j-=2) tab[j] += tab[j - 2];
    
    t = tab + 30;
    t1 = tab + 2;
    do {
        t[0] += t[-4];
        t[1] += t[1 - 4];
        t -= 4;
    } while (t != t1);

    t = tab + 28;
    t1 = tab + 4;
    do {
        t[0] += t[-8];
        t[1] += t[1-8];
        t[2] += t[2-8];
        t[3] += t[3-8];
        t -= 8;
    } while (t != t1);
    
    t = tab;
    t1 = tab + 32;
    do {
        t[ 3] = -t[ 3];    
        t[ 6] = -t[ 6];    
        
        t[11] = -t[11];    
        t[12] = -t[12];    
        t[13] = -t[13];    
        t[15] = -t[15]; 
        t += 16;
    } while (t != t1);

    
    t = tab;
    t1 = tab + 8;
    do {
        int x1, x2, x3, x4;
        
        x3 = MUL(t[16], FIX(SQRT2*0.5));
        x4 = t[0] - x3;
        x3 = t[0] + x3;
        
        x2 = MUL(-(t[24] + t[8]), FIX(SQRT2*0.5));
        x1 = MUL((t[8] - x2), xp[0]);
        x2 = MUL((t[8] + x2), xp[1]);

        t[ 0] = x3 + x1;
        t[ 8] = x4 - x2;
        t[16] = x4 + x2;
        t[24] = x3 - x1;
        t++;
    } while (t != t1);

    xp += 2;
    t = tab;
    t1 = tab + 4;
    do {
        xr = MUL(t[28],xp[0]);
        t[28] = (t[0] - xr);
        t[0] = (t[0] + xr);

        xr = MUL(t[4],xp[1]);
        t[ 4] = (t[24] - xr);
        t[24] = (t[24] + xr);
        
        xr = MUL(t[20],xp[2]);
        t[20] = (t[8] - xr);
        t[ 8] = (t[8] + xr);
            
        xr = MUL(t[12],xp[3]);
        t[12] = (t[16] - xr);
        t[16] = (t[16] + xr);
        t++;
    } while (t != t1);
    xp += 4;

    for (i = 0; i < 4; i++) {
        xr = MUL(tab[30-i*4],xp[0]);
        tab[30-i*4] = (tab[i*4] - xr);
        tab[   i*4] = (tab[i*4] + xr);
        
        xr = MUL(tab[ 2+i*4],xp[1]);
        tab[ 2+i*4] = (tab[28-i*4] - xr);
        tab[28-i*4] = (tab[28-i*4] + xr);
        
        xr = MUL(tab[31-i*4],xp[0]);
        tab[31-i*4] = (tab[1+i*4] - xr);
        tab[ 1+i*4] = (tab[1+i*4] + xr);
        
        xr = MUL(tab[ 3+i*4],xp[1]);
        tab[ 3+i*4] = (tab[29-i*4] - xr);
        tab[29-i*4] = (tab[29-i*4] + xr);
        
        xp += 2;
    }

    t = tab + 30;
    t1 = tab + 1;
    do {
        xr = MUL(t1[0], *xp);
        t1[0] = (t[0] - xr);
        t[0] = (t[0] + xr);
        t -= 2;
        t1 += 2;
        xp++;
    } while (t >= tab);

    for(i=0;i<32;i++) {
        out[i] = tab[bitinv32[i]];
    }
}

#define WSHIFT (WFRAC_BITS + 15 - FRAC_BITS)

static void filter(MpegAudioContext *s, int ch, short *samples, int incr)
{
    short *p, *q;
    int sum, offset, i, j;
    int tmp[64];
    int tmp1[32];
    int *out;

    //    print_pow1(samples, 1152);

    offset = s->samples_offset[ch];
    out = &s->sb_samples[ch][0][0][0];
    for(j=0;j<36;j++) {
        /* 32 samples at once */
        for(i=0;i<32;i++) {
            s->samples_buf[ch][offset + (31 - i)] = be2me_16(samples[0]);
            samples += incr;
        }

        /* filter */
        p = s->samples_buf[ch] + offset;
        q = filter_bank;
        /* maxsum = 23169 */
        for(i=0;i<64;i++) {
            sum = p[0*64] * q[0*64];
            sum += p[1*64] * q[1*64];
            sum += p[2*64] * q[2*64];
            sum += p[3*64] * q[3*64];
            sum += p[4*64] * q[4*64];
            sum += p[5*64] * q[5*64];
            sum += p[6*64] * q[6*64];
            sum += p[7*64] * q[7*64];
            tmp[i] = sum;
            p++;
            q++;
        }
        tmp1[0] = tmp[16] >> WSHIFT;
        for( i=1; i<=16; i++ ) tmp1[i] = (tmp[i+16]+tmp[16-i]) >> WSHIFT;
        for( i=17; i<=31; i++ ) tmp1[i] = (tmp[i+16]-tmp[80-i]) >> WSHIFT;

        idct32(out, tmp1);

        /* advance of 32 samples */
        offset -= 32;
        out += 32;
        /* handle the wrap around */
        if (offset < 0) {
            memmove(s->samples_buf[ch] + SAMPLES_BUF_SIZE - (512 - 32), 
                    s->samples_buf[ch], (512 - 32) * 2);
            offset = SAMPLES_BUF_SIZE - 512;
        }
    }
    s->samples_offset[ch] = offset;

    //    print_pow(s->sb_samples, 1152);
}

static void compute_scale_factors(unsigned char scale_code[SBLIMIT],
                                  unsigned char scale_factors[SBLIMIT][3], 
                                  int sb_samples[3][12][SBLIMIT],
                                  int sblimit)
{
    int *p, vmax, v, n, i, j, k, code;
    int index, d1, d2;
    unsigned char *sf = &scale_factors[0][0];
    
    for(j=0;j<sblimit;j++) {
        for(i=0;i<3;i++) {
            /* find the max absolute value */
            p = &sb_samples[i][0][j];
            vmax = abs(*p);
            for(k=1;k<12;k++) {
                p += SBLIMIT;
                v = abs(*p);
                if (v > vmax)
                    vmax = v;
            }
            /* compute the scale factor index using log 2 computations */
            if (vmax > 0) {
                n = av_log2(vmax);
                /* n is the position of the MSB of vmax. now 
                   use at most 2 compares to find the index */
                index = (21 - n) * 3 - 3;
                if (index >= 0) {
                    while (vmax <= scale_factor_table[index+1])
                        index++;
                } else {
                    index = 0; /* very unlikely case of overflow */
                }
            } else {
                index = 62; /* value 63 is not allowed */
            }

#if 0
            printf("%2d:%d in=%x %x %d\n", 
                   j, i, vmax, scale_factor_table[index], index);
#endif
            /* store the scale factor */
           // assert(index >=0 && index <= 63);
            sf[i] = index;
        }

        /* compute the transmission factor : look if the scale factors
           are close enough to each other */
        d1 = scale_diff_table[sf[0] - sf[1] + 64];
        d2 = scale_diff_table[sf[1] - sf[2] + 64];
        
        /* handle the 25 cases */
        switch(d1 * 5 + d2) {
        case 0*5+0:
        case 0*5+4:
        case 3*5+4:
        case 4*5+0:
        case 4*5+4:
            code = 0;
            break;
        case 0*5+1:
        case 0*5+2:
        case 4*5+1:
        case 4*5+2:
            code = 3;
            sf[2] = sf[1];
            break;
        case 0*5+3:
        case 4*5+3:
            code = 3;
            sf[1] = sf[2];
            break;
        case 1*5+0:
        case 1*5+4:
        case 2*5+4:
            code = 1;
            sf[1] = sf[0];
            break;
        case 1*5+1:
        case 1*5+2:
        case 2*5+0:
        case 2*5+1:
        case 2*5+2:
            code = 2;
            sf[1] = sf[2] = sf[0];
            break;
        case 2*5+3:
        case 3*5+3:
            code = 2;
            sf[0] = sf[1] = sf[2];
            break;
        case 3*5+0:
        case 3*5+1:
        case 3*5+2:
            code = 2;
            sf[0] = sf[2] = sf[1];
            break;
        case 1*5+3:
            code = 2;
            if (sf[0] > sf[2])
              sf[0] = sf[2];
            sf[1] = sf[2] = sf[0];
            break;
        }
        
#if 0
        printf("%d: %2d %2d %2d %d %d -> %d\n", j, 
               sf[0], sf[1], sf[2], d1, d2, code);
#endif
        scale_code[j] = code;
        sf += 3;
    }
}

/* The most important function : psycho acoustic module. In this
   encoder there is basically none, so this is the worst you can do,
   but also this is the simpler. */
static void psycho_acoustic_model(MpegAudioContext *s, short smr[SBLIMIT])
{
    int i;

    for(i=0;i<s->sblimit;i++) {
        smr[i] = (int)(fixed_smr[i] * 10);
    }
}


#define SB_NOTALLOCATED  0
#define SB_ALLOCATED     1
#define SB_NOMORE        2

/* Try to maximize the smr while using a number of bits inferior to
   the frame size. I tried to make the code simpler, faster and
   smaller than other encoders :-) */
static void compute_bit_allocation(MpegAudioContext *s, 
                                   short smr1[MPA_MAX_CHANNELS][SBLIMIT],
                                   unsigned char bit_alloc[MPA_MAX_CHANNELS][SBLIMIT],
                                   int *padding)
{
    int i, ch, b, max_smr, max_ch, max_sb, current_frame_size, max_frame_size;
    int incr;
    short smr[MPA_MAX_CHANNELS][SBLIMIT];
    unsigned char subband_status[MPA_MAX_CHANNELS][SBLIMIT];
    const unsigned char *alloc;

    memcpy(smr, smr1, s->nb_channels * sizeof(short) * SBLIMIT);
    memset(subband_status, SB_NOTALLOCATED, s->nb_channels * SBLIMIT);
    memset(bit_alloc, 0, s->nb_channels * SBLIMIT);
    
    /* compute frame size and padding */
    max_frame_size = s->frame_size;
    s->frame_frac += s->frame_frac_incr;
    if (s->frame_frac >= 65536) {
        s->frame_frac -= 65536;
        s->do_padding = 1;
        max_frame_size += 8;
    } else {
        s->do_padding = 0;
    }

    /* compute the header + bit alloc size */
    current_frame_size = 32;
    alloc = s->alloc_table;
    for(i=0;i<s->sblimit;i++) {
        incr = alloc[0];
        current_frame_size += incr * s->nb_channels;
        alloc += 1 << incr;
    }
    for(;;) {
        /* look for the subband with the largest signal to mask ratio */
        max_sb = -1;
        max_ch = -1;
        max_smr = 0x80000000;
        for(ch=0;ch<s->nb_channels;ch++) {
            for(i=0;i<s->sblimit;i++) {
                if (smr[ch][i] > max_smr && subband_status[ch][i] != SB_NOMORE) {
                    max_smr = smr[ch][i];
                    max_sb = i;
                    max_ch = ch;
                }
            }
        }
#if 0
        printf("current=%d max=%d max_sb=%d alloc=%d\n", 
               current_frame_size, max_frame_size, max_sb,
               bit_alloc[max_sb]);
#endif        
        if (max_sb < 0)
            break;
        
        /* find alloc table entry (XXX: not optimal, should use
           pointer table) */
        alloc = s->alloc_table;
        for(i=0;i<max_sb;i++) {
            alloc += 1 << alloc[0];
        }

        if (subband_status[max_ch][max_sb] == SB_NOTALLOCATED) {
            /* nothing was coded for this band: add the necessary bits */
            incr = 2 + nb_scale_factors[s->scale_code[max_ch][max_sb]] * 6;
            incr += total_quant_bits[alloc[1]];
        } else {
            /* increments bit allocation */
            b = bit_alloc[max_ch][max_sb];
            incr = total_quant_bits[alloc[b + 1]] - 
                total_quant_bits[alloc[b]];
        }

        if (current_frame_size + incr <= max_frame_size) {
            /* can increase size */
            b = ++bit_alloc[max_ch][max_sb];
            current_frame_size += incr;
            /* decrease smr by the resolution we added */
            smr[max_ch][max_sb] = smr1[max_ch][max_sb] - quant_snr[alloc[b]];
            /* max allocation size reached ? */
            if (b == ((1 << alloc[0]) - 1))
                subband_status[max_ch][max_sb] = SB_NOMORE;
            else
                subband_status[max_ch][max_sb] = SB_ALLOCATED;
        } else {
            /* cannot increase the size of this subband */
            subband_status[max_ch][max_sb] = SB_NOMORE;
        }
    }
    *padding = max_frame_size - current_frame_size;
    //assert(*padding >= 0);

#if 0
    for(i=0;i<s->sblimit;i++) {
        printf("%d ", bit_alloc[i]);
    }
    printf("\n");
#endif
}

/*
 * Output the mpeg audio layer 2 frame. Note how the code is small
 * compared to other encoders :-)
 */
static void encode_frame(MpegAudioContext *s,
                         unsigned char bit_alloc[MPA_MAX_CHANNELS][SBLIMIT],
                         int padding)
{
    int i, j, k, l, bit_alloc_bits, b, ch;
    unsigned char *sf;
    int q[3];
    PutBitContext *p = &s->pb;

    /* header */

    put_bits(p, 12, 0xfff);
    put_bits(p, 1, 1 - s->lsf); /* 1 = mpeg1 ID, 0 = mpeg2 lsf ID */
    put_bits(p, 2, 4-2);  /* layer 2 */
    put_bits(p, 1, 1); /* no error protection */
    put_bits(p, 4, s->bitrate_index);
    put_bits(p, 2, s->freq_index);
    put_bits(p, 1, s->do_padding); /* use padding */
    put_bits(p, 1, 0);             /* private_bit */
    put_bits(p, 2, s->nb_channels == 2 ? MPA_STEREO : MPA_MONO);
    put_bits(p, 2, 0); /* mode_ext */
    put_bits(p, 1, 0); /* no copyright */
    put_bits(p, 1, 1); /* original */
    put_bits(p, 2, 0); /* no emphasis */

    /* bit allocation */
    j = 0;
    for(i=0;i<s->sblimit;i++) {
        bit_alloc_bits = s->alloc_table[j];
        for(ch=0;ch<s->nb_channels;ch++) {
            put_bits(p, bit_alloc_bits, bit_alloc[ch][i]);
        }
        j += 1 << bit_alloc_bits;
    }
    
    /* scale codes */
    for(i=0;i<s->sblimit;i++) {
        for(ch=0;ch<s->nb_channels;ch++) {
            if (bit_alloc[ch][i]) 
                put_bits(p, 2, s->scale_code[ch][i]);
        }
    }

    /* scale factors */
    for(i=0;i<s->sblimit;i++) {
        for(ch=0;ch<s->nb_channels;ch++) {
            if (bit_alloc[ch][i]) {
                sf = &s->scale_factors[ch][i][0];
                switch(s->scale_code[ch][i]) {
                case 0:
                    put_bits(p, 6, sf[0]);
                    put_bits(p, 6, sf[1]);
                    put_bits(p, 6, sf[2]);
                    break;
                case 3:
                case 1:
                    put_bits(p, 6, sf[0]);
                    put_bits(p, 6, sf[2]);
                    break;
                case 2:
                    put_bits(p, 6, sf[0]);
                    break;
                }
            }
        }
    }
    
    /* quantization & write sub band samples */

    for(k=0;k<3;k++) {
        for(l=0;l<12;l+=3) {
            j = 0;
            for(i=0;i<s->sblimit;i++) {
                bit_alloc_bits = s->alloc_table[j];
                for(ch=0;ch<s->nb_channels;ch++) {
                    b = bit_alloc[ch][i];
                    if (b) {
                        int qindex, steps, m, sample, bits;
                        /* we encode 3 sub band samples of the same sub band at a time */
                        qindex = s->alloc_table[j+b];
                        steps = quant_steps[qindex];
                        for(m=0;m<3;m++) {
                            sample = s->sb_samples[ch][k][l + m][i];
                            /* divide by scale factor */
#ifdef USE_FLOATS
                            {
                                float a;
                                a = (float)sample * scale_factor_inv_table[s->scale_factors[ch][i][k]];
                                q[m] = (int)((a + 1.0) * steps * 0.5);
                            }
#else
                            {
                                int q1, e, shift, mult;
                                e = s->scale_factors[ch][i][k];
                                shift = scale_factor_shift[e];
                                mult = scale_factor_mult[e];
                                
                                /* normalize to P bits */
                                if (shift < 0)
                                    q1 = sample << (-shift);
                                else
                                    q1 = sample >> shift;
                                q1 = (q1 * mult) >> P;
                                q[m] = ((q1 + (1 << P)) * steps) >> (P + 1);
                            }
#endif
                            if (q[m] >= steps)
                                q[m] = steps - 1;
                            //assert(q[m] >= 0 && q[m] < steps);
                        }
                        bits = quant_bits[qindex];
                        if (bits < 0) {
                            /* group the 3 values to save bits */
                            put_bits(p, -bits, 
                                     q[0] + steps * (q[1] + steps * q[2]));
#if 0
                            printf("%d: gr1 %d\n", 
                                   i, q[0] + steps * (q[1] + steps * q[2]));
#endif
                        } else {
#if 0
                            printf("%d: gr3 %d %d %d\n", 
                                   i, q[0], q[1], q[2]);
#endif                               
                            put_bits(p, bits, q[0]);
                            put_bits(p, bits, q[1]);
                            put_bits(p, bits, q[2]);
                        }
                    }
                }
                /* next subband in alloc table */
                j += 1 << bit_alloc_bits; 
            }
        }
    }

    /* padding */
    for(i=0;i<padding;i++)
        put_bits(p, 1, 0);

    /* flush */
    flush_put_bits(p);
}

int MPA_encode_frame(MpegAudioContext *s, unsigned char *frame, int buf_size,
				unsigned char *sampbuf, int step)
{
    short smr[MPA_MAX_CHANNELS][SBLIMIT];
    unsigned char bit_alloc[MPA_MAX_CHANNELS][SBLIMIT];
    int padding, i;

    for(i=0;i<s->nb_channels;i++) {
        filter(s, i, ((short*)(sampbuf)) + i, step >> 1);
    }

    for(i=0;i<s->nb_channels;i++) {
        compute_scale_factors(s->scale_code[i], s->scale_factors[i], 
                              s->sb_samples[i], s->sblimit);
    }
    for(i=0;i<s->nb_channels;i++) {
        psycho_acoustic_model(s, smr[i]);
    }
    compute_bit_allocation(s, smr, bit_alloc, &padding);

    init_put_bits(&s->pb, frame, MPA_MAX_CODED_FRAME_SIZE);

    encode_frame(s, bit_alloc, padding);
    
    return pbBufPtr(&s->pb) - s->pb.buf;
}
