/*
 *      gen_hdlc.c -- generate DTMF sequences
 *
 *      Copyright (C) 1997
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "gen.h"
#include <string.h>

#define COS(x) costabi[(((x)>>6)&0x3ffu)]

/* ---------------------------------------------------------------------- */
/*
 * the CRC routines are stolen from WAMPES
 * by Dieter Deyke
 */

static const unsigned short crc_ccitt_table[] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*---------------------------------------------------------------------------*/

#if 0
static inline void append_crc_ccitt(unsigned char *buffer, int len)
{
  unsigned int crc = 0xffff;

  for (;len>0;len--)
    crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ *buffer++) & 0xff];
  crc ^= 0xffff;
  *buffer++ = crc;
  *buffer++ = crc >> 8;
}

/*---------------------------------------------------------------------------*/

static inline int check_crc_ccitt(const unsigned char *buf, int cnt)
{
  unsigned int crc = 0xffff;

  for (; cnt > 0; cnt--)
    crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ *buf++) & 0xff];
  return (crc & 0xffff) == 0xf0b8;
}
#endif

/*---------------------------------------------------------------------------*/

static __inline__ int calc_crc_ccitt(const unsigned char *buf, int cnt)
{
  unsigned int crc = 0xffff;

  for (; cnt > 0; cnt--)
    crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ *buf++) & 0xff];
  crc ^= 0xffff;
  return (crc & 0xffff);
}

/* ---------------------------------------------------------------------- */

struct hdlctx {
  unsigned int bitstream;
  unsigned int bitbuf;
  int numbits;
};

static void txb_addbyte(struct gen_state *s, struct hdlctx *hdlctx,
			unsigned char bits, unsigned char stuff)
{
  unsigned int mask1, mask2;
  unsigned int mask3;
  int i;

  if (hdlctx->numbits >= 8) {
    if (s->s.hdlc.datalen >= sizeof(s->s.hdlc.data))
      return;
    s->s.hdlc.data[s->s.hdlc.datalen++] = hdlctx->bitbuf;
    hdlctx->bitbuf >>= 8;
    hdlctx->numbits -= 8;
  }
  hdlctx->bitbuf |= bits << hdlctx->numbits;
  hdlctx->bitstream >>= 8;
  hdlctx->bitstream |= bits << 8;
  mask1 = 0x1f0;
  mask2 = 0x100;
  mask3 = 0xffffffff >> (31 - hdlctx->numbits);
  hdlctx->numbits += 8;
  if (!stuff)
    goto nostuff;
  for(i = 0; i < 8; i++, mask1 <<= 1, mask2 <<= 1, mask3 = (mask3 << 1) | 1) {
    if ((hdlctx->bitstream & mask1) != mask1)
      continue;
    hdlctx->bitstream &= ~mask2;
    hdlctx->bitbuf = (hdlctx->bitbuf & mask3) | ((hdlctx->bitbuf & (~mask3)) << 1);
    hdlctx->numbits++;
    mask3 = (mask3 << 1) | 1;
  }
 nostuff:
  if (hdlctx->numbits >= 8) {
    if (s->s.hdlc.datalen >= sizeof(s->s.hdlc.data))
      return;
    s->s.hdlc.data[s->s.hdlc.datalen++] = hdlctx->bitbuf;
    hdlctx->bitbuf >>= 8;
    hdlctx->numbits -= 8;
  }
}

/* ---------------------------------------------------------------------- */

void gen_init_hdlc(struct gen_params *p, struct gen_state *s)
{
  struct hdlctx hdlctx = { 0, 0, 0 };
  int i;

  memset(s, 0, sizeof(struct gen_state));

  s->s.hdlc.bitmask = 1;
  for (i = 0; i < (p->p.hdlc.txdelay * (1200/100) / 8); i++)
    txb_addbyte(s, &hdlctx, 0x7e, 0);

  txb_addbyte(s, &hdlctx, 0x7e, 0);

  for (i = 0; i < p->p.hdlc.pktlen; i++)
    txb_addbyte(s, &hdlctx, p->p.hdlc.pkt[i], 1);

  i = calc_crc_ccitt(p->p.hdlc.pkt, p->p.hdlc.pktlen);

  txb_addbyte(s, &hdlctx, i, 1);
  txb_addbyte(s, &hdlctx, i >> 8, 1);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
  txb_addbyte(s, &hdlctx, 0x7e, 0);
}


int hdlc_sample_rate = DEFAULT_SAMPLE_RATE;

int gen_hdlc(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
  int num = 0;

  if (!s || s->s.hdlc.ch_idx < 0 || s->s.hdlc.ch_idx >= s->s.hdlc.datalen)
    return 0;
  for (; buflen > 0; buflen--, buf++, num++) {
    s->s.hdlc.bitph += 0x10000*1200 / hdlc_sample_rate;
    if (s->s.hdlc.bitph >= 0x10000u) {
      s->s.hdlc.bitph &= 0xffffu;
      s->s.hdlc.bitmask <<= 1;
      if (s->s.hdlc.bitmask >= 0x100) {
	s->s.hdlc.bitmask = 1;
	s->s.hdlc.ch_idx++;
	if (s->s.hdlc.ch_idx >= s->s.hdlc.datalen)
	  return num;
      }
      if (!(s->s.hdlc.data[s->s.hdlc.ch_idx] & s->s.hdlc.bitmask))
	s->s.hdlc.lastb = !s->s.hdlc.lastb;
      s->s.hdlc.phinc = (s->s.hdlc.lastb) ?
	0x10000*2200/hdlc_sample_rate : 0x10000*1200/hdlc_sample_rate;
    }
    *buf += (p->ampl * COS(s->s.hdlc.ph)) >> 15;
    s->s.hdlc.ph += s->s.hdlc.phinc;
  }
  return num;
}
