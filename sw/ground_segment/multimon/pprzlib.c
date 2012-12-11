/*
 *      Copyright (C) 1996
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *      Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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


/* ---------------------------------------------------------------------- */


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/soundcard.h>
#include <sys/ioctl.h>


#include "multimon.h"
#include "filter.h"
#include "pprz.h"
#include "pprzlib.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define LEFT 0
#define RIGHT 1

/* ---------------------------------------------------------------------- */

/*
 * Standard CMX469A clock frequency: 4.032 Mhz
 * Xtal used: 4 MHz
 * Ratio: 0.992063
 * Mark frequency:  4761.905 Hz
 * Space frequency: 2380.952 Hz
 */

#define FREQ_MARK  4762
#define FREQ_SPACE 2381
#define FREQ_SAMP  22050
//#define FREQ_SAMP  44100
#define BAUD       4762

/* ---------------------------------------------------------------------- */

#define CORRLEN (2*(int)(FREQ_SAMP/BAUD))
#define SPHASEINC (0x10000u*BAUD/FREQ_SAMP)

static float corr_mark_i[CORRLEN];
static float corr_mark_q[CORRLEN];
static float corr_space_i[CORRLEN];
static float corr_space_q[CORRLEN];

/* ---------------------------------------------------------------------- */

#define TEMP_BUF_LEN 10
#define OUTPUT_BUF_LEN 512


static int verbose_level = 0;

void
verbprintf(int verb_level, const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  if (verb_level <= verbose_level)
    vfprintf(stdout, fmt, args);
  va_end(args);
}


static int glob_data_received_len = 0; /** Easier with a global var */
static void
pprz_tmtc_send(unsigned char *data, unsigned int len, char* data_received)
{
  unsigned char i;
  for (i=0; i < len && glob_data_received_len < OUTPUT_BUF_LEN; i++) {
    data_received[glob_data_received_len++] = data[i];
  }
}


void
my_pprz_baudot_rxbit(struct demod_state *s, int bit, char* data)
{
  /* (stop)bit found? */
  if (bit & 1) {
    /* check for sync and start bit */
    if ((s->l2.hdlc.rxbitbuf & 1) &&
	!(s->l2.hdlc.rxbitbuf & 2)) {
      /* found new byte */
      *s->l2.hdlc.rxptr++ = (s->l2.hdlc.rxbitbuf >> 2) & 0xFF;
      verbprintf(8, "0x%02X ", (s->l2.hdlc.rxbitbuf >> 2) & 0xFF);
      /* reset bit buffer */
      s->l2.hdlc.rxbitbuf = 0x400;
    } else {
      /* add data bit */
      s->l2.hdlc.rxbitbuf |= 0x400;
    }
  }

  if ((s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf) >= TEMP_BUF_LEN) {
    pprz_tmtc_send(s->l2.hdlc.rxbuf, s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf, data);
    /* reset data buffer */
    s->l2.hdlc.rxptr = s->l2.hdlc.rxbuf;
  }

  /* frame end / out of sync */
  if (s->l2.hdlc.rxbitbuf & 1)
    s->l2.hdlc.rxbitbuf |= 2;

	/* shift in new data */
  s->l2.hdlc.rxbitbuf >>= 1;

  return;
}

void
pprz_init(struct demod_state *s)
{
  memset(&s->l2.hdlc, 0, sizeof(s->l2.hdlc));

  /* reset buffer pointer */
  s->l2.pprz.rxptr = s->l2.hdlc.rxbuf;
}

static void
afsk48p_init(struct demod_state *s)
{
  float f;
  int i;

  pprz_init(s);
  memset(&s->l1.afsk48p, 0, sizeof(s->l1.afsk48p));
  for (f = 0, i = 0; i < CORRLEN; i++) {
    corr_mark_i[i] = cos(f);
    corr_mark_q[i] = sin(f);
    f += 2.0*M_PI*FREQ_MARK/FREQ_SAMP;
  }
  for (f = 0, i = 0; i < CORRLEN; i++) {
    corr_space_i[i] = cos(f);
    corr_space_q[i] = sin(f);
    f += 2.0*M_PI*FREQ_SPACE/FREQ_SAMP;
  }
  for (i = 0; i < CORRLEN; i++) {
    f = 0.54 - 0.46*cos(2*M_PI*i/(float)(CORRLEN-1));
    corr_mark_i[i] *= f;
    corr_mark_q[i] *= f;
    corr_space_i[i] *= f;
    corr_space_q[i] *= f;
  }

  s->l1.afsk48p.sample_count = 0;
}

static void
afsk48p_demod(struct demod_state *s, float *buffer, int length, char* data)
{
  float f;
  unsigned char curbit;

  s->l1.afsk48p.sample_count += length;
  if (s->l1.afsk48p.sample_count > FREQ_SAMP) {
    s->l1.afsk48p.sample_count -= FREQ_SAMP;
    // pprz_status(s);
  }

  for (; length > 0; length--, buffer++) {
    f = fsqr(mac(buffer, corr_mark_i, CORRLEN)) +
      fsqr(mac(buffer, corr_mark_q, CORRLEN)) -
      fsqr(mac(buffer, corr_space_i, CORRLEN)) -
      fsqr(mac(buffer, corr_space_q, CORRLEN));
    s->l1.afsk48p.dcd_shreg <<= 1;
    s->l1.afsk48p.dcd_shreg |= (f > 0);
    verbprintf(10, "%c", '0'+(s->l1.afsk48p.dcd_shreg & 1));
    /*
     * check if transition
     */
    if ((s->l1.afsk48p.dcd_shreg ^ (s->l1.afsk48p.dcd_shreg >> 1)) & 1) {
      if (s->l1.afsk48p.sphase < (0x8000u-(SPHASEINC/2)))
	s->l1.afsk48p.sphase += SPHASEINC/8;
      else
	s->l1.afsk48p.sphase -= SPHASEINC/8;
    }
    s->l1.afsk48p.sphase += SPHASEINC;
    if (s->l1.afsk48p.sphase >= 0x10000u) {
      s->l1.afsk48p.sphase &= 0xffffu;
      s->l1.afsk48p.lasts <<= 1;
      s->l1.afsk48p.lasts |= s->l1.afsk48p.dcd_shreg & 1;
      /* we use direct coded bits, not NRZI */
      curbit = (s->l1.afsk48p.lasts ^ 1) & 1;
      verbprintf(9, " %c ", '0'+curbit);
      my_pprz_baudot_rxbit(s, curbit, data);
    }
  }
}

int fd;
union {
  short s[8192];
  unsigned char b[8192];
} b;
int stereo = 1;
unsigned int sample_rate = FREQ_SAMP;

static int
input_init(const char *ifname) {
  int sndparam;

  if ((fd = open(ifname, O_RDONLY)) < 0) {
    perror("open");
    exit (10);
  }
  if (!strncmp("/dev", ifname, 4)) {
    sndparam = AFMT_S16_LE; /* we want 16 bits/sample signed */
    /* little endian; works only on little endian systems! */
    if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
      perror("ioctl: SNDCTL_DSP_SETFMT");
      exit (10);
    }
    if (sndparam != AFMT_S16_LE) {
      perror("ioctl: AFMT_S16_LE");
      exit (10);
    }
    stereo = TRUE;
    sndparam = 1;   /* we want 2 channels */
    if (ioctl(fd, SNDCTL_DSP_STEREO, &sndparam) == -1) {
      perror("ioctl: SNDCTL_DSP_STEREO");
      exit (10);
    }
    if (sndparam == 0) {
      fprintf(stderr, "soundif: Error, cannot set the channel "
	      "number to 2: using mono\n");
      stereo=FALSE;
    } else if (sndparam != 1) {
      fprintf(stderr, "soundif: Error, cannot set the channel "
	      "number to 2\n");
      exit (10);
    }
    sndparam = sample_rate;
    if (ioctl(fd, SNDCTL_DSP_SPEED, &sndparam) == -1) {
      perror("ioctl: SNDCTL_DSP_SPEED");
      exit (10);
    }
    if ((10*abs(sndparam-sample_rate)) > sample_rate) {
      perror("ioctl: SNDCTL_DSP_SPEED");
      exit (10);
    }
    if (sndparam != sample_rate) {
      fprintf(stderr, "Warning: Sampling rate is %u, "
	      "requested %u\n", sndparam, sample_rate);
    }
  }
  return fd;
}


static struct demod_state dem_st[2];

char data_left[OUTPUT_BUF_LEN+1];
char data_right[OUTPUT_BUF_LEN+1];
struct data data_received = {data_left, 0, data_right, 0};
unsigned int fbuf_cnt = 0;
float fbuf[2][16384];

struct data*
pprz_demod_read_data(void) {
  unsigned int overlap = CORRLEN;
  int i;
  short *sp;

  i = read(fd, sp = b.s, sizeof(b.s));
  if (i < 0 && errno != EAGAIN) {
    perror("read");
    exit(4);
  }
  if (i > 0) {
    data_received.len_left = 0;
    data_received.len_right = 0;
    if (stereo) {
      for (; i >= sizeof(b.s[0]); i -= (sizeof(b.s[0])*2), sp+=2) {
	fbuf[LEFT][fbuf_cnt] = (*sp) * (1.0/32768.0);
	fbuf[RIGHT][fbuf_cnt++] = (*(sp+1)) * (1.0/32768.0);
      }
    } else {
      for (; i >= sizeof(b.s[0]); i -= sizeof(b.s[0]), sp++)
	fbuf[LEFT][fbuf_cnt++] = (*sp) * (1.0/32768.0);
      }
    if (i)
	fprintf(stderr, "warning: noninteger number of samples read\n");
    if (fbuf_cnt > overlap) {

      glob_data_received_len = 0;
      afsk48p_demod(&dem_st[LEFT], fbuf[LEFT], fbuf_cnt-overlap, data_received.data_left);

      data_received.len_left = glob_data_received_len;
      memmove(fbuf[LEFT], fbuf[LEFT]+fbuf_cnt-overlap, overlap*sizeof(fbuf[LEFT][0]));
      if (stereo) {
	glob_data_received_len = 0;
	afsk48p_demod(&dem_st[RIGHT], fbuf[RIGHT], fbuf_cnt-overlap, data_received.data_right);
	data_received.len_right = glob_data_received_len;
	memmove(fbuf[RIGHT], fbuf[RIGHT]+fbuf_cnt-overlap, overlap*sizeof(fbuf[RIGHT][0]));
      }
      fbuf_cnt = overlap;
    }
    return &data_received;
  } else
    return NULL;
}


int pprz_demod_init(char *dev) {
  memset(dem_st, 0, 2*sizeof(struct demod_state));
  afsk48p_init(&dem_st[LEFT]);
  afsk48p_init(&dem_st[RIGHT]);
  return input_init(dev);
}
