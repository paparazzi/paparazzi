/*
 *      ml_hdlc.c -- OCaml binding to multimon HDLC library
 *
 *      Copyright (C) 1997
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *      Copyright (C) 2007, Pascal Brisset for Paparazzi
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#ifndef __APPLE__
#include <sys/soundcard.h>
#include <sys/ioctl.h>
#endif

#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <caml/signals.h>

#include "gen.h"
#include "multimon.h"

#define Min(a, b) ((a) < (b) ? (a) : (b))

static struct gen_params params;
static struct gen_state state;
int fd;
int fmt = 0;

#define MY_BUF_LEN 32768
short my_buffer[MY_BUF_LEN];
int idx_start, idx_end;

/** intermediate buffer  */
#define BUF_LEN 16384
short silence_buffer[BUF_LEN];


#define OUTPUT_BUF_LEN 1024


static struct demod_state dem_st;


static int process_buffer(short *buf, int len)
{
  memset(buf, 0, len*sizeof(buf[0]));
  return gen_hdlc(buf, len, &params, &state);
}

value ml_init_gen_hdlc(value device) {
  /** From multimon, gen.c:output_sound() */

#ifndef __APPLE__
  char *ifname=String_val(device);
  int sample_rate = DEFAULT_SAMPLE_RATE;

  int sndparam;

  if ((fd = open(ifname ? ifname : "/dev/dsp", O_WRONLY)) < 0) {
    caml_failwith("open");
    exit (10);
  }

  /** Doesn't work, stay with 8192
  int frag = (0x7fff << 16) | (14); // 14=16384, http://manuals.opensound.com/developer/SNDCTL_DSP_SETFRAGMENT.html
  ioctl(fd, SNDCTL_DSP_SETFRAGMENT, &frag);
  */

  sndparam = AFMT_S16_LE; /* we want 16 bits/sample signed */
  /* little endian; works only on little endian systems! */
  if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_SETFMT");
    exit (10);
  }
  if (sndparam != AFMT_S16_LE) {
    fmt = 1;
    sndparam = AFMT_U8;
    if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
      perror("ioctl: SNDCTL_DSP_SETFMT");
      exit (10);
    }
    if (sndparam != AFMT_U8) {
      perror("ioctl: SNDCTL_DSP_SETFMT");
      exit (10);
    }
  }
  sndparam = 0;   /* we want only 1 channel */
  if (ioctl(fd, SNDCTL_DSP_STEREO, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_STEREO");
    exit (10);
  }
  if (sndparam != 0) {
    fprintf(stderr, "gen: Error, cannot set the channel "
	    "number to 1\n");
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


  params.type = gentype_hdlc;
  params.ampl = 16384;
  params.p.hdlc.modulation = 0;
  params.p.hdlc.pkt[0] = ('H') << 1;
  params.p.hdlc.pkt[1] = ('B') << 1;
  params.p.hdlc.pkt[2] = ('9') << 1;
  params.p.hdlc.pkt[3] = ('J') << 1;
  params.p.hdlc.pkt[4] = ('N') << 1;
  params.p.hdlc.pkt[5] = ('X') << 1;
  params.p.hdlc.pkt[6] = (0x00) << 1;
  params.p.hdlc.pkt[7] = ('A') << 1;
  params.p.hdlc.pkt[8] = ('E') << 1;
  params.p.hdlc.pkt[9] = ('4') << 1;
  params.p.hdlc.pkt[10] = ('W') << 1;
  params.p.hdlc.pkt[11] = ('A') << 1;
  params.p.hdlc.pkt[12] = (' ') << 1;
  params.p.hdlc.pkt[13] = ((0x00) << 1) | 1;
  params.p.hdlc.pkt[14] = 0x03;
  params.p.hdlc.pkt[15] = 0xf0;

  /** Generate a buffer full of silence **/
  params.p.hdlc.txdelay = 100; /* 1 s */
  params.p.hdlc.pktlen = 16;
  memset(&state, 0, sizeof(state));
  gen_init_hdlc(&params, &state);
  process_buffer(silence_buffer, sizeof(silence_buffer)/sizeof(silence_buffer[0]));

  idx_start = idx_end = 0;

  return Val_int(fd);
#else
  failwith("Not supported under OSX");
#endif

}


value ml_init_dec_hdlc(value dev) {
  char *ifname = String_val(dev);
#ifndef __APPLE__
  if ((fd = open(ifname, O_RDONLY)) < 0) {
    caml_failwith("Hdlc.init_dec: open failed");
  }

  int sndparam = AFMT_S16_LE; /* we want 16 bits/sample signed */
  /* little endian; works only on little endian systems! */
  if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_SETFMT");
    exit (10);
  }
  if (sndparam != AFMT_S16_LE) {
    perror("ioctl: AFMT_S16_LE");
    exit (10);
  }

  sndparam = DEFAULT_SAMPLE_RATE;
  if (ioctl(fd, SNDCTL_DSP_SPEED, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_SPEED");
    exit (10);
  }
  if ((10*abs(sndparam-DEFAULT_SAMPLE_RATE)) > DEFAULT_SAMPLE_RATE) {
    perror("ioctl: SNDCTL_DSP_SPEED");
    exit (10);
  }
  if (sndparam != DEFAULT_SAMPLE_RATE) {
    fprintf(stderr, "Warning: Sampling rate is %u, "
	    "requested %u\n", sndparam, DEFAULT_SAMPLE_RATE);
  }

  memset(&dem_st, 0, sizeof(struct demod_state));
  afsk12_init(&dem_st);
  dem_st.dem_par = &demod_afsk1200;
  return Val_int(fd);
#else
  failwith("Not supported under OSX");
#endif

}


value ml_gen_hdlc(value val_data) {
#ifndef __APPLE__
  /** Handling the data input */
  int data_len = string_length(val_data);
  if (data_len + 16 > sizeof(params.p.hdlc.pkt))
    caml_invalid_argument("Hdlc.write_data: argument too large");

  char *data = String_val(val_data);
  int i;
  for(i = 0; i < data_len; i++) {
    params.p.hdlc.pkt[16+i] = data[i];
  }
  params.p.hdlc.pktlen = 16 + data_len;
  params.p.hdlc.txdelay = 10;

  memset(&state, 0, sizeof(state));
  gen_init_hdlc(&params, &state);

  /** Intermediate buffer */
  short s[BUF_LEN];

  int n = process_buffer(s, sizeof(s)/sizeof(s[0]));
  assert(state.s.hdlc.ch_idx == state.s.hdlc.datalen);

  int occupied = (idx_end + MY_BUF_LEN - idx_start) % MY_BUF_LEN;
  if (occupied + n > MY_BUF_LEN) {
    caml_failwith("buffer full");
  } else {
    int i;
    for(i = 0; i < n; i++)
      my_buffer[(idx_end+i) % MY_BUF_LEN] = s[i];
    idx_end = (idx_end + n) % MY_BUF_LEN;
  }

  return Val_unit;
#else
  failwith("Not supported under OSX");
#endif

}


/* max ospace = 8192 = 4096 samples = 4096/22050 seconds = 0.18 */
value ml_write_to_dsp(value _) {
#ifndef __APPLE__
  audio_buf_info bi;
  ioctl(fd, SNDCTL_DSP_GETOSPACE, &bi);
  int ospace = bi.bytes;

  int n = ospace/sizeof(short);

  int available = (idx_end + MY_BUF_LEN - idx_start) % MY_BUF_LEN;
  /* complete with silence */
  int i;
  for(i = available; i < n; i++) {
    my_buffer[(idx_start+i)%MY_BUF_LEN] = silence_buffer[i];
  }

  int k =  Min(n, MY_BUF_LEN - idx_start);
  int num = write(fd, my_buffer+idx_start, k*sizeof(short));
  num += write(fd, my_buffer+0, (n - k)*sizeof(short));
  if (available <= n) {
    /* buffer is empty */
    idx_start = idx_end = 0;
  } else {
    idx_start = (idx_start + n) % MY_BUF_LEN;
  }

  assert(num == n*sizeof(short));

  return Val_unit
#else
  failwith("Not supported under OSX");
#endif
;
}

static union {
  short s[8192];
  unsigned char b[8192];
} b;
static float fbuf[16384];
static unsigned int fbuf_cnt = 0;


value ml_get_hdlc(value unit) {
#ifndef __APPLE__
  unsigned int overlap = demod_afsk1200.overlap;
  short *sp;
  int i = read(fd, sp = b.s, sizeof(b.s));
  if(i < 0)
    caml_failwith("Hdlc.get_data: read failed");

  CAMLparam0();
  CAMLlocal1 (result);

  hdlc_data_received_idx = 0;
  if(i > 0) {
    for (; i >= sizeof(b.s[0]); i -= sizeof(b.s[0]), sp++)
      fbuf[fbuf_cnt++] = (*sp) * (1.0/32768.0);
    if (i)
	fprintf(stderr, "warning: noninteger number of samples read\n");
    if (fbuf_cnt > overlap) {
      demod_afsk1200.demod(&dem_st, fbuf, fbuf_cnt-overlap);

      memmove(fbuf, fbuf+fbuf_cnt-overlap, overlap*sizeof(fbuf[0]));
      fbuf_cnt = overlap;
    }
  }

  /* Alloc the caml string and fill it */
  result = alloc_string(hdlc_data_received_idx);
  for(i = 0; i < hdlc_data_received_idx; i++)
    Byte(result, i) = hdlc_data_received[i];
  CAMLreturn(result);
#else
  failwith("Not supported under OSX");
#endif

}
