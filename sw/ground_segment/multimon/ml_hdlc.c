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
#include <sys/soundcard.h>
#include <sys/ioctl.h>


#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <caml/signals.h>

#include "gen.h"

static struct gen_params params;
static struct gen_state state;
int fd;
int fmt = 0;


/** Intermediate buffer */
union {
  short s[BUF_LEN];
  unsigned char b[BUF_LEN];
} b;


value ml_init_gen_hdlc(value device) {
  /** From multimon, gen.c:output_sound() */

  char *ifname=String_val(device);
  int sample_rate = SAMPLE_RATE;

  int sndparam;

  if ((fd = open(ifname ? ifname : "/dev/dsp", O_WRONLY)) < 0) {
    perror("open");
    exit (10);
  }
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
  params.p.hdlc.txdelay = 40; /* 1e-2 s */
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

  return Val_int(fd);
}

static int process_buffer(short *buf, int len)
{
  memset(buf, 0, len*sizeof(buf[0]));
  return gen_hdlc(buf, len, &params, &state);
}


/** intermediate buffer 1s large */
#define BUF_LEN 16384

value ml_gen_hdlc(value val_data) {
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
  
  memset(&state, 0, sizeof(state));
  gen_init_hdlc(&params, &state);

  int num2;
  do {
    int num;
    num2 = num = process_buffer(b.s, sizeof(b.s)/sizeof(b.s[0]));
    if (fmt) {
      unsigned char *bp;
      short *sp;
	
      for (bp = b.b, sp = b.s, i = sizeof(b.s)/sizeof(b.s[0]); i > 0; i--, bp++, sp++)
	*bp = 0x80 + (*sp >> 8);
      bp = b.b;
      while (num > 0) {
	i = write(fd, bp, num*sizeof(bp[0]));
	if (i < 0 && errno != EAGAIN) {
	  perror("write");
	  exit(4);
	}
	if (i > 0) {
	  if (i % sizeof(bp[0]))
	    fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
	  num -= i / sizeof(bp[0]);
	}
      }
    } else {
      short *sp = b.s;
      while (num > 0) {
	//	enter_blocking_section();
	i = write(fd, sp, num*sizeof(sp[0]));
	printf("num=%d i=%d\n", num, i);
	//	leave_blocking_section();
	if (i < 0 && errno != EAGAIN) {
	  perror("write");
	  exit(4);
	}
	if (i > 0) {
	  if (i % sizeof(sp[0]))
	    fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
	  num -= i / sizeof(sp[0]);
	}
      }
    }
  } while (num2 > 0);
 
  return Val_unit;
}
