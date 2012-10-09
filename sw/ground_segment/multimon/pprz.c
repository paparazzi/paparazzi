/*
 *      pprz.c -- paparazzi decoder
 *
 *      Copyright (C) 1996
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *      Copyright (C) 2005
 *          Martin Mueller (ma.mueller@tu-bs.de)
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

#include "multimon.h"
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define STX         0x02
#define ETX         0x03

#define MSG_DATA    0
#define MSG_ERROR   1
#define MSG_CD      2
#define MSG_DEBUG   3
#define MSG_VALIM   4

#define INPUT_BUF_LEN 10

#define MULTIMON_PIPE_NAME "/tmp/multimon"


/* ---------------------------------------------------------------------- */

static void pprz_tmtc_send(struct demod_state *s, unsigned char *data, unsigned int len, unsigned char type)
{
	unsigned char msg[INPUT_BUF_LEN+6];
	unsigned int count = 0;
	int ret;

	unsigned char checksum = 0;
	const unsigned char real_len = len + 2;
	unsigned char i;

	msg[count++] = STX;
	msg[count++] = real_len;
	checksum ^= real_len;
	msg[count++] = type;
	checksum ^= type;
	for (i=0; i < len; i++) {
		msg[count++] = data[i];
		checksum ^= data[i];
	}
	msg[count++] = checksum;
	msg[count++] = ETX;

	ret = write(s->l2.pprz.pipe_fd, msg, count);

	if (count != ret)
		perror("write pipe");
}

/* ---------------------------------------------------------------------- */

char multimon_pipe_name[1024] = MULTIMON_PIPE_NAME;

void pprz_init(struct demod_state *s)
{
	memset(&s->l2.hdlc, 0, sizeof(s->l2.hdlc));

	/* open named pipe */
	struct stat st;
	if (stat(multimon_pipe_name, &st)) {
	  if (mkfifo(multimon_pipe_name, 0644) == -1) {
	    perror("make pipe");
	    exit (10);
	  }
	}
	if ((s->l2.pprz.pipe_fd = open(multimon_pipe_name, O_WRONLY /*| O_NONBLOCK*/)) < 0) {
		perror("open pipe");
		exit (10);
	}

	/* reset buffer pointer */
	s->l2.pprz.rxptr = s->l2.hdlc.rxbuf;
}

/* ---------------------------------------------------------------------- */

void pprz_baudot_rxbit(struct demod_state *s, int bit)
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

	if ((s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf) >= INPUT_BUF_LEN) {
		pprz_tmtc_send(s, s->l2.hdlc.rxbuf, s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf, MSG_DATA);
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

/* ---------------------------------------------------------------------- */

void pprz_hdlc_rxbit(struct demod_state *s, int bit)
{
	s->l2.hdlc.rxbitstream <<= 1;
	s->l2.hdlc.rxbitstream |= !!bit;
	if ((s->l2.hdlc.rxbitstream & 0xff) == 0x7e) {
		if (s->l2.hdlc.rxstate && (s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf) > 2)
			pprz_tmtc_send(s, s->l2.hdlc.rxbuf, s->l2.hdlc.rxptr - s->l2.hdlc.rxbuf, MSG_DATA);
		s->l2.hdlc.rxstate = 1;
		s->l2.hdlc.rxptr = s->l2.hdlc.rxbuf;
		s->l2.hdlc.rxbitbuf = 0x80;
		return;
	}
	if ((s->l2.hdlc.rxbitstream & 0x7f) == 0x7f) {
		s->l2.hdlc.rxstate = 0;
		return;
	}
	if (!s->l2.hdlc.rxstate)
		return;
	if ((s->l2.hdlc.rxbitstream & 0x3f) == 0x3e) /* stuffed bit */
		return;
	if (s->l2.hdlc.rxbitstream & 1)
		s->l2.hdlc.rxbitbuf |= 0x100;
	if (s->l2.hdlc.rxbitbuf & 1) {
		if (s->l2.hdlc.rxptr >= s->l2.hdlc.rxbuf+sizeof(s->l2.hdlc.rxbuf)) {
			s->l2.hdlc.rxstate = 0;
			verbprintf(1, "Error: packet size too large\n");
			return;
		}
		*s->l2.hdlc.rxptr++ = s->l2.hdlc.rxbitbuf >> 1;
		s->l2.hdlc.rxbitbuf = 0x80;
		return;
	}
      	s->l2.hdlc.rxbitbuf >>= 1;
}

/* ---------------------------------------------------------------------- */

void pprz_status(struct demod_state *s)
{
	unsigned char data[2];

	/* TODO find carrier */
	data[0] = 0x01; pprz_tmtc_send(s, data, 1, MSG_CD);
	/* just write some useful value (get it from acpi?) */
	data[0] = 0x00; data[1] = 0x04; pprz_tmtc_send(s, data, 2, MSG_VALIM);
	/* no debug value */
	data[0] = 0x00; pprz_tmtc_send(s, data, 1, MSG_DEBUG);

	return;
}

/* ---------------------------------------------------------------------- */
