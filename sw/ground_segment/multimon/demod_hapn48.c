/*
 *      demod_hapn48.c -- HAPN 4800 baud demodulator (G3RUH)
 *
 *      Copyright (C) 1996  
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

#include "multimon.h"
#include "filter.h"
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------- */

#define FREQ_SAMP  22050
#define BAUD       4800

/* ---------------------------------------------------------------------- */

#define SPHASEINC (0x10000u*BAUD/FREQ_SAMP)

/* ---------------------------------------------------------------------- */
	
static void hapn48_init(struct demod_state *s)
{
	hdlc_init(s);
	memset(&s->l1.hapn48, 0, sizeof(s->l1.hapn48));
}

/* ---------------------------------------------------------------------- */

static void hapn48_demod(struct demod_state *s, float *buffer, int length)
{
	int cursync;
	unsigned int curbit;

	for (; length > 0; length--, buffer++) {
		s->l1.hapn48.lvlhi *= 0.999;
		s->l1.hapn48.lvllo *= 0.999;
		if (buffer[1] > s->l1.hapn48.lvlhi)
			s->l1.hapn48.lvlhi = buffer[1];
		if (buffer[1] < s->l1.hapn48.lvllo)
			s->l1.hapn48.lvllo = buffer[1];
		cursync = 0;
		s->l1.hapn48.shreg = (s->l1.hapn48.shreg << 1) | 
			(s->l1.hapn48.shreg & 1);
		if (buffer[1] > s->l1.hapn48.lvlhi * 0.5) {
			s->l1.hapn48.shreg |= 1;
			cursync = (buffer[1] > buffer[0] && buffer[1] > buffer[2]);
		} else if (buffer[1] < s->l1.hapn48.lvllo * 0.5) {
			s->l1.hapn48.shreg &= ~1;
			cursync = (buffer[1] < buffer[0] && buffer[1] < buffer[2]);
		}
		verbprintf(10, "%c", '0' + (s->l1.hapn48.shreg & 1));
		s->l1.hapn48.sphase += SPHASEINC;
		if (((s->l1.hapn48.shreg >> 1) ^ s->l1.hapn48.shreg) & 1)
			if (s->l1.hapn48.sphase >= 0x8000+SPHASEINC/2)
				s->l1.hapn48.sphase -= 0x800;
			else
				s->l1.hapn48.sphase += 0x800;
#if 0
		if (cursync)
			if (((s->l1.hapn48.sphase-0x8000)&0xffffu) >= 0x8000+SPHASEINC/2)
				s->l1.hapn48.sphase -= 0x800;
			else
				s->l1.hapn48.sphase += 0x800;
#endif
		if (s->l1.hapn48.sphase >= 0x10000) {
			s->l1.hapn48.sphase &= 0xffff;
			curbit = ((s->l1.hapn48.shreg >> 4) ^ s->l1.hapn48.shreg ^ 1) & 1;
			verbprintf(9, " %c ", '0'+curbit);
			hdlc_rxbit(s, curbit);
		}
	}
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_hapn4800 = {
	"HAPN4800", FREQ_SAMP, 3, hapn48_init, hapn48_demod
};

/* ---------------------------------------------------------------------- */
