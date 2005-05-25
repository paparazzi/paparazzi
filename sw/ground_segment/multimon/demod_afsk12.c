/*
 *      demod_afsk12.c -- 1200 baud AFSK demodulator
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

/*
 * Standard TCM3105 clock frequency: 4.4336MHz
 * Mark frequency: 2200 Hz
 * Space frequency: 1200 Hz
 */

#define FREQ_MARK  1200
#define FREQ_SPACE 2200
#define FREQ_SAMP  22050
#define BAUD       1200
#define SUBSAMP    2

/* ---------------------------------------------------------------------- */

#define CORRLEN ((int)(FREQ_SAMP/BAUD))
#define SPHASEINC (0x10000u*BAUD*SUBSAMP/FREQ_SAMP)

static float corr_mark_i[CORRLEN];
static float corr_mark_q[CORRLEN];
static float corr_space_i[CORRLEN];
static float corr_space_q[CORRLEN];

/* ---------------------------------------------------------------------- */
	
static void afsk12_init(struct demod_state *s)
{
	float f;
	int i;

	hdlc_init(s);
	memset(&s->l1.afsk12, 0, sizeof(s->l1.afsk12));
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
}

/* ---------------------------------------------------------------------- */

static void afsk12_demod(struct demod_state *s, float *buffer, int length)
{
	float f;
	unsigned char curbit;

	if (s->l1.afsk12.subsamp) {
		int numfill = SUBSAMP - s->l1.afsk12.subsamp;
		if (length < numfill) {
			s->l1.afsk12.subsamp += length;
			return;
		}
		buffer += numfill;
		length -= numfill;
		s->l1.afsk12.subsamp = 0;
	}
	for (; length >= SUBSAMP; length -= SUBSAMP, buffer += SUBSAMP) {
		f = fsqr(mac(buffer, corr_mark_i, CORRLEN)) +
			fsqr(mac(buffer, corr_mark_q, CORRLEN)) -
			fsqr(mac(buffer, corr_space_i, CORRLEN)) -
			fsqr(mac(buffer, corr_space_q, CORRLEN));
		s->l1.afsk12.dcd_shreg <<= 1;
		s->l1.afsk12.dcd_shreg |= (f > 0);
		verbprintf(10, "%c", '0'+(s->l1.afsk12.dcd_shreg & 1));
		/*
		 * check if transition
		 */
		if ((s->l1.afsk12.dcd_shreg ^ (s->l1.afsk12.dcd_shreg >> 1)) & 1) {
			if (s->l1.afsk12.sphase < (0x8000u-(SPHASEINC/2)))
				s->l1.afsk12.sphase += SPHASEINC/8;
			else
				s->l1.afsk12.sphase -= SPHASEINC/8;
		}
		s->l1.afsk12.sphase += SPHASEINC;
		if (s->l1.afsk12.sphase >= 0x10000u) {
			s->l1.afsk12.sphase &= 0xffffu;
			s->l1.afsk12.lasts <<= 1;
			s->l1.afsk12.lasts |= s->l1.afsk12.dcd_shreg & 1;
			curbit = (s->l1.afsk12.lasts ^ 
				  (s->l1.afsk12.lasts >> 1) ^ 1) & 1;
			verbprintf(9, " %c ", '0'+curbit);
			hdlc_rxbit(s, curbit);
		}
	}
	s->l1.afsk12.subsamp = length;
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_afsk1200 = {
	"AFSK1200", FREQ_SAMP, CORRLEN, afsk12_init, afsk12_demod
};

/* ---------------------------------------------------------------------- */
