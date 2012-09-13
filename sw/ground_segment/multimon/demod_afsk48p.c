/*
 *      demod_afsk48p.c -- 4800 baud AFSK demodulator for paparazzi
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
#include "pprz.h"
#include <math.h>
#include <string.h>
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

static void afsk48p_init(struct demod_state *s)
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

/* ---------------------------------------------------------------------- */

static void afsk48p_demod(struct demod_state *s, float *buffer, int length)
{
	float f;
	unsigned char curbit;

	s->l1.afsk48p.sample_count += length;
	if (s->l1.afsk48p.sample_count > FREQ_SAMP) {
		s->l1.afsk48p.sample_count -= FREQ_SAMP;
		pprz_status(s);
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
			pprz_baudot_rxbit(s, curbit);
		}
	}
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_afsk4800p = {
	"AFSK4800_P", FREQ_SAMP, CORRLEN, afsk48p_init, afsk48p_demod
};

/* ---------------------------------------------------------------------- */
