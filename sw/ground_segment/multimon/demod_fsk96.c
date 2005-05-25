/*
 *      demod_fsk96.c -- FSK 9600 baud demodulator (G3RUH)
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
#define BAUD       9600

/* ---------------------------------------------------------------------- */

#define DESCRAM_TAP1 0x20000
#define DESCRAM_TAP2 0x01000
#define DESCRAM_TAP3 0x00001

#define DESCRAM_TAPSH1 17
#define DESCRAM_TAPSH2 12
#define DESCRAM_TAPSH3 0

#define SCRAM_TAP1 0x20000 /* X^17 */
#define SCRAM_TAPN 0x00021 /* X^0+X^5 */

/* --------------------------------------------------------------------- */

#define FILTLEN 24
#define UPSAMP 3

static const float inp_filt[3][24] = {
        {  0.000440, -0.001198, -0.000493,  0.003648,
          -0.000630, -0.008433,  0.005567,  0.015557,
          -0.019931, -0.026514,  0.079822,  0.181779,
           0.124956, -0.002471, -0.032062,  0.008024,
           0.012568, -0.006559, -0.004235,  0.003711,
           0.000909, -0.001520, -0.000018,  0.000709},
        {  0.000686, -0.000618, -0.001332,  0.002494,
           0.002258, -0.007308, -0.001538,  0.016708,
          -0.004897, -0.035748,  0.034724,  0.161417,
           0.161417,  0.034724, -0.035748, -0.004897,
           0.016708, -0.001538, -0.007308,  0.002258,
           0.002494, -0.001332, -0.000618,  0.000686},
        {  0.000709, -0.000018, -0.001520,  0.000909,
           0.003711, -0.004235, -0.006559,  0.012568,
           0.008024, -0.032062, -0.002471,  0.124956,
           0.181779,  0.079822, -0.026514, -0.019931,
           0.015557,  0.005567, -0.008433, -0.000630,
           0.003648, -0.000493, -0.001198,  0.000440}
};

#define SPHASEINC (0x10000u*BAUD/FREQ_SAMP/UPSAMP)

/* ---------------------------------------------------------------------- */
	
static void fsk96_init(struct demod_state *s)
{
	hdlc_init(s);
	memset(&s->l1.fsk96, 0, sizeof(s->l1.fsk96));
}

/* ---------------------------------------------------------------------- */

static void fsk96_demod(struct demod_state *s, float *buffer, int length)
{
	float f;
	unsigned char curbit;
	int i;
	unsigned int descx;

	for (; length > 0; length--, buffer++) {
		for (i = 0; i < UPSAMP; i++) {
			f = mac(buffer, inp_filt[i], FILTLEN);
			s->l1.fsk96.dcd_shreg <<= 1;
			s->l1.fsk96.dcd_shreg |= (f > 0);
			verbprintf(10, "%c", '0'+(s->l1.fsk96.dcd_shreg & 1));
			/*
			 * check if transition
			 */
			if ((s->l1.fsk96.dcd_shreg ^ (s->l1.fsk96.dcd_shreg >> 1)) & 1) {
				if (s->l1.fsk96.sphase < (0x8000u-(SPHASEINC/2)))
					s->l1.fsk96.sphase += SPHASEINC/8;
				else
					s->l1.fsk96.sphase -= SPHASEINC/8;
			}
			s->l1.fsk96.sphase += SPHASEINC;
			if (s->l1.fsk96.sphase >= 0x10000u) {
				s->l1.fsk96.sphase &= 0xffffu;
				s->l1.fsk96.descram <<= 1;
				s->l1.fsk96.descram |= s->l1.fsk96.dcd_shreg & 1;
				descx = s->l1.fsk96.descram ^ (s->l1.fsk96.descram >> 1);
				curbit = ((descx >> DESCRAM_TAPSH1) ^ (descx >> DESCRAM_TAPSH2) ^
					  (descx >> DESCRAM_TAPSH3) ^ 1) & 1;
				verbprintf(9, " %c ", '0'+curbit);
				hdlc_rxbit(s, curbit);
			}
		}
	}
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_fsk9600 = {
	"FSK9600", FREQ_SAMP, FILTLEN, fsk96_init, fsk96_demod
};

/* ---------------------------------------------------------------------- */
