/*
 *      demod_poc12.c -- 1200 baud POCSAG demodulator
 *
 *      Copyright (C) 1996  
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *
 *      POCSAG (Post Office Code Standard Advisory Group)
 *      Radio Paging Decoder
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "multimon.h"
#include "filter.h"
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------- */

#define FREQ_SAMP  22050
#define BAUD       1200
#define SUBSAMP    2
#define FILTLEN    1

/* ---------------------------------------------------------------------- */

#define SPHASEINC (0x10000u*BAUD*SUBSAMP/FREQ_SAMP)

/* ---------------------------------------------------------------------- */
	
static void poc12_init(struct demod_state *s)
{
	pocsag_init(s);
	memset(&s->l1.poc12, 0, sizeof(s->l1.poc12));
}

/* ---------------------------------------------------------------------- */

static void poc12_demod(struct demod_state *s, float *buffer, int length)
{
	if (s->l1.poc12.subsamp) {
		int numfill = SUBSAMP - s->l1.poc12.subsamp;
		if (length < numfill) {
			s->l1.poc12.subsamp += length;
			return;
		}
		buffer += numfill;
		length -= numfill;
		s->l1.poc12.subsamp = 0;
	}
	for (; length >= SUBSAMP; length -= SUBSAMP, buffer += SUBSAMP) {
		s->l1.poc12.dcd_shreg <<= 1;
		s->l1.poc12.dcd_shreg |= ((*buffer) > 0);
		verbprintf(10, "%c", '0'+(s->l1.poc12.dcd_shreg & 1));
		/*
		 * check if transition
		 */
		if ((s->l1.poc12.dcd_shreg ^ (s->l1.poc12.dcd_shreg >> 1)) & 1) {
			if (s->l1.poc12.sphase < (0x8000u-(SPHASEINC/2)))
				s->l1.poc12.sphase += SPHASEINC/8;
			else
				s->l1.poc12.sphase -= SPHASEINC/8;
		}
		s->l1.poc12.sphase += SPHASEINC;
		if (s->l1.poc12.sphase >= 0x10000u) {
			s->l1.poc12.sphase &= 0xffffu;
			pocsag_rxbit(s, s->l1.poc12.dcd_shreg & 1);
		}
	}
	s->l1.poc12.subsamp = length;
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_poc12 = {
	"POCSAG1200", FREQ_SAMP, FILTLEN, poc12_init, poc12_demod
};

/* ---------------------------------------------------------------------- */
