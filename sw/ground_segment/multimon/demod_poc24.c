/*
 *      demod_poc24.c -- 2400 baud POCSAG demodulator
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
#define BAUD       2400
#define FILTLEN    1

/* ---------------------------------------------------------------------- */

#define SPHASEINC (0x10000u*BAUD/FREQ_SAMP)

/* ---------------------------------------------------------------------- */
	
static void poc24_init(struct demod_state *s)
{
	pocsag_init(s);
	memset(&s->l1.poc24, 0, sizeof(s->l1.poc24));
}

/* ---------------------------------------------------------------------- */

static void poc24_demod(struct demod_state *s, float *buffer, int length)
{
	for (; length > 0; length--, buffer++) {
		s->l1.poc24.dcd_shreg <<= 1;
		s->l1.poc24.dcd_shreg |= ((*buffer) > 0);
		verbprintf(10, "%c", '0'+(s->l1.poc24.dcd_shreg & 1));
		/*
		 * check if transition
		 */
		if ((s->l1.poc24.dcd_shreg ^ (s->l1.poc24.dcd_shreg >> 1)) & 1) {
			if (s->l1.poc24.sphase < (0x8000u-(SPHASEINC/2)))
				s->l1.poc24.sphase += SPHASEINC/8;
			else
				s->l1.poc24.sphase -= SPHASEINC/8;
		}
		s->l1.poc24.sphase += SPHASEINC;
		if (s->l1.poc24.sphase >= 0x10000u) {
			s->l1.poc24.sphase &= 0xffffu;
			pocsag_rxbit(s, s->l1.poc24.dcd_shreg & 1);
		}
	}
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_poc24 = {
	"POCSAG2400", FREQ_SAMP, FILTLEN, poc24_init, poc24_demod
};

/* ---------------------------------------------------------------------- */
