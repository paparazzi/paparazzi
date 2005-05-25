/*
 *      gen_sine.c -- generate DTMF sequences
 *
 *      Copyright (C) 1997
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

#include "gen.h"
#include <string.h>

/* ---------------------------------------------------------------------- */

void gen_init_sine(struct gen_params *p, struct gen_state *s)
{
	memset(s, 0, sizeof(struct gen_state));
	s->s.sine.ph = 0;
	s->s.sine.phinc = (float)0x10000 * p->p.sine.freq / SAMPLE_RATE;
	s->s.sine.time = p->p.sine.duration;
}

int gen_sine(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
	int num = 0;

	for (; (buflen > 0) && (s->s.sine.time > 0); buflen--, buf++, num++, s->s.sine.time--) {
		*buf += (p->ampl * COS(s->s.sine.ph)) >> 15;
		s->s.sine.ph += s->s.sine.phinc;
	}
	return num;
}
