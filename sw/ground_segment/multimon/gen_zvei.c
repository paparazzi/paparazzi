/*
 *      gen_zvei.c -- generate DTMF sequences
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
#include <ctype.h>
#include <stdio.h>

/* ---------------------------------------------------------------------- */

#define PHINC(x)   ((float)(x)*0x10000/SAMPLE_RATE)

static const unsigned int zvei_freq[16] = {
	PHINC(2400), PHINC(1060), PHINC(1160), PHINC(1270), 
	PHINC(1400), PHINC(1530), PHINC(1670), PHINC(1830), 
	PHINC(2000), PHINC(2200), PHINC(2800), PHINC(810), 
	PHINC(970), PHINC(886), PHINC(2600), PHINC(0)
};

static const unsigned int zveis_freq[16] = {
	PHINC(2400), PHINC(1060), PHINC(1160), PHINC(1270), 
	PHINC(1400), PHINC(1530), PHINC(1670), PHINC(1830), 
	PHINC(2000), PHINC(2200), PHINC(886), PHINC(810), 
	PHINC(740), PHINC(680), PHINC(970), PHINC(0)
};

void gen_init_zvei(struct gen_params *p, struct gen_state *s)
{
	memset(s, 0, sizeof(struct gen_state));
}

int gen_zvei(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
	char c;
	int num = 0, i;

	for (; buflen > 0; buflen--, buf++, num++) {
		if (s->s.zvei.time <= 0) {
			c = p->p.zvei.str[s->s.zvei.ch_idx];
			if (!c)
				return num;
			s->s.zvei.ch_idx++;
			if (!isxdigit(c)) {
				s->s.zvei.time = s->s.zvei.time2 = 1;
				fprintf(stderr, "gen: zvei; invalid char '%c'\n", c);
			} else {
				s->s.zvei.time = p->p.zvei.duration + p->p.zvei.pause;
				s->s.zvei.time2 = p->p.zvei.duration;
				if (c >= '0' && c <= '9')
					i = c - '0';
				else if (c >= 'A' && c <= 'F')
					i = c - 'A' + 10;
				else
					i = c - 'a' + 10;
				s->s.zvei.phinc = zvei_freq[i & 0xf];
			}
		} else if (!s->s.zvei.time2) {
			s->s.zvei.phinc = 0;
			s->s.zvei.ph = 0xc000;
		}
		s->s.zvei.time--;
		s->s.zvei.time2--;
		*buf += (p->ampl * COS(s->s.zvei.ph)) >> 15;
		s->s.zvei.ph += s->s.zvei.phinc;
	}
	return num;
}
