/*
 *      gen_dtmf.c -- generate DTMF sequences
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

/*
 *
 * DTMF frequencies
 *
 *      1209 1336 1477 1633
 *  697   1    2    3    A
 *  770   4    5    6    B
 *  852   7    8    9    C
 *  941   *    0    #    D
 * 
 */


static const char *dtmf_transl = "123A456B789C*0#D";

#define PHINC(x)   ((float)(x)*0x10000/SAMPLE_RATE)

static const unsigned int row_freq[4] = {
	PHINC(697), PHINC(770), PHINC(852), PHINC(941)
};

static const unsigned int col_freq[4] = {
	PHINC(1209), PHINC(1336), PHINC(1477), PHINC(1633)
};

void gen_init_dtmf(struct gen_params *p, struct gen_state *s)
{
	memset(s, 0, sizeof(struct gen_state));
}

int gen_dtmf(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
	char c;
	char *cp;
	int num = 0, i;

	for (; buflen > 0; buflen--, buf++, num++) {
		if (s->s.dtmf.time <= 0) {
			c = p->p.dtmf.str[s->s.dtmf.ch_idx];
			if (!c)
				return num;
			s->s.dtmf.ch_idx++;
			cp = memchr(dtmf_transl, toupper(c), 16);
			if (!cp) {
				s->s.dtmf.time = s->s.dtmf.time2 = 1;
				fprintf(stderr, "gen: dtmf; invalid char '%c'\n", c);
			} else {
				s->s.dtmf.time = p->p.dtmf.duration + p->p.dtmf.pause;
				s->s.dtmf.time2 = p->p.dtmf.duration;
				i = cp - dtmf_transl;
				s->s.dtmf.phinc_row = row_freq[(i >> 2) & 3];
				s->s.dtmf.phinc_col = col_freq[i & 3];
			}
		} else if (!s->s.dtmf.time2) {
			s->s.dtmf.phinc_row = s->s.dtmf.phinc_col = 0;
			s->s.dtmf.ph_row = s->s.dtmf.ph_col = 0xc000;
		}
		s->s.dtmf.time--;
		s->s.dtmf.time2--;
		*buf += ((p->ampl >> 1) * (COS(s->s.dtmf.ph_row) + COS(s->s.dtmf.ph_col))) >> 15;
		s->s.dtmf.ph_row += s->s.dtmf.phinc_row;
		s->s.dtmf.ph_col += s->s.dtmf.phinc_col;
	}
	return num;
}
