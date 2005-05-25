/*
 *      demod_zvei.c -- ZVEI signalling demodulator/decoder
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

#define SAMPLE_RATE 22050
#define BLOCKLEN (SAMPLE_RATE/100)  /* 10ms blocks */
#define BLOCKNUM 4    /* must match numbers in multimon.h */

#define PHINC(x) ((x)*0x10000/SAMPLE_RATE)

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

/* ---------------------------------------------------------------------- */
	
static void zvei_init(struct demod_state *s)
{
	memset(&s->l1.zvei, 0, sizeof(s->l1.zvei));
}

/* ---------------------------------------------------------------------- */

static int find_max_idx(const float *f)
{
	float en = 0;
	int idx = -1, i;

	for (i = 0; i < 16; i++)
		if (f[i] > en) {
			en = f[i];
			idx = i;
		}
	if (idx < 0)
		return -1;
	en *= 0.1;
	for (i = 0; i < 16; i++)
		if (idx != i && f[i] > en)
			return -1;
	return idx;
}

/* ---------------------------------------------------------------------- */

static inline int process_block(struct demod_state *s)
{
	float tote;
	float totte[32];
	int i, j;

	tote = 0;
	for (i = 0; i < BLOCKNUM; i++)
		tote += s->l1.zvei.energy[i];
	for (i = 0; i < 32; i++) {
		totte[i] = 0;
		for (j = 0; j < BLOCKNUM; j++)
			totte[i] += s->l1.zvei.tenergy[j][i];
	}
	for (i = 0; i < 16; i++)
		totte[i] = fsqr(totte[i]) + fsqr(totte[i+16]);
	memmove(s->l1.zvei.energy+1, s->l1.zvei.energy, 
		sizeof(s->l1.zvei.energy) - sizeof(s->l1.zvei.energy[0]));
	s->l1.zvei.energy[0] = 0;
	memmove(s->l1.zvei.tenergy+1, s->l1.zvei.tenergy, 
		sizeof(s->l1.zvei.tenergy) - sizeof(s->l1.zvei.tenergy[0]));
	memset(s->l1.zvei.tenergy, 0, sizeof(s->l1.zvei.tenergy[0]));
	tote *= (BLOCKNUM*BLOCKLEN*0.5);  /* adjust for block lengths */
	verbprintf(10, "ZVEI: Energies: %8.5f  %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f"
		   " %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n",
		   tote, totte[0], totte[1], totte[2], totte[3], totte[4], totte[5], totte[6], totte[7],
		   totte[8], totte[9], totte[10], totte[11], totte[12], totte[13], totte[14], totte[15]);
	if ((i = find_max_idx(totte)) < 0)
		return -1;
	if ((tote * 0.4) > totte[i])
		return -1;
	return i;
}

/* ---------------------------------------------------------------------- */

static void zvei_demod(struct demod_state *s, float *buffer, int length)
{
	float s_in;
	int i;

	for (; length > 0; length--, buffer++) {
		s_in = *buffer;
		s->l1.zvei.energy[0] += fsqr(s_in);
		for (i = 0; i < 16; i++) {
			s->l1.zvei.tenergy[0][i] += COS(s->l1.zvei.ph[i]) * s_in;
			s->l1.zvei.tenergy[0][i+16] += SIN(s->l1.zvei.ph[i]) * s_in;
			s->l1.zvei.ph[i] += zvei_freq[i];
		}
		if ((s->l1.zvei.blkcount--) <= 0) {
			s->l1.zvei.blkcount = BLOCKLEN;
			i = process_block(s);
			if (i != s->l1.zvei.lastch && i >= 0)
				verbprintf(0, "ZVEI: %1x\n", i);
			s->l1.zvei.lastch = i;
		}
	}
}
				
/* ---------------------------------------------------------------------- */

const struct demod_param demod_zvei = {
	"ZVEI", SAMPLE_RATE, 0, zvei_init, zvei_demod
};

/* ---------------------------------------------------------------------- */
