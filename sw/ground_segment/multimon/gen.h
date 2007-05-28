/*
 *      gen.h -- generate different test signals
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

#define DEFAULT_SAMPLE_RATE 22050
extern int hdlc_sample_rate;

#define MS(x) ((float)(x)*SAMPLE_RATE/1000)

extern const int costabi[0x400];

enum gen_type { gentype_dtmf, gentype_sine, gentype_zvei, gentype_hdlc };

struct gen_params {
	enum gen_type type;
	int ampl;
	union {
		struct {
			int duration;
			int pause;
			char str[256];
		} dtmf;
		struct {
			int duration;
			int freq;
		} sine;
		struct {
			int duration;
			int pause;
			char str[256];
		} zvei;
		struct {
			int modulation;
			int txdelay;
			int pktlen;
			unsigned char pkt[256];
		} hdlc;
	} p;
};

struct gen_state {
	union {
		struct {
			int ch_idx;
			int ph_row, ph_col, phinc_row, phinc_col;
			int time, time2;
		} dtmf;
		struct {
			int ph, phinc;
			int time;
		} sine;
		struct {
			int ch_idx;
			int ph, phinc;
			int time, time2;
		} zvei;
		struct {
			int lastb;
			int ch_idx, bitmask;
			unsigned int ph, phinc, bitph;
			unsigned int datalen;
			unsigned char data[512];
		} hdlc;
	} s;
};

extern void gen_init_dtmf(struct gen_params *p, struct gen_state *s);
extern int gen_dtmf(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s);

extern void gen_init_sine(struct gen_params *p, struct gen_state *s);
extern int gen_sine(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s);

extern void gen_init_zvei(struct gen_params *p, struct gen_state *s);
extern int gen_zvei(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s);

extern void gen_init_hdlc(struct gen_params *p, struct gen_state *s);
extern int gen_hdlc(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s);

