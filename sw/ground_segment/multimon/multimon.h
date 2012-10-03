/*
 *      multimon.h -- Monitor for many different modulation formats
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

#ifndef _MULTIMON_H
#define _MULTIMON_H

/* ---------------------------------------------------------------------- */

extern const float costabf[0x400];
#define COS(x) costabf[(((x)>>6)&0x3ffu)]
#define SIN(x) COS((x)+0xc000)

/* ---------------------------------------------------------------------- */

struct demod_state {
  const struct demod_param *dem_par;
  union {
    struct l2_state_hdlc {
      unsigned char rxbuf[512];
      unsigned char *rxptr;
      unsigned int rxstate;
      unsigned int rxbitstream;
      unsigned int rxbitbuf;
    } hdlc;

    struct l2_state_pprz {
      unsigned char rxbuf[512];
      unsigned char *rxptr;
      unsigned int rxstate;
      unsigned int rxbitstream;
      unsigned int rxbitbuf;
      char* pipe_path;
      int pipe_fd;
    } pprz;
  } l2;
  union {
    struct l1_state_afsk48p {
      unsigned int dcd_shreg;
      unsigned int sphase;
      unsigned int lasts;
      unsigned int dcd_count;
      unsigned int sample_count;
    } afsk48p;

    struct l1_state_scope {
      int datalen;
      int dispnum;
      float data[512];
    } scope;

    struct l1_state_afsk12 {
      unsigned int dcd_shreg;
      unsigned int sphase;
      unsigned int lasts;
      unsigned int subsamp;
    } afsk12;
  } l1;
};

struct demod_param {
  const char *name;
  unsigned int samplerate;
  unsigned int overlap;
  void (*init)(struct demod_state *s);
  void (*demod)(struct demod_state *s, float *buffer, int length);
};


#define HDLC_DATA_LEN 256
extern char hdlc_data_received[HDLC_DATA_LEN];
extern int hdlc_data_received_idx;


/* ---------------------------------------------------------------------- */

extern const struct demod_param demod_afsk4800p;
extern const struct demod_param demod_afsk1200;
extern const struct demod_param demod_scope;

#define ALL_DEMOD &demod_afsk4800p,&demod_scope

/* ---------------------------------------------------------------------- */

void verbprintf(int verb_level, const char *fmt, ...);

void hdlc_init(struct demod_state *s);
void hdlc_rxbit(struct demod_state *s, int bit);

void afsk12_init(struct demod_state *s);

void pprz_init(struct demod_state *s);
void pprz_rxbit(struct demod_state *s, int bit);
void pprz_status(struct demod_state *s);

void xdisp_terminate(int cnum);
int xdisp_start(void);
int xdisp_update(int cnum, float *f);

/* ---------------------------------------------------------------------- */
#endif /* _MULTIMON_H */
