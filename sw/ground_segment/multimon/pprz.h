#ifndef PPRZ_H
#define PPRZ_H


void pprz_init(struct demod_state *s);
void pprz_baudot_rxbit(struct demod_state *s, int bit);
void pprz_hdlc_rxbit(struct demod_state *s, int bit);
void pprz_status(struct demod_state *s);

#endif /* PPRZ_H */
