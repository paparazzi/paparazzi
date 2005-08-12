#ifndef PPRZ_H
#define PPRZ_H

extern char multimon_pipe_name[];


void pprz_init(struct demod_state *s);
void pprz_baudot_rxbit(struct demod_state *s, int bit);
void pprz_hdlc_rxbit(struct demod_state *s, int bit);
void pprz_status(struct demod_state *s);

#endif /* PPRZ_H */
