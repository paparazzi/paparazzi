#ifndef VOR_LF_FILTER_H
#define VOR_LF_FILTER_H

struct Filter {
  unsigned nb_num;
  unsigned nb_den;
  const FLOAT_T* num;
  const FLOAT_T* den;
  FLOAT_T* x;
  FLOAT_T* y;
};

extern void vor_lf_filter_init(struct Filter* f,
			       unsigned nb_num, unsigned nb_den,
			       const FLOAT_T* num, const FLOAT_T* den);

extern FLOAT_T vor_lf_filter_run(struct Filter *f, FLOAT_T xn);


#endif /* VOR_LF_FILTER_H */

