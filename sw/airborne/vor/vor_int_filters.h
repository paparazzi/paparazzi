#ifndef VOR_INT_FILTERS_H
#define VOR_INT_FILTERS_H


#define VIF_CRES  16
#define VIF_CFACT (1<<VIF_CRES)
#define VIF_PCOEF(a) (a * VIF_CFACT + 0.5)
#define VIF_NCOEF(a) (a * VIF_CFACT - 0.5)

#define VIF_SRES  10
#define VIF_SFACT (1<<VIF_SRES)

#define VIF_SSCALE 5

inline int32_t vor_int_filter_bp_var( int16_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0005064187715341341225472326925682864385);
  const int32_t a1 = VIF_PCOEF( 0.0015192563146024023676416980777048593154);
  const int32_t a2 = VIF_PCOEF( 0.0015192563146024023676416980777048593154);
  const int32_t a3 = VIF_PCOEF( 0.0005064187715341341225472326925682864385);

  const int32_t b1 = VIF_NCOEF(-2.6639389158386133082956348516745492815971);
  const int32_t b2 = VIF_PCOEF( 2.3820072066637907326480672054458409547806);
  const int32_t b3 = VIF_NCOEF(-0.7140169406529043305553727805090602487326);

  //  printf("%d %d %d %d\n", a0, a1, a2, a3);
  //  printf("%d %d %d\n", b1, b2, b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int32_t _y1 = 0;
  static int32_t y2 = 0;
  static int32_t y3 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
              + a1 * x1
              + a2 * x2
              + a3 * x3
              - b1 * _y1
              - b2 * y2
              - b3 * y3;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);
}

inline int32_t vor_int_filter_bp_ref( int16_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0175489156490784836694984960558940656483);
  //  const int32_t a1 = VIF_PCOEF( 0.0000000000000000000000000000000000000000);
  const int32_t a2 = VIF_NCOEF(-0.0526467469472354510084954881676821969450);
  //  const int32_t a3 = VIF_PCOEF( 0.0000000000000000000000000000000000000000);
  const int32_t a4 = VIF_PCOEF( 0.0526467469472354510084954881676821969450);
  //  const int32_t a5 = VIF_PCOEF( 0.0000000000000000000000000000000000000000);
  const int32_t a6 = VIF_NCOEF(-0.0175489156490784836694984960558940656483);

  const int32_t b1 = VIF_PCOEF( 2.5508195725874163173330089193768799304962);
  const int32_t b2 = VIF_PCOEF( 3.9857308274503626677187639870680868625641);
  const int32_t b3 = VIF_PCOEF( 3.8245798569419009460546021728077903389931);
  const int32_t b4 = VIF_PCOEF( 2.6296760724926846464200025366153568029404);
  const int32_t b5 = VIF_PCOEF( 1.0926715614934312537087635064381174743176);
  const int32_t b6 = VIF_PCOEF( 0.2825578543465128156242371915141120553017);

  //  printf("%d %d %d %d %d %d %d\n", a0, a1, a2, a3, a4, a5, a6);
  //  printf("%d %d %d %d %d %d\n", b1, b2, b3, b4, b5, b6);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;
  static int16_t x5 = 0;
  static int16_t x6 = 0;

  static int32_t _y1 = 0;
  static int32_t y2 = 0;
  static int32_t y3 = 0;
  static int32_t y4 = 0;
  static int32_t y5 = 0;
  static int32_t y6 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
  //          + a1 * x1
              + a2 * x2
  //          + a3 * x3
              + a4 * x4
  //          + a5 * x5
              + a6 * x6
              - b1 * _y1
              - b2 * y2
              - b3 * y3
              - b4 * y4
              - b5 * y5
              - b6 * y6;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x6 = x5;
  x5 = x4;
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y6 = y5;
  y5 = y4;
  y4 = y3;
  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);
}



#define VIF_CRES_t  16
#define VIF_CFACT_t (1<<VIF_CRES_t)
#define VIF_PCOEF_t(a) (a * VIF_CFACT_t + 0.5)
#define VIF_NCOEF_t(a) (a * VIF_CFACT_t - 0.5)

//#define VIF_SRES_t  10
//#define VIF_SFACT_t (1<<VIF_SRES)

//#define VIF_SSCALE_t 5



inline int32_t vor_int_filter_lp_decim( int16_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0005064187715341341225472326925682864385);
  const int32_t a1 = VIF_PCOEF( 0.0015192563146024023676416980777048593154);
  const int32_t a2 = VIF_PCOEF( 0.0015192563146024023676416980777048593154);
  const int32_t a3 = VIF_PCOEF( 0.0005064187715341341225472326925682864385);

  const int32_t b1 = VIF_NCOEF(-2.6639389158386133082956348516745492815971);
  const int32_t b2 = VIF_PCOEF( 2.3820072066637907326480672054458409547806);
  const int32_t b3 = VIF_NCOEF(-0.7140169406529043305553727805090602487326);

  // printf("%d %d %d %d\n", a0, a1, a2, a3);
  // printf("%d %d %d\n", b1, b2, b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t y2 = 0;
  static int16_t y3 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
              + a1 * x1
              + a2 * x2
              + a3 * x3
              - b1 * _y1
              - b2 * y2
              - b3 * y3;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);
}


inline int32_t vor_int_filter_lp_var( int32_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0001325928962621713658003030911203268261);
  const int32_t a1 = VIF_PCOEF( 0.0007955573775730281948018185467219609563);
  const int32_t a2 = VIF_PCOEF( 0.0019888934439325706496348722396305674920);
  const int32_t a3 = VIF_PCOEF( 0.0026518579252434275328464963195074233226);
  const int32_t a4 = VIF_PCOEF( 0.0019888934439325706496348722396305674920);
  const int32_t a5 = VIF_PCOEF( 0.0007955573775730281948018185467219609563);
  const int32_t a6 = VIF_PCOEF( 0.0001325928962621713658003030911203268261);

  const int32_t b1 = VIF_NCOEF(-3.9811884900947003274040980613790452480316);
  const int32_t b2 = VIF_PCOEF( 6.8452604202756832663112618320155888795853);
  const int32_t b3 = VIF_NCOEF(-6.4498289229953256196381516929250210523605);
  const int32_t b4 = VIF_PCOEF( 3.4951386512490887348292289971141144633293);
  const int32_t b5 = VIF_NCOEF(-1.0292502263335985279724127394729293882847);
  const int32_t b6 = VIF_PCOEF( 0.1283545132596315418993526691338047385216);

  //  printf("%d %d %d %d %d %d %d\n", a0, a1, a2, a3, a4, a5, a6);
  //  printf("%d %d %d %d %d %d\n", b1, b2, b3, b4, b5, b6);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;
  static int16_t x5 = 0;
  static int16_t x6 = 0;

  static int32_t _y1 = 0;
  static int32_t y2 = 0;
  static int32_t y3 = 0;
  static int32_t y4 = 0;
  static int32_t y5 = 0;
  static int32_t y6 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
              + a1 * x1
              + a2 * x2
              + a3 * x3
              + a4 * x4
              + a5 * x5
              + a6 * x6
              - b1 * _y1
              - b2 * y2
              - b3 * y3
              - b4 * y4
              - b5 * y5
              - b6 * y6;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x6 = x5;
  x5 = x4;
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y6 = y5;
  y5 = y4;
  y4 = y3;
  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);
}

inline int32_t vor_int_filter_lp_ref( int32_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0182843871571847782497854950634064152837);
  const int32_t a1 = VIF_PCOEF( 0.0548531614715543347493564851902192458510);
  const int32_t a2 = VIF_PCOEF( 0.0548531614715543347493564851902192458510);
  const int32_t a3 = VIF_PCOEF( 0.0182843871571847782497854950634064152837);

  const int32_t b1 = VIF_NCOEF(-1.7551740553005390488294779061106964945793);
  const int32_t b2 = VIF_PCOEF( 1.1780240265537846866550353297498077154160);
  const int32_t b3 = VIF_NCOEF(-0.2765748739957675783607271569053409621119);

  // printf("%d %d %d %d\n", a0, a1, a2, a3);
  // printf("%d %d %d\n", b1, b2, b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int32_t _y1 = 0;
  static int32_t y2 = 0;
  static int32_t y3 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
              + a1 * x1
              + a2 * x2
              + a3 * x3
              - b1 * _y1
              - b2 * y2
              - b3 * y3;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);
}

inline int32_t vor_int_filter_lp_fm( int32_t xn) {

  const int32_t a0 = VIF_PCOEF( 0.0001325928962621713658003030911203268261);
  const int32_t a1 = VIF_PCOEF( 0.0007955573775730281948018185467219609563);
  const int32_t a2 = VIF_PCOEF( 0.0019888934439325706496348722396305674920);
  const int32_t a3 = VIF_PCOEF( 0.0026518579252434275328464963195074233226);
  const int32_t a4 = VIF_PCOEF( 0.0019888934439325706496348722396305674920);
  const int32_t a5 = VIF_PCOEF( 0.0007955573775730281948018185467219609563);
  const int32_t a6 = VIF_PCOEF( 0.0001325928962621713658003030911203268261);

  const int32_t b1 = VIF_NCOEF(-3.9811884900947003274040980613790452480316);
  const int32_t b2 = VIF_PCOEF( 6.8452604202756832663112618320155888795853);
  const int32_t b3 = VIF_NCOEF(-6.4498289229953256196381516929250210523605);
  const int32_t b4 = VIF_PCOEF( 3.4951386512490887348292289971141144633293);
  const int32_t b5 = VIF_NCOEF(-1.0292502263335985279724127394729293882847);
  const int32_t b6 = VIF_PCOEF( 0.1283545132596315418993526691338047385216);

  //  printf("%d %d %d %d %d %d %d\n", a0, a1, a2, a3, a4, a5, a6);
  //  printf("%d %d %d %d %d %d\n", b1, b2, b3, b4, b5, b6);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;
  static int16_t x5 = 0;
  static int16_t x6 = 0;

  static int32_t _y1 = 0;
  static int32_t y2 = 0;
  static int32_t y3 = 0;
  static int32_t y4 = 0;
  static int32_t y5 = 0;
  static int32_t y6 = 0;

  /* scale input */
  xn = (xn << VIF_SSCALE);

  int32_t _yn =  a0 * xn
              + a1 * x1
              + a2 * x2
              + a3 * x3
              + a4 * x4
              + a5 * x5
              + a6 * x6
              - b1 * _y1
              - b2 * y2
              - b3 * y3
              - b4 * y4
              - b5 * y5
              - b6 * y6;
  //  printf("%d\n", _yn);
  _yn = (_yn >> VIF_CRES);
  
  x6 = x5;
  x5 = x4;
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y6 = y5;
  y5 = y4;
  y4 = y3;
  y3 = y2;
  y2 = _y1;
  _y1 = _yn;

  return (_yn >> VIF_SSCALE);

}

#endif /* VOR_INT_FILTERS_H */
