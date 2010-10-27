#ifndef VOR_FLOAT_FILTERS_H
#define VOR_FLOAT_FILTERS_H

//typedef float (*vor_float_filter_fun)( float xn);

inline float vor_float_filter_bp_var( float xn) {

  const float a0 =  0.0000294918571461433008684162315748977790;
  const float a1 =  0.0000884755714384299093815122727590960494;
  const float a2 =  0.0000884755714384299093815122727590960494;
  const float a3 =  0.0000294918571461433008684162315748977790;

  const float b1 = -2.8738524677701420273479016032069921493530;
  const float b2 =  2.7555363566291544152875303552718833088875;
  const float b3 = -0.8814479540018430592240861187747213989496;

  static float x1 = 0;
  static float x2 = 0;
  static float x3 = 0;

  static float y1 = 0;
  static float y2 = 0;
  static float y3 = 0;

  float yn =  a0 * xn
            + a1 * x1
            + a2 * x2
            + a3 * x3
            - b1 * y1
            - b2 * y2
            - b3 * y3;
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = y1;
  y1 = yn;

  return yn;
}

inline float vor_float_filter_bp_ref( float xn) {

  const float a0 =  0.0175489156490784836694984960558940656483;
  //  const float a1 =  0.0000000000000000000000000000000000000000;
  const float a2 = -0.0526467469472354510084954881676821969450;
  //  const float a3 =  0.0000000000000000000000000000000000000000;
  const float a4 =  0.0526467469472354510084954881676821969450;
  //  const float a5 = 0.0000000000000000000000000000000000000000;
  const float a6 = -0.0175489156490784836694984960558940656483;

  const float b1 =  2.5508195725874163173330089193768799304962;
  const float b2 =  3.9857308274503626677187639870680868625641;
  const float b3 =  3.8245798569419009460546021728077903389931;
  const float b4 =  2.6296760724926846464200025366153568029404;
  const float b5 =  1.0926715614934312537087635064381174743176;
  const float b6 =  0.2825578543465128156242371915141120553017;

  static float x1 = 0;
  static float x2 = 0;
  static float x3 = 0;
  static float x4 = 0;
  static float x5 = 0;
  static float x6 = 0;

  static float y1 = 0;
  static float y2 = 0;
  static float y3 = 0;
  static float y4 = 0;
  static float y5 = 0;
  static float y6 = 0;

  float yn =  a0 * xn
  //        + a1 * x1
            + a2 * x2
  //        + a3 * x3
            + a4 * x4
  //        + a5 * x5
            + a6 * x6
            - b1 * y1
            - b2 * y2
            - b3 * y3
            - b4 * y4
            - b5 * y5
            - b6 * y6;
  
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
  y2 = y1;
  y1 = yn;

  return yn;
}


inline float vor_float_filter_lp_decim( float xn) {

  const float a0 =  0.0000294918571461433008684162315748977790;
  const float a1 =  0.0000884755714384299093815122727590960494;
  const float a2 =  0.0000884755714384299093815122727590960494;
  const float a3 =  0.0000294918571461433008684162315748977790;

  const float b1 = -2.8738524677701420273479016032069921493530;
  const float b2 =  2.7555363566291544152875303552718833088875;
  const float b3 = -0.8814479540018430592240861187747213989496;

  static float x1 = 0.;
  static float x2 = 0.;
  static float x3 = 0.;

  static float y1 = 0.;
  static float y2 = 0.;
  static float y3 = 0.;

  float yn = a0 * xn
           + a1 * x1
           + a2 * x2
           + a3 * x3
           - b1 * y1
           - b2 * y2
           - b3 * y3;

  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = y1;
  y1 = yn;

  return yn;
}

inline float vor_float_filter_lp_var( float xn) {

  const float a0 =  0.0001325928962621713658003030911203268261;
  const float a1 =  0.0007955573775730281948018185467219609563;
  const float a2 =  0.0019888934439325706496348722396305674920;
  const float a3 =  0.0026518579252434275328464963195074233226;
  const float a4 =  0.0019888934439325706496348722396305674920;
  const float a5 =  0.0007955573775730281948018185467219609563;
  const float a6 =  0.0001325928962621713658003030911203268261;

  const float b1 = -3.9811884900947003274040980613790452480316;
  const float b2 =  6.8452604202756832663112618320155888795853;
  const float b3 = -6.4498289229953256196381516929250210523605;
  const float b4 =  3.4951386512490887348292289971141144633293;
  const float b5 = -1.0292502263335985279724127394729293882847;
  const float b6 =  0.1283545132596315418993526691338047385216;

  static float x1 = 0.;
  static float x2 = 0.;
  static float x3 = 0.;
  static float x4 = 0.;
  static float x5 = 0.;
  static float x6 = 0.;

  static float y1 = 0.;
  static float y2 = 0.;
  static float y3 = 0.;
  static float y4 = 0.;
  static float y5 = 0.;
  static float y6 = 0.;

  float yn = a0 * xn
           + a1 * x1
           + a2 * x2
           + a3 * x3
           + a4 * x4
           + a5 * x5
           + a6 * x6
           - b1 * y1
           - b2 * y2
           - b3 * y3
           - b4 * y4
           - b5 * y5
           - b6 * y6;
  
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
  y2 = y1;
  y1 = yn;

  return yn;
}


inline float vor_float_filter_lp_ref( float xn) {

  const float a0 =  0.0182843871571847782497854950634064152837;
  const float a1 =  0.0548531614715543347493564851902192458510;
  const float a2 =  0.0548531614715543347493564851902192458510;
  const float a3 =  0.0182843871571847782497854950634064152837;

  const float b1 = -1.7551740553005390488294779061106964945793;
  const float b2 =  1.1780240265537846866550353297498077154160;
  const float b3 = -0.2765748739957675783607271569053409621119;

  static float x1 = 0.;
  static float x2 = 0.;
  static float x3 = 0.;

  static float y1 = 0.;
  static float y2 = 0.;
  static float y3 = 0.;

  float yn = a0 * xn
           + a1 * x1
           + a2 * x2
           + a3 * x3
           - b1 * y1
           - b2 * y2
           - b3 * y3;

  x3 = x2;
  x2 = x1;
  x1 = xn;

  y3 = y2;
  y2 = y1;
  y1 = yn;

  return yn;
}


inline float vor_float_filter_lp_fm( float xn) {

  const float a0 =  0.0001325928962621713658003030911203268261;
  const float a1 =  0.0007955573775730281948018185467219609563;
  const float a2 =  0.0019888934439325706496348722396305674920;
  const float a3 =  0.0026518579252434275328464963195074233226;
  const float a4 =  0.0019888934439325706496348722396305674920;
  const float a5 =  0.0007955573775730281948018185467219609563;
  const float a6 =  0.0001325928962621713658003030911203268261;

  const float b1 = -3.9811884900947003274040980613790452480316;
  const float b2 =  6.8452604202756832663112618320155888795853;
  const float b3 = -6.4498289229953256196381516929250210523605;
  const float b4 =  3.4951386512490887348292289971141144633293;
  const float b5 = -1.0292502263335985279724127394729293882847;
  const float b6 =  0.1283545132596315418993526691338047385216;

  static float x1 = 0.;
  static float x2 = 0.;
  static float x3 = 0.;
  static float x4 = 0.;
  static float x5 = 0.;
  static float x6 = 0.;

  static float y1 = 0.;
  static float y2 = 0.;
  static float y3 = 0.;
  static float y4 = 0.;
  static float y5 = 0.;
  static float y6 = 0.;

  float yn = a0 * xn
           + a1 * x1
           + a2 * x2
           + a3 * x3
           + a4 * x4
           + a5 * x5
           + a6 * x6
           - b1 * y1
           - b2 * y2
           - b3 * y3
           - b4 * y4
           - b5 * y5
           - b6 * y6;
  
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
  y2 = y1;
  y1 = yn;

  return yn;

}

#endif /* VOR_FLOAT_FILTERS_H */
