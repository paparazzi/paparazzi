#ifndef VOR_FLOAT_FILTERS_H
#define VOR_FLOAT_FILTERS_H

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





#endif /* VOR_FLOAT_FILTERS_H */
