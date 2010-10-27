#ifndef VOR_INT_FILTERS_DECIM_H
#define VOR_INT_FILTERS_DECIM_H

extern inline int32_t vor_int_filter_bp_ref( int16_t xn);
extern inline int32_t vor_int_filter_lp_ref( int16_t xn);
extern inline int32_t vor_int_filter_bp_var( int16_t xn);
extern inline int32_t vor_int_filter_lp_decim1v( int16_t xn);
extern inline int32_t vor_int_filter_lp_decim1r( int16_t xn);
extern inline int32_t vor_int_filter_lp_decim2v( int16_t xn);
extern inline int32_t vor_int_filter_lp_decim2r( int16_t xn);
extern inline int32_t vor_int_filter_lp_var3( int16_t xn);
extern inline int32_t vor_int_filter_lp_fm3( int16_t xn);
extern inline int32_t vor_int_filter_lp_var4( int16_t xn);
extern inline int32_t vor_int_filter_lp_fm4( int16_t xn);

#undef VIF_RES
#define VIF_RES  14
#define VIF_FACT (1<<VIF_RES)
#define VIF_PCOEF(a) (a * VIF_FACT + 0.5)
#define VIF_NCOEF(a) (a * VIF_FACT - 0.5)

inline int32_t vor_int_filter_bp_ref( int16_t xn) {

#undef VIF_RES
#define VIF_RES  12

  const int32_t a0 = VIF_PCOEF(0.0175489156490784840000000000000000000000);
  // const int32_t a1 = VIF_PCOEF(0.0000000000000000000000000000000000000000);
  const int32_t a2 = VIF_NCOEF(-0.0526467469472354510000000000000000000000);
  // const int32_t a3 = VIF_PCOEF(0.0000000000000000000000000000000000000000);
  const int32_t a4 = VIF_PCOEF(0.0526467469472354510000000000000000000000);
  // const int32_t a5 = VIF_PCOEF(0.0000000000000000000000000000000000000000);
  const int32_t a6 = VIF_NCOEF(-0.0175489156490784840000000000000000000000);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_PCOEF(2.5508195725874163000000000000000000000000);
  const int32_t b2 = VIF_PCOEF(3.9857308274503627000000000000000000000000);
  const int32_t b3 = VIF_PCOEF(3.8245798569419009000000000000000000000000);
  const int32_t b4 = VIF_PCOEF(2.6296760724926846000000000000000000000000);
  const int32_t b5 = VIF_PCOEF(1.0926715614934313000000000000000000000000);
  const int32_t b6 = VIF_PCOEF(0.2825578543465128200000000000000000000000);

  // printf("%d %d %d %d %d %d %d\n",a0,a1,a2,a3,a4,a5,a6);
  // printf("Den : %d %d %d %d %d %d %d\n",b0,b1,b2,b3,b4,b5,b6);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;
  static int16_t x5 = 0;
  static int16_t x6 = 0;

  static int32_t _y1 = 0;
  static int32_t _y2 = 0;
  static int32_t _y3 = 0;
  static int32_t _y4 = 0;
  static int32_t _y5 = 0;
  static int32_t _y6 = 0;

  int32_t _yn =  a0 * xn
  //          + a1 * x1
              + a2 * x2
  //          + a3 * x3
              + a4 * x4
  //          + a5 * x5
              + a6 * x6
              - b1 * _y1
              - b2 * _y2
              - b3 * _y3
              - b4 * _y4
              - b5 * _y5
              - b6 * _y6;

  int32_t ret = _yn;

  _yn = (_yn/VIF_FACT);
  
  x6 = x5;
  x5 = x4;
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y6 = _y5;
  _y5 = _y4;
  _y4 = _y3;
  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

#undef VIF_RES
#define VIF_RES  11
  
  return (ret/VIF_FACT);
}

inline int32_t vor_int_filter_lp_ref( int16_t xn) {

#undef VIF_RES
#define VIF_RES  8

  const int32_t a0 = VIF_PCOEF(0.0375258681661874704538206515280762687325);
  const int32_t a1 = VIF_PCOEF(0.1125776044985624113614619545842288061976);
  const int32_t a2 = VIF_PCOEF(0.1125776044985624113614619545842288061976);
  const int32_t a3 = VIF_PCOEF(0.0375258681661874704538206515280762687325);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-1.3532012520987992676424482851871289312840);
  const int32_t b2 = VIF_PCOEF(0.8279475542861756132140271802200004458427);
  const int32_t b3 = VIF_NCOEF(-0.1745393568578767484744673765817424282432);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  int32_t ret = _yn;

  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

#undef VIF_RES
#define VIF_RES  7
  
  return (ret/VIF_FACT);
}

inline int32_t vor_int_filter_bp_var( int16_t xn) {
  
#undef VIF_RES
#define VIF_RES  13

  const int32_t a0 = VIF_PCOEF(0.0029300958945794793000000000000000000000);
  const int32_t a1 = VIF_PCOEF(0.0087902876837384382000000000000000000000);
  const int32_t a2 = VIF_PCOEF(0.0087902876837384382000000000000000000000);
  const int32_t a3 = VIF_PCOEF(0.0029300958945794793000000000000000000000);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.3715994101489439000000000000000000000000);
  const int32_t b2 = VIF_PCOEF(1.9257572822812750000000000000000000000000);
  const int32_t b3 = VIF_NCOEF(-0.5307171049756953500000000000000000000000);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int32_t _y1 = 0;
  static int32_t _y2 = 0;
  static int32_t _y3 = 0;

  int32_t _yn = a0 * xn
             + a1 * x1
             + a2 * x2
             + a3 * x3
             - b1 * _y1
	     - b2 * _y2
	     - b3 * _y3;

  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

inline int32_t vor_int_filter_lp_decim1v( int16_t xn) {

#undef VIF_RES
#define VIF_RES  13

  const int32_t a0 = VIF_PCOEF(0.0028981946337214297033935128666826130939);
  const int32_t a1 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a2 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a3 = VIF_PCOEF(0.0028981946337214297033935128666826130939);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.3740947437093518068706998747074976563454);
  const int32_t b2 = VIF_PCOEF(1.9293556690912150308747641247464343905449);
  const int32_t b3 = VIF_NCOEF(-0.5320753683120916788240606365434359759092);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  _yn = (_yn/VIF_FACT);

  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

inline int32_t vor_int_filter_lp_decim1r( int16_t xn) {

#undef VIF_RES
#define VIF_RES  13

  const int32_t a0 = VIF_PCOEF(0.0028981946337214297033935128666826130939);
  const int32_t a1 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a2 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a3 = VIF_PCOEF(0.0028981946337214297033935128666826130939);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.3740947437093518068706998747074976563454);
  const int32_t b2 = VIF_PCOEF(1.9293556690912150308747641247464343905449);
  const int32_t b3 = VIF_NCOEF(-0.5320753683120916788240606365434359759092);

  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

inline int32_t vor_int_filter_lp_decim2v( int16_t xn) {

#undef VIF_RES
#define VIF_RES  13

  const int32_t a0 = VIF_PCOEF(0.0028981946337214297033935128666826130939);
  const int32_t a1 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a2 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a3 = VIF_PCOEF(0.0028981946337214297033935128666826130939);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.3740947437093518068706998747074976563454);
  const int32_t b2 = VIF_PCOEF(1.9293556690912150308747641247464343905449);
  const int32_t b3 = VIF_NCOEF(-0.5320753683120916788240606365434359759092);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  int32_t ret = _yn;

  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

#undef VIF_RES
#define VIF_RES  12
  
  return (ret/VIF_FACT);
}

inline int32_t vor_int_filter_lp_decim2r( int16_t xn) {

#undef VIF_RES
#define VIF_RES  13

  const int32_t a0 = VIF_PCOEF(0.0028981946337214297033935128666826130939);
  const int32_t a1 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a2 = VIF_PCOEF(0.0086945839011642895438614075942496128846);
  const int32_t a3 = VIF_PCOEF(0.0028981946337214297033935128666826130939);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.3740947437093518068706998747074976563454);
  const int32_t b2 = VIF_PCOEF(1.9293556690912150308747641247464343905449);
  const int32_t b3 = VIF_NCOEF(-0.5320753683120916788240606365434359759092);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

//-Filtres des boucles 30 VAR et 30 FM ordre 3----------------------//

inline int32_t vor_int_filter_lp_var3( int16_t xn) {

#undef VIF_RES
#define VIF_RES  12

  const int32_t a0 = VIF_PCOEF(0.0046739610072115845423867952490581956226);
  const int32_t a1 = VIF_PCOEF(0.0140218830216347536271603857471745868679);
  const int32_t a2 = VIF_PCOEF(0.0140218830216347536271603857471745868679);
  const int32_t a3 = VIF_PCOEF(0.0046739610072115845423867952490581956226);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.2545578663321697021615364064928144216537);
  const int32_t b2 = VIF_PCOEF(1.7624338481987593674205072602489963173866);
  const int32_t b3 = VIF_NCOEF(-0.4704842938088966697307569120312109589577);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

inline int32_t vor_int_filter_lp_fm3( int16_t xn) {

#undef VIF_RES
#define VIF_RES  12

  const int32_t a0 = VIF_PCOEF(0.0046739610072115845423867952490581956226);
  const int32_t a1 = VIF_PCOEF(0.0140218830216347536271603857471745868679);
  const int32_t a2 = VIF_PCOEF(0.0140218830216347536271603857471745868679);
  const int32_t a3 = VIF_PCOEF(0.0046739610072115845423867952490581956226);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.2545578663321697021615364064928144216537);
  const int32_t b2 = VIF_PCOEF(1.7624338481987593674205072602489963173866);
  const int32_t b3 = VIF_NCOEF(-0.4704842938088966697307569120312109589577);
  
  // printf("Num : %d %d %d %d\n",a0,a1,a2,a3);
  // printf("Den : %d %d %d %d\n",b0,b1,b2,b3);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3;
  
  _yn = (_yn/VIF_FACT);
  
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

//-Filtres des boucles 30 VAR et 30 FM ordre 4----------------------//

inline int32_t vor_int_filter_lp_var4( int16_t xn) {

#undef VIF_RES
#define VIF_RES  12

  const int32_t a0 = VIF_PCOEF(0.0063471529575526031835552842608194623608);
  const int32_t a1 = VIF_PCOEF(0.0253886118302104127342211370432778494433);
  const int32_t a2 = VIF_PCOEF(0.0380829177453156225707786575185309629887);
  const int32_t a3 = VIF_PCOEF(0.0253886118302104127342211370432778494433);
  const int32_t a4 = VIF_PCOEF(0.0063471529575526031835552842608194623608);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.2338575278593175710284413071349263191223);
  const int32_t b2 = VIF_PCOEF(2.1052487831605173340676628868095576763153);
  const int32_t b3 = VIF_NCOEF(-0.9314940379298783934558514374657534062862);
  const int32_t b4 = VIF_PCOEF(0.1616572299495205866648461778822820633650);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;
  static int16_t _y4 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               + a4 * x4
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3
               - b4 * _y4;
  
  _yn = (_yn/VIF_FACT);
  
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y4 = _y3;
  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

inline int32_t vor_int_filter_lp_fm4( int16_t xn) {

#undef VIF_RES
#define VIF_RES  12

  const int32_t a0 = VIF_PCOEF(0.0063471529575526031835552842608194623608);
  const int32_t a1 = VIF_PCOEF(0.0253886118302104127342211370432778494433);
  const int32_t a2 = VIF_PCOEF(0.0380829177453156225707786575185309629887);
  const int32_t a3 = VIF_PCOEF(0.0253886118302104127342211370432778494433);
  const int32_t a4 = VIF_PCOEF(0.0063471529575526031835552842608194623608);

  // const int32_t b0 = VIF_PCOEF(1.0000000000000000000000000000000000000000);
  const int32_t b1 = VIF_NCOEF(-2.2338575278593175710284413071349263191223);
  const int32_t b2 = VIF_PCOEF(2.1052487831605173340676628868095576763153);
  const int32_t b3 = VIF_NCOEF(-0.9314940379298783934558514374657534062862);
  const int32_t b4 = VIF_PCOEF(0.1616572299495205866648461778822820633650);

  static int16_t x1 = 0;
  static int16_t x2 = 0;
  static int16_t x3 = 0;
  static int16_t x4 = 0;

  static int16_t _y1 = 0;
  static int16_t _y2 = 0;
  static int16_t _y3 = 0;
  static int16_t _y4 = 0;

  int32_t _yn =  a0 * xn
               + a1 * x1
               + a2 * x2
               + a3 * x3
               + a4 * x4
               - b1 * _y1
               - b2 * _y2
               - b3 * _y3
               - b4 * _y4;
  
  _yn = (_yn/VIF_FACT);
  
  x4 = x3;
  x3 = x2;
  x2 = x1;
  x1 = xn;

  _y4 = _y3;
  _y3 = _y2;
  _y2 = _y1;
  _y1 = _yn;

  return (_yn);
}

#endif /* VOR_INT_FILTERS_DECIM_H */
