#ifndef PPRZ_GEODETIC_H
#define PPRZ_GEODETIC_H


#define ENU_OF_TO_NED(_po, _pi) {		\
    (_po).x =  (_pi).y;				\
    (_po).y =  (_pi).x;				\
    (_po).z = -(_pi).z;				\
  }

#define LLA_ASSIGN(_pos,_lat,_lon,_alt){	\
    (_pos).lat = (_lat);			\
    (_pos).lon = (_lon);			\
    (_pos).alt = (_alt);			\
  }

#define LLA_COPY(_pos1,_pos2){			\
    (_pos1).lat = (_pos2).lat;			\
    (_pos1).lon = (_pos2).lon;			\
    (_pos1).alt = (_pos2).alt;			\
}

#define LTP_DEF_COPY(_def1,_def2){                              \
    LLA_COPY((_def1).lla, (_def2).lla);                         \
    VECT3_COPY((_def1).ecef, (_def2).ecef);                     \
    RMAT_COPY((_def1).ltp_of_ecef, (_def2).ltp_of_ecef);        \
    (_def1).hmsl = (_def2).hmsl;                                \
  }

#define UTM_COPY(_u1, _u2) {     \
    (_u1).north = (_u2).north;   \
    (_u1).east = (_u2).east;     \
    (_u1).alt = (_u2).alt;       \
    (_u1).zone = (_u2).zone;     \
  }


#define ENU_OF_UTM_DIFF(_pos, _utm1, _utm2) { \
  (_pos).x = (_utm1).east - (_utm2).east;     \
  (_pos).y = (_utm1).north - (_utm2).north;   \
  (_pos).z = (_utm1).alt - (_utm2).alt;       \
}

#define NED_OF_UTM_DIFF(_pos, _utm1, _utm2) { \
  (_pos).x = (_utm1).north - (_utm2).north;   \
  (_pos).y = (_utm1).east - (_utm2).east;     \
  (_pos).z = -(_utm1).alt + (_utm2).alt;      \
}


#endif /* PPRZ_GEODETIC_H */
