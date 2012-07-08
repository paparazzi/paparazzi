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

#endif /* PPRZ_GEODETIC_H */
