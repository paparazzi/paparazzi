/*
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
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 *
 *    ===================================================================
 *    #                                                                 #
 *    # Important note:                                                 #
 *    # Please pay respect to the different perspectives that Eigen     #
 *    # uses in comparison to Paparazzi.                                #
 *    # See the documentation in doc/pprz_algebra for more explanation. #
 *    #                                                                 #
 *    # As a result the euler angles are negated                        #
 *    #                                                                 #
 *    # You should also pay attention to the multtiplications:          #
 *    # Although quaternions and matrices "from?to?" are unique         #
 *    # The order in the mutliplication might change                    #
 *    #                                                                 #
 *    #   Thank you for you attention :-)                               #
 *    #                                                                 #
 *    ===================================================================
 */

/*
 * additional mathematical transformations
 * (could also be in math/algebra.h)
 */

#define PI_180 1.74532925199433e-02
#define ARCSEC_ACRMIN_ANGLE_IN_RADIANS(degree, arcmin, arcsec) ((degree+arcmin/60+arcsec/3600)*M_PI/180)
#define ABS(a) ((a<0)?-a:a)
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MAXABS(a,b) MAX(ABS(a),ABS(b))
#define MAT33_TRANSP(_to,_from) {                       \
    MAT33_ELMT((_to),0,0) = MAT33_ELMT((_from),0,0);  \
    MAT33_ELMT((_to),0,1) = MAT33_ELMT((_from),1,0);  \
    MAT33_ELMT((_to),0,2) = MAT33_ELMT((_from),2,0);  \
    MAT33_ELMT((_to),1,0) = MAT33_ELMT((_from),0,1);  \
    MAT33_ELMT((_to),1,1) = MAT33_ELMT((_from),1,1);  \
    MAT33_ELMT((_to),1,2) = MAT33_ELMT((_from),2,1);  \
    MAT33_ELMT((_to),2,0) = MAT33_ELMT((_from),0,2);  \
    MAT33_ELMT((_to),2,1) = MAT33_ELMT((_from),1,2);  \
    MAT33_ELMT((_to),2,2) = MAT33_ELMT((_from),2,2);  \
  }
#define SWAP(a, b){       \
    typeof (a) temp = a;    \
    a = b;                  \
    b = temp;               \
  }

#define MAT33_ROW(mat, row, value0, value1, value2){    \
    mat[row*3+0]  = value0;                               \
    mat[row*3+1]  = value1;                               \
    mat[row*3+2]  = value2;                               \
  }

#define ENU_NED_transformation(mat){    \
    SWAP(mat.m[0], mat.m[3]);       \
    SWAP(mat.m[1], mat.m[4]);       \
    SWAP(mat.m[2], mat.m[5]);       \
    mat.m[6] = -mat.m[6];         \
    mat.m[7] = -mat.m[7];         \
    mat.m[8] = -mat.m[8];         \
  }

#define DOUBLE_VECT3_NORM(_v) (sqrt((_v).x*(_v).x + (_v).y*(_v).y + (_v).z*(_v).z))

#define DOUBLE_QUAT_OF_RMAT(_q, _r) {         \
    const double tr = RMAT_TRACE(_r);         \
    if (tr > 0) {             \
      const double two_qi = sqrt(1.+tr);        \
      const double four_qi = 2. * two_qi;       \
      _q.qi = 0.5 * two_qi;           \
      _q.qx =  (RMAT_ELMT(_r, 1, 2)-RMAT_ELMT(_r, 2, 1))/four_qi; \
      _q.qy =  (RMAT_ELMT(_r, 2, 0)-RMAT_ELMT(_r, 0, 2))/four_qi; \
      _q.qz =  (RMAT_ELMT(_r, 0, 1)-RMAT_ELMT(_r, 1, 0))/four_qi; \
      /*printf("tr > 0\n");*/           \
    }                 \
    else {                \
      if (RMAT_ELMT(_r, 0, 0) > RMAT_ELMT(_r, 1, 1) &&      \
          RMAT_ELMT(_r, 0, 0) > RMAT_ELMT(_r, 2, 2)) {      \
        const double two_qx = sqrt(RMAT_ELMT(_r, 0, 0) -RMAT_ELMT(_r, 1, 1) \
                                   -RMAT_ELMT(_r, 2, 2) + 1);   \
        const double four_qx = 2. * two_qx;       \
        _q.qi = (RMAT_ELMT(_r, 1, 2)-RMAT_ELMT(_r, 2, 1))/four_qx;  \
        _q.qx = 0.5 * two_qx;           \
        _q.qy = (RMAT_ELMT(_r, 0, 1)+RMAT_ELMT(_r, 1, 0))/four_qx;  \
        _q.qz = (RMAT_ELMT(_r, 2, 0)+RMAT_ELMT(_r, 0, 2))/four_qx;  \
        /*printf("m00 largest\n");*/          \
      }                 \
      else if (RMAT_ELMT(_r, 1, 1) > RMAT_ELMT(_r, 2, 2)) {   \
        const double two_qy =           \
                                        sqrt(RMAT_ELMT(_r, 1, 1) - RMAT_ELMT(_r, 0, 0) - RMAT_ELMT(_r, 2, 2) + 1); \
        const double four_qy = 2. * two_qy;       \
        _q.qi = (RMAT_ELMT(_r, 2, 0) - RMAT_ELMT(_r, 0, 2))/four_qy;  \
        _q.qx = (RMAT_ELMT(_r, 0, 1) + RMAT_ELMT(_r, 1, 0))/four_qy;  \
        _q.qy = 0.5 * two_qy;           \
        _q.qz = (RMAT_ELMT(_r, 1, 2) + RMAT_ELMT(_r, 2, 1))/four_qy;  \
        /*printf("m11 largest\n");*/          \
      }                 \
      else {                \
        const double two_qz =           \
                                        sqrt(RMAT_ELMT(_r, 2, 2) - RMAT_ELMT(_r, 0, 0) - RMAT_ELMT(_r, 1, 1) + 1); \
        const double four_qz = 2. * two_qz;       \
        _q.qi = (RMAT_ELMT(_r, 0, 1)- RMAT_ELMT(_r, 1, 0))/four_qz; \
        _q.qx = (RMAT_ELMT(_r, 2, 0)+ RMAT_ELMT(_r, 0, 2))/four_qz; \
        _q.qy = (RMAT_ELMT(_r, 1, 2)+ RMAT_ELMT(_r, 2, 1))/four_qz; \
        _q.qz = 0.5 * two_qz;           \
        /*printf("m22 largest\n");*/          \
      }                 \
    }                 \
  }

#define QUAT_ENU_FROM_TO_NED(from, to){ \
    to.qi = - from.qx - from.qy;          \
    to.qy = + from.qi + from.qz;          \
    to.qx = + from.qi - from.qz;          \
    to.qz = - from.qx + from.qy;          \
    QUAT_SMUL(to, to, M_SQRT1_2);         \
  }

#define QUAT_IMAGINARY_PART(quat, vector) VECT3_ASSIGN(vector, (quat).qx, (quat).qy, (quat).qz)

#define VECT3_TO_EULERS(vector, eulers){  \
    (eulers).phi    = (vector).x;           \
    (eulers).theta  = (vector).y;           \
    (eulers).psi    = (vector).z;           \
  }


/*
 * transformations from paparazzi to Eigen
 */

#define VECT3_AS_VECTOR3D(coords) Vector3d(coords.x, coords.y, coords.z)
#define EULER_AS_VECTOR3D(euler) -Vector3d(euler.phi, euler.theta, euler.psi);
#define RATES_AS_VECTOR3D(rates) Vector3d(rates.p,rates.q,rates.r)
#define RATES_BFP_AS_VECTOR3D(rates) Vector3d(RATE_FLOAT_OF_BFP(rates.p),RATE_FLOAT_OF_BFP(rates.q),RATE_FLOAT_OF_BFP(rates.r))
#define DOUBLEQUAT_AS_QUATERNIOND(quat) Quaterniond(quat.qi, quat.qx, quat.qy, quat.qz)

#define RMAT_TO_EIGENMATRIX(Eigen,c)  Eigen << (c)[0],(c)[1],(c)[2],(c)[3],(c)[4],(c)[5],(c)[6],(c)[7],(c)[8]


/*
 * tranformations from Eigen to paparazzi
 */

#define VECTOR_AS_VECT3(coords, vector) { coords.x = vector(0); coords.y = vector(1); coords.z = vector(2);}
#define QUATERNIOND_AS_DOUBLEQUAT(doublequat, quaterniond) {(doublequat).qi = (quaterniond).w(); (doublequat).qx = (quaterniond).x(); (doublequat).qy = (quaterniond).y(); (doublequat).qz = (quaterniond).z();}
#define PPRZ_LLA_TO_EIGEN_ECEF(lla, ecef){  \
    struct EcefCoor_f ecef_pprz;              \
    ecef_of_lla_f(&ecef_pprz, &lla);          \
    ecef = VECT3_AS_VECTOR3D(ecef_pprz);      \
  }
