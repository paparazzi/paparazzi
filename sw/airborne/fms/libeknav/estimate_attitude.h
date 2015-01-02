#ifndef ESTIMATE_ATTITUDE_H
#define ESTIMATE_ATTITUDE_H

/* for sqrt */
#include <math.h>

#include "paparazzi_eigen_conversion.h"
#include "../../math/pprz_algebra.h"
#include "../../math/pprz_algebra_double.h"

/** beacuse I'm lazy    **/
#define M3(Mat, row, col) MAT33_ELMT(Mat, row,col)

/** everything for the 4D-algebra **/
struct DoubleMat44 {
  double m[4 * 4];
};
typedef struct DoubleQuat DoubleVect4;

// accessing single values
#define M4(Mat, row, col) Mat.m[row*4+col]

#define DOUBLE_MAT44_ASSIGN(Mat, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p){ \
    M4(Mat, 0,0) = a; M4(Mat, 0,1) = b; M4(Mat, 0,2) = c; M4(Mat, 0,3) = d;         \
    M4(Mat, 1,0) = e; M4(Mat, 1,1) = f; M4(Mat, 1,2) = g; M4(Mat, 1,3) = h;         \
    M4(Mat, 2,0) = i; M4(Mat, 2,1) = j; M4(Mat, 2,2) = k; M4(Mat, 2,3) = l;         \
    M4(Mat, 3,0) = m; M4(Mat, 3,1) = n; M4(Mat, 3,2) = o; M4(Mat, 3,3) = p;         \
  }

#define DOUBLE_MAT_VMULT4(out, mat, in){                                                  \
    (out).qi = M4(mat, 0,0)*(in).qi + M4(mat, 0,1)*(in).qx + M4(mat, 0,2)*(in).qy + M4(mat, 0,3)*(in).qz;  \
    (out).qx = M4(mat, 1,0)*(in).qi + M4(mat, 1,1)*(in).qx + M4(mat, 1,2)*(in).qy + M4(mat, 1,3)*(in).qz;  \
    (out).qy = M4(mat, 2,0)*(in).qi + M4(mat, 2,1)*(in).qx + M4(mat, 2,2)*(in).qy + M4(mat, 2,3)*(in).qz;  \
    (out).qz = M4(mat, 3,0)*(in).qi + M4(mat, 3,1)*(in).qx + M4(mat, 3,2)*(in).qy + M4(mat, 3,3)*(in).qz;  \
  }

struct DoubleMat44 square_skaled(struct DoubleMat44);

/* everything for normes */
#define MAX4(a,b,c,d) (MAX(MAX(a,b),MAX(c,d)))
#define MAX4ABS(a,b,c,d) (MAX(MAXABS(a,b),MAXABS(c,d)))
#define MAX16ABS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) MAX4(MAX4ABS(a,b,c,d), MAX4ABS(e,f,g,h), MAX4ABS(i,j,k,l), MAX4ABS(m,n,o,p))

#define INFTY_NORM4(v) (MAX4ABS(v.qi,v.qx,v.qy,v.qz))
#define INFTY_NORM16(M) MAX16ABS( M.m[ 0], M.m[ 1], M.m[ 2], M.m[ 3], \
                                  M.m[ 4], M.m[ 5], M.m[ 6], M.m[ 7], \
                                  M.m[ 8], M.m[ 9], M.m[10], M.m[11], \
                                  M.m[12], M.m[13], M.m[14], M.m[15])

#define NORM_VECT4(v) sqrt(SQUARE(v.qi)+SQUARE(v.qx)+SQUARE(v.qy)+SQUARE(v.qz))

/** Wahba-solver **/
DoubleVect4 dominant_Eigenvector(struct DoubleMat44, unsigned int, double, struct DoubleMat44, DoubleVect4 *);
struct DoubleMat44 generate_K_matrix(struct DoubleMat33);
struct DoubleQuat  estimated_attitude(struct DoubleMat33, unsigned int, double, struct DoubleMat33,
                                      struct DoubleQuat *);


/** Attitude Measurement Handling **/
struct Orientation_Measurement {
  struct DoubleVect3 reference_direction,
      measured_direction;
  double weight_of_the_measurement;
};

void add_orientation_measurement(struct DoubleMat33 *, struct Orientation_Measurement);
struct Orientation_Measurement fake_orientation_measurement(struct Orientation_Measurement,
    struct Orientation_Measurement);
void add_set_of_three_measurements(struct DoubleMat33 *, struct Orientation_Measurement,
                                   struct Orientation_Measurement);

#endif /* ESTIMATE_ATTITUDE_H */
