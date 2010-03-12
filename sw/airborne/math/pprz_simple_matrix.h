#ifndef PPRZ_SIMPLE_MATRIX_H
#define PPRZ_SIMPLE_MATRIX_H

//
// C = A*B   A:(i,k) B:(k,j) C:(i,j)
//
#define MAT_MUL(_i, _k, _j, C, A, B) {				\
    int l,c,m;							\
    for (l=0; l<_i; l++)					\
      for (c=0; c<_j; c++) {					\
	C[l][c] = 0.;						\
	for (m=0; m<_k; m++)					\
	  C[l][c] += A[l][m]*B[m][c];				\
      }								\
  }

//
// C = A*B'   A:(i,k) B:(j,k) C:(i,j)
//
#define MAT_MUL_T(_i, _k, _j, C, A, B) {			\
    int l,c,m;							\
    for (l=0; l<_i; l++)					\
      for (c=0; c<_j; c++) {					\
	C[l][c] = 0.;						\
	for (m=0; m<_k; m++)					\
	  C[l][c] += A[l][m]*B[c][m];				\
      }								\
  }


//
// C = A-B
//
#define MAT_SUB(_i, _j, C, A, B) {					\
    int l,c;								\
    for (l=0; l<_i; l++)						\
      for (c=0; c<_j; c++)						\
	C[l][c] = A[l][c] - B[l][c];					\
  }




//
// invS = 1/det(S) com(S)'
//
#define MAT_INV33(invS, S) {						\
    const float m00 = S[1][1]*S[2][2] - S[1][2]*S[2][1];		\
    const float m10 = S[0][1]*S[2][2] - S[0][2]*S[2][1];		\
    const float m20 = S[0][1]*S[1][2] - S[0][2]*S[1][1];		\
    const float m01 = S[1][0]*S[2][2] - S[1][2]*S[2][0];		\
    const float m11 = S[0][0]*S[2][2] - S[0][2]*S[2][0];		\
    const float m21 = S[0][1]*S[1][2] - S[0][2]*S[1][0];		\
    const float m02 = S[1][0]*S[2][1] - S[1][1]*S[2][0];		\
    const float m12 = S[0][0]*S[2][1] - S[0][1]*S[2][0];		\
    const float m22 = S[0][0]*S[1][1] - S[0][1]*S[1][0];		\
    const float det = S[0][0]*m00 - S[1][0]*m10 + S[2][0]*m20;		\
    if (fabs(det) > FLT_EPSILON) {					\
      invS[0][0] =  m00 / det;						\
      invS[1][0] = -m01 / det;						\
      invS[2][0] =  m02 / det;						\
      invS[0][1] = -m10 / det;						\
      invS[1][1] =  m11 / det;						\
      invS[2][1] = -m12 / det;						\
      invS[0][2] =  m20 / det;						\
      invS[1][2] = -m21 / det;						\
      invS[2][2] =  m22 / det;						\
    }									\
  }


#endif /* PPRZ_SIMPLE_MATRIX_H */

