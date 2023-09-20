/*
 * This file is a modified subset of the R8lib from John Burkardt.
 * http://people.sc.fsu.edu/~jburkardt/c_src/r8lib/r8lib.html
 *
 * It is the minimal set of functions from r8lib needed to use qr_solve.
 *
 * This code is distributed under the GNU LGPL license.
 */

# include "r8lib_min.h"
/*#include "std.h"*/
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>
# include <stdlib.h>
/*# include <math.h>*/

#define DEBUG_FPRINTF(...)
#define DEBUG_PRINT(...)
#define DEBUG_EXIT(...)

void r8mat_copy_new ( int m, int n, num_t a1[], num_t a2[])

/******************************************************************************/
/*
  Purpose:

    R8MAT_COPY_NEW copies one R8MAT to a "new" R8MAT.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    26 July 2008

  Author:

    John Burkardt

  Parameters:

    Input, int M, N, the number of rows and columns.

    Input, num_t A1[M*N], the matrix to be copied.

    Output, num_t R8MAT_COPY_NEW[M*N], the copy of A1.
*/
{
  int i;
  int j;

  /*a2 = ( num_t * ) malloc ( m * n * sizeof ( num_t ) );*/

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      a2[i+j*m] = a1[i+j*m];
    }
  }
}
/******************************************************************************/

num_t r8_epsilon ( void )

/******************************************************************************/
/*
  Purpose:

    R8_EPSILON returns the R8 round off unit.

  Discussion:

    R8_EPSILON is a number R which is a power of 2 with the property that,
    to the precision of the computer's arithmetic,
      1 < 1 + R
    but
      1 = ( 1 + R / 2 )

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    01 September 2012

  Author:

    John Burkardt

  Parameters:

    Output, num_t R8_EPSILON, the R8 round-off unit.
*/
{
  const num_t value = 1.192092896E-7;

  return value;
}
/******************************************************************************/

num_t r8mat_amax ( int m, int n, num_t a[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_AMAX returns the maximum absolute value entry of an R8MAT.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 September 2012

  Author:

    John Burkardt

  Parameters:

    Input, int M, the number of rows in A.

    Input, int N, the number of columns in A.

    Input, num_t A[M*N], the M by N matrix.

    Output, num_t R8MAT_AMAX, the maximum absolute value entry of A.
*/
{
  int i;
  int j;
  num_t value;

  value = fabs ( a[0+0*m] );

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < m; i++ )
    {
      if ( value < fabs ( a[i+j*m] ) )
      {
        value = fabs ( a[i+j*m] );
      }
    }
  }
  return value;
}
/******************************************************************************/

num_t r8_sign ( num_t x )

/******************************************************************************/
/*
  Purpose:

    R8_SIGN returns the sign of an R8.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    08 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, num_t X, the number whose sign is desired.

    Output, num_t R8_SIGN, the sign of X.
*/
{
  num_t value;

  if ( x < 0.0 )
  {
    value = - 1.0;
  }
  else
  {
    value = + 1.0;
  }
  return value;
}
/******************************************************************************/

num_t r8_max ( num_t x, num_t y )

/******************************************************************************/
/*
  Purpose:

    R8_MAX returns the maximum of two R8's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, num_t X, Y, the quantities to compare.

    Output, num_t R8_MAX, the maximum of X and Y.
*/
{
  num_t value;

  if ( y < x )
  {
    value = x;
  }
  else
  {
    value = y;
  }
  return value;
}
/******************************************************************************/

num_t *r8mat_l_solve ( int n, num_t a[], num_t b[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_L_SOLVE solves a lower triangular linear system.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 June 2008

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of rows and columns of
    the matrix A.

    Input, num_t A[N*N], the N by N lower triangular matrix.

    Input, num_t B[N], the right hand side of the linear system.

    Output, num_t R8MAT_L_SOLVE[N], the solution of the linear system.
*/
{
  num_t dot;
  int i;
  int j;
  num_t *x;

  x = ( num_t * ) malloc ( n * sizeof ( num_t ) );
/*
  Solve L * x = b.
*/
  for ( i = 0; i < n; i++ )
  {
    dot = 0.0;
    for ( j = 0; j < i; j++ )
    {
      dot = dot + a[i+j*n] * x[j];
    }
    x[i] = ( b[i] - dot ) / a[i+i*n];
  }

  return x;
}
/******************************************************************************/

num_t *r8mat_lt_solve ( int n, num_t a[], num_t b[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_LT_SOLVE solves a transposed lower triangular linear system.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

    Given the lower triangular matrix A, the linear system to be solved is:

      A' * x = b

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    08 April 2009

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of rows and columns of the matrix A.

    Input, num_t A[N*N], the N by N lower triangular matrix.

    Input, num_t B[N], the right hand side of the linear system.

    Output, num_t R8MAT_LT_SOLVE[N], the solution of the linear system.
*/
{
  int i;
  int j;
  num_t *x;

  x = ( num_t * ) malloc ( n * sizeof ( num_t ) );

  for ( j = n-1; 0 <= j; j-- )
  {
    x[j] = b[j];
    for ( i = j+1; i < n; i++ )
    {
      x[j] = x[j] - x[i] * a[i+j*n];
    }
    x[j] = x[j] / a[j+j*n];
  }

  return x;
}
/******************************************************************************/

num_t *r8mat_mtv_new ( int m, int n, num_t a[], num_t x[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_MTV_NEW multiplies a transposed matrix times a vector.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

    For this routine, the result is returned as the function value.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    26 August 2011

  Author:

    John Burkardt

  Parameters:

    Input, int M, N, the number of rows and columns of the matrix.

    Input, num_t A[M,N], the M by N matrix.

    Input, num_t X[M], the vector to be multiplied by A.

    Output, num_t R8MAT_MTV_NEW[N], the product A'*X.
*/
{
  int i;
  int j;
  num_t *y;

  y = ( num_t * ) malloc ( n * sizeof ( num_t ) );

  for ( j = 0; j < n; j++ )
  {
    y[j] = 0.0;
    for ( i = 0; i < m; i++ )
    {
      y[j] = y[j] + a[i+j*m] * x[i];
    }
  }

  return y;
}
/******************************************************************************/

num_t r8vec_max ( int n, num_t r8vec[] )

/******************************************************************************/
/*
  Purpose:

    R8VEC_MAX returns the value of the maximum element in a R8VEC.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    05 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the array.

    Input, num_t R8VEC[N], a pointer to the first entry of the array.

    Output, num_t R8VEC_MAX, the value of the maximum element.  This
    is set to 0.0 if N <= 0.
*/
{
  int i;
  num_t value;

  if ( n <= 0 )
  {
    value = 0.0;
    return value;
  }

  value = r8vec[0];

  for ( i = 1; i < n; i++ )
  {
    if ( value < r8vec[i] )
    {
      value = r8vec[i];
    }
  }
  return value;
}
/******************************************************************************/

int i4_min ( int i1, int i2 )

/******************************************************************************/
/*
  Purpose:

    I4_MIN returns the smaller of two I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    29 August 2006

  Author:

    John Burkardt

  Parameters:

    Input, int I1, I2, two integers to be compared.

    Output, int I4_MIN, the smaller of I1 and I2.
*/
{
  int value;

  if ( i1 < i2 )
  {
    value = i1;
  }
  else
  {
    value = i2;
  }
  return value;
}
/******************************************************************************/

int i4_max ( int i1, int i2 )

/******************************************************************************/
/*
  Purpose:

    I4_MAX returns the maximum of two I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    29 August 2006

  Author:

    John Burkardt

  Parameters:

    Input, int I1, I2, are two integers to be compared.

    Output, int I4_MAX, the larger of I1 and I2.
*/
{
  int value;

  if ( i2 < i1 )
  {
    value = i1;
  }
  else
  {
    value = i2;
  }
  return value;
}
/******************************************************************************/
