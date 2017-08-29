#include "fmatrix.h"

/* Function to add two matrices to eachother */
void fmat_add(int n_row, int n_col, float* r, float* a, float* b) {
	int row, col, ridx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] + b[ridx];
		}
	}
}

/* Function to subtract two matrices from eachother */
void fmat_sub(int n_row, int n_col, float* r, float* a, float* b) {
	int row, col, ridx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] - b[ridx];
		}
	}
}

/* Function to obtain the transpose of a matrix */
void fmat_transpose(int n_row, int n_col, float* r, float* a) {
	int row, col, ridx, aidx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			aidx = row * n_col + col;
			ridx = col * n_row + row;
			r[ridx] = a[aidx];
		}
	}
}

/* Function to multiply a matrix by a scalar value */
void fmat_scal_mult(int n_row, int n_col, float* r, float k, float* a) {
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			if (a[ridx] != 0.0)
				r[ridx] = k * a[ridx];
			else
				r[ridx] = 0.0;
		}
	}
}

/* Add a scalar value to a matrix */
void fmat_add_scal_mult(int n_row, int n_col, float* r, float*a, float k, float* b) {
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] + k * b[ridx];
		}
	}
}

/* Multiply two matrices with eachother */
void fmat_mult(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b) {
	int row, col, k, ridx, aidx, bidx;
	for (row = 0; row < n_rowa; row++)
	{
		for (col = 0; col < n_colb; col++)
		{
			ridx = col + row * n_colb;
			r[ridx] =0.;
			for (k=0; k < n_cola; k++)
			{
				aidx = k + row * n_cola;
				bidx = col + k * n_colb;
				r[ridx] += a[aidx] * b[bidx];
			}
		}
	}
}

#ifndef ARM_COMPILER

/* Print the matrix */
void fmat_print(int n_row, int n_col, float* a)
{
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			printf("%2.2f\t", a[ridx]);
		}
		printf("\n");
	}
}

#endif

/* Calculate inverse of matrix 

Compliments of:
https://www.quora.com/How-do-I-make-a-C++-program-to-get-the-inverse-of-a-matrix-100-X-100

Algorithm verified with Matlab
*/
void fmat_inverse(int n, float* matinv, float *mat)
{
	int i, j, k, row, col;
	float t;

	float a[2*n*n];

	/* Append an ide1ntity matrix on the right of the original matrix */
	for(row = 0; row < n; row++)
	{
		for(col = 0; col < 2*n; col++)
		{
			if (col < n)
			{
				a[ row*2*n+col ] = mat[row*n + col];
			}
			else if( (col >= n) && (col == row+n))
			{
				a[ row*2*n+col ] = 1.0;
			}
			else
			{
				a[ row*2*n+col ] = 0.0;
			}
		}	
	}

	/* Do the inversion */
	for( i = 0; i < n; i++)
	{
		// Store diagonal variable (temp)
		t = a[ i*2*n + i ];
		
		for(j = i; j < 2*n; j++)
		{
			// Divide by the diagonal value
			a[ i*2*n + j ] = a[ i*2*n + j ]/t;
		}

		for(j = 0; j < n; j++)

		{
			if( i!=j )
			{
				t = a[ j*2*n + i ];
				
				for( k = 0; k < 2*n; k++)
				{
					a[j*2*n + k] = a[j*2*n + k] - t*a[i*2*n + k];
				}

			}

		}

	}

	/* Cut out the identity, which has now moved to the left side */
	for(row = 0 ; row < n ; row++ )
	{
		for(col = n; col < 2*n; col++ )
		{
			matinv[row * n + col - n] = a[row*2*n + col];
		}
		
	}

};

/* Make a matrix of zeros */
void fmat_make_zeroes(float *matrix, int row, int col)
{
	int i,j;
	for(i = 0 ; i < row; i++)
	{
		for(j = 0 ; j < col; j++)
		{
			matrix[i*col+j] = 0.0;
		}
	}
};

/* Make an identity matrix */
void fmat_make_identity(float *matrix, int n)
{
	int i,j;
	for(i = 0 ; i < n; i++)
	{
		for(j = 0 ; j < n; j++)
		{
			if (i == j)
			{
				matrix[i*n+j] = 1.0;
			}
			else
			{
				matrix[i*n+j] = 0.0;
			}
		}
	}
};