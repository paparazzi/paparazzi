#include "arrayfunctions.h"

/*
Shifts an array to the left by a defined number of steps at a time AND copies the left most element back to the right side!
*/
void array_shiftleft(float *array, int size, int shift)
{
	int i;
	float temp[size];
	memcpy(temp, array, size * sizeof(float));

	for (i = 0; i < size; i++)
	{
		if (i+shift >= size) {
			array[i] = temp[i+shift - size];
		}
		else { 
				array[i] = temp[i+shift];
			}
	}

}

void array_shiftleft_bool(bool *array, int size, int shift)
{
	int i;
	bool temp[size];
	memcpy(temp, array, size * sizeof(bool));

	for (i = 0; i < size; i++)
	{
		if (i+shift >= size) {
			array[i] = temp[i+shift - size];
		}
		else { 
				array[i] = temp[i+shift];
			}
	}

}

/*
Shifts an array to the right by a defined number of steps at a time
*/
void array_shiftright(float *array, int size, int shift)
{
	int i;
	float temp[size];
	memcpy(temp, array, size * sizeof(float));

	for (i = 0; i < size; i++) {
		if (i-shift < 0) {		
			array[i] = temp[i-shift + size];
		}
		else {
			array[i] = temp[i-shift];
		}
	}
}
void array_shiftright_bool(bool *array, int size, int shift)
{
	int i;
	bool temp[size];
	memcpy(temp, array, size * sizeof(bool));

	for (i = 0; i < size; i++) {
		if (i-shift < 0) {		
			array[i] = temp[i-shift + size];
		}
		else {
			array[i] = temp[i-shift];
		}
	}
}


/*
Returns the idx of the minimum value in an array
*/
int array_getminidx(int length, float *x)
{
	int i;
	int idxmin = 0;
	float min = x[0];

	for (i = 1; i < length; i++) {
		if (x[i] < min) {
			min = x[i];
			idxmin = i;
		}
	}

	return idxmin;
}

/* 
Returns the idx of the maximum value in an array
*/
int array_getmaxidx(int length, float *x)
{
	int i;
	int idxmax = 0;
	float max = x[0];

	for (i = 1; i < length; i++) {
		if (x[i] > max) {
			max = x[i];
			idxmax = i;
		}
	}

	return idxmax;
}


/* 

Makes the vector x1 into a vector that features the minimum value
at each index position when comparing vector x1 to vector x2 .

x1 is altered.
x2 is not altered.

*/
void array_arraymin(int length, float *x1, float *x2)
{
	int i;

	for (i = 0; i < length; i++) {
		if (x1[i] > x2[i]) {
			x1[i] = x2[i];
		}
	}
}

/* 

Makes the vector x1 into a vector that features the maximum value
at each index position when comparing vector x1 to vector x2 .

x1 is altered.
x2 is not altered.

*/
void array_arraymax(int length, float *x1, float *x2)
{
	int i;
	for (i = 0; i < length; i++) {
		if (x1[i] < x2[i]) {
			x1[i] = x2[i];
		}
	}
}

/* Finds a value in an array and returns the location */
bool array_find_int(int length, int *x, int value, int *location) {
	int i;
	for (i = 0; i < length; i++) {
		if (x[i] == value) {
			*location = i;
			return true;
		}
	}
	*location = -1;
	return false;
}


#ifndef ARM_COMPILER
void array_print(int length, float *x)
{
	int i;

	for (i = 0; i < length; i++) {
		printf("%2.2f\t", x[i]);
	}

	printf("\n");
}

void array_print_int(int length, int *x)
{
	int i;

	for (i = 0; i < length; i++) {
		printf("%d\t", x[i]);
	}

	printf("\n");
}

void array_print_bool(int length, bool *x)
{
	int i;

	for (i = 0; i < length; i++) {
		printf("%d\t", x[i]);
	}

	printf("\n");
}
#endif

void array_make_zeros(int length, float *x)
{
	int i;
	for(i = 0 ; i < length; i++) {
		x[i] = 0.0;
	}
};

void array_make_zeros_int(int length, int *x)
{
	int i;
	for(i = 0 ; i < length; i++) {
		x[i] = 0;
	}
};

void array_make_zeros_bool(int length, bool *x)
{
	int i;
	for(i = 0 ; i < length; i++) {
		x[i] = false;
	}
};

void array_make_ones(int length, float *x)
{
	int i;
	for(i = 0 ; i < length; i++) {
		x[i] = 1.0;
	}
};

void array_mult_scal(int length, float *y, float k, float *x)
{
	int i;
	for(i = 0 ; i < length; i++) {
		y[i] = k * x[i];
	}
};

float array_sum(int length, float *x)
{
	int i;
	float out = 0.0;
	for (i = 0; i < length; i++)
	{
		out = out + x[i];
	}
	return out;
}

float array_sum_weighted(int length, float *x)
{
	int i;
	float out = 0.0;
	for (i = 0; i < length; i++)
	{
		out+=(x[i]*(1/((float)length-(float)i+1.0)));
	}
	return out;
}

void array_copy_bool(bool *array_out, bool *array_in, int size)
{
	memcpy(array_out, array_in, size * sizeof(bool));
}