#include "shape.h"

/*
	Returns the area of a 2D geometric shape on a cartesian plane
	The shape is described by the array: [x1 y1 x2 y2 .. .. xn yn]

	The shoelace formula is used to calculate the area:
		area = 1/2 * sum i to n { x(i) * ( y(i+1) - y(i-1) ) }
*/
float shape_getarea(float *tr, int size)
{

	int i;

	float a = 0.0;

	float temp[size];

	memcpy(temp, tr, size * sizeof(float));

	for (i = 0; i < size/2; i++)
	{
		a += temp[0] * ( temp[3] - temp[size-1] );
		array_shiftleft(temp, size, 2);
	}

	return fabs(a)/2.0;

}


/*
	Rotate a geometric shape on a cartesian plane about the origin
	The shape is described by the array: [x1 y1 x2 y2 .. .. xn yn]

	It will rotate the triangle about the origin point.
	Angle is counter-clockwise.

	x' = x cos f - y sin f
	y' = y cos f + x sin f

	Courtesy of 
	https://www.siggraph.org/education/materials/HyperGraph/modeling/mod_tran/2drota.htm
	
*/
void shape_rotateatorigin(float *tr, int size, float ang)
{

	int i;

	float temp[size];
	memcpy(temp, tr, size * sizeof(float));
	
	/*Rotate all x points */
	for (i = 0; i < size; i = i+2)
	{
		tr[i] = temp[i]*cos(ang) - temp[i+1]*sin(ang);
	}

	/*Rotate all y points */
	for (i = 1; i < size; i = i+2)
	{
		tr[i] = temp[i]*cos(ang) + temp[i-1]*sin(ang);
	}
	
}

/*
	Rotate a geometric shape on a cartesian plane about the origin
	The shape is described by the array: [x1 y1 x2 y2 .. .. xn yn]

	It will rotate the triangle about the origin point.
	Angle is counter-clockwise.

	x' = ( (x-x0) cos f - (y-y0) sin f ) + x0
	y' = ( (y-y0) cos f + (x-x0) sin f ) + y0
	
*/
void shape_rotateatpoint(float *tr, int size, float ang, float x0, float y0)
{

	int i;

	float temp[size];
	memcpy(temp, tr, size * sizeof(float));

	/*Rotate all x points */
	for (i = 0; i < size; i = i+2)
	{
		tr[i] = (temp[i] - x0)*cos(ang) - (temp[i+1] - y0)*sin(ang);
		tr[i] = tr[i] + x0;
	}

	/*Rotate all y points */
	for (i = 1; i < size; i = i+2)
	{
		tr[i] = (temp[i] - y0)*cos(ang) + (temp[i-1] - x0)*sin(ang);
		tr[i] = tr[i] + y0;
	}

}


/*
	Shift a shape on a cartesian plane
	The shape is described by the array: [x1 y1 x2 y2 .. .. xn yn]
*/
void shape_shift(float *tr, int size, float x, float y)
{
	int i;

	/* Move all x points by the x shift */
	for (i = 0; i < size; i = i+2)
	{
		tr[i] = tr[i] + x;
	}

	/* Rotate all y points by the y shift */
	for (i = 1; i < size; i = i+2)
	{
		tr[i] = tr[i] + y;
	}

}


/*

	Checks if a point is contained within the area delimeted by the shape defined by the array on a cartesian plane.

	The array is defiend by the points [x1 y1 x2 y2 .. .. xn yn]

	The function is based on the trick that the area between all triangles should be the same as the area of the shape.

	If the area is larger than the area of the shape, then the point must be outside of the polygon shape -- function returns 0

	If the point is inside then the areas are the same and the function returns 1

*/

bool shape_checkifpointinarea(float* tr, int size, float* px)
{
	int i;

	float a = shape_getarea(tr, size);
	float test_a = 0.0;

	float temp[size];
	memcpy(temp, tr, size * sizeof(float));

	// Divide the shape up in triangles and check if they fit
	for (i = 0; i < size; i = i+2)
	{

		temp[i] = px[0];
		temp[i+1] = px[1];

		test_a += shape_getarea(temp, 6);

		memcpy(temp, tr, size * sizeof(float));
		array_shiftleft(temp, size, i+2);

	}
	
	if ((test_a - a) > 0.1)
		return false;
	else 
		return true;

}