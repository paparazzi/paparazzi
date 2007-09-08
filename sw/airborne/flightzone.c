/*
 * $Id$
 *  
 * Copyright (C) 2007  Arnold Schroeter
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file  flightzone.c 
 *  \brief check whether a point is inside the polygon limiting the 
 *         competition area	
 * 
 * filename:        flightzone.c
 * project:         MAV 2007
 * description:     check whether a point is inside the polygon limiting
 *                  the competition area
 *                  
 *                 
 * todo:            - support concave/convex polygons
 *                  - sort points automatically
 *                  - minimize size of Orthogonal[]
 *				   
 * 				   
 * limitations:     - boundary polygon has to be convex				   
 *                  - points have to be ordered clockwise
 *				   
 * author:          Arnold Schroeter
 * history:         
 *                  2.9.07 initial version
 *
 */

#include "flightzone.h"

typedef struct { COORD_TYPE x;
				 COORD_TYPE y;
			   } POINT;

POINT Corner[] = {
				 12, 18,
				 12, 25,
				 15, 29,
				 18, 25,
				 18, 18, 
 				 13.5, 16,
				 0 , 0}; // last corner is a dummy, which must not be deleted!!! 

POINT Orthogonal[20]; // Attention!!! array must be at least as long as Corner[]

unsigned char bNumberOfCorners = 0;

/*******************************************************************
; function name: vInitIsInsideBoundaries  	
; description:   	
;*******************************************************************/
void vInitIsInsideBoundaries(void)
{
	unsigned char i;

   	bNumberOfCorners = sizeof(Corner)/sizeof(POINT) - 1; // last corner is always a dummy

	Corner[bNumberOfCorners].x = Corner[0].x;
	Corner[bNumberOfCorners].y = Corner[0].y;

	for (i = 0; i < bNumberOfCorners; i++)
	{
		Orthogonal[i].x =     Corner[i+1].y - Corner[i].y;
		Orthogonal[i].y =  - (Corner[i+1].x - Corner[i].x);

#if 0
		printf("%d: corner (%f, %f), orthogonal (%f, %f)\n", 
		        i, 
		        Corner[i].x, Corner[i].y, 				    
		        Orthogonal[i].x, Orthogonal[i].y);
#endif
	}

}

/*******************************************************************
; function name: iIsInsideBoundaries  	
; description:   	
;                   
; parameters: 		 
; returns:       1 if point is inside boundaries
;                0 if point is outside boundaries           
;*******************************************************************/
int iIsInsideBoundaries(COORD_TYPE x, COORD_TYPE y)
{

	int r = 1;
	static unsigned char i; 

 	i = 0;

   	while (    (i < bNumberOfCorners)
	        && (r == 1)
		  )
    {
		if (  (   (x - Corner[i].x) * Orthogonal[i].x 
		        + (y - Corner[i].y) * Orthogonal[i].y
		      ) < 0. )
		{
			r = 0;
		}

		++i;
    }

	return r;
}


