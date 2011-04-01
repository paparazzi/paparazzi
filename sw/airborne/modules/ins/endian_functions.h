/**********************************************************************************************
* Ryan Mechatronics firmware (C) 2007 - All Rights Reserved
* CONFIDENTIAL: NO PART OF THIS CODE MAY BE RELEASED WITHOUT WRITTEN PERMISSION
* ---------------------------------------------------------------------------------------------
*
* Module:
*       Endian Functions - Handles various low level endian swap functions
*
***********************************************************************************************/
#ifndef ENDIAN_H
#define ENDIAN_H


short int ShortSwap( short int s );

short int ShortNoSwap( short int s );

int LongSwap (int i);

int LongNoSwap( int i );

float FloatSwap( float f );

float FloatNoSwap( float f );

#endif
