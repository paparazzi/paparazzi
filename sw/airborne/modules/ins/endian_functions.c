/**********************************************************************************************
* Ryan Mechatronics firmware (C) 2007 - All Rights Reserved
* CONFIDENTIAL: NO PART OF THIS CODE MAY BE RELEASED WITHOUT WRITTEN PERMISSION
* ---------------------------------------------------------------------------------------------
*
* Module:
*       Endian Functions - Handles various low level endian swap functions
*
***********************************************************************************************/
//-----------------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------------
#include "endian_functions.h"
#include <globals.h>


short int ShortSwap( short int s )
{
  unsigned char b1, b2;
  
  b1 = s & 255;
  b2 = (s >> 8) & 255;

  return (b1 << 8) + b2;
}

short int ShortNoSwap( short int s )
{
  return s;
}

int LongSwap (int i)
{
  unsigned char b1, b2, b3, b4;

  b1 = i & 255;
  b2 = ( i >> 8 ) & 255;
  b3 = ( i>>16 ) & 255;
  b4 = ( i>>24 ) & 255;

  return ((int)b1 << 24) + ((int)b2 << 16) + ((int)b3 << 8) + b4;
}

int LongNoSwap( int i )
{
  return i;
}

float FloatSwap( float f )
{
  union
  {
    float f;
    unsigned char b[4];
  } dat1, dat2;

  dat1.f = f;
  dat2.b[0] = dat1.b[3];
  dat2.b[1] = dat1.b[2];
  dat2.b[2] = dat1.b[1];
  dat2.b[3] = dat1.b[0];
  return dat2.f;
}

float FloatNoSwap( float f )
{
  return f;
}






