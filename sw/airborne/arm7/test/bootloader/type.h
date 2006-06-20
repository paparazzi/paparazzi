/*
	(c) 2006, Bertrik Sikken, bertrik@sikken.nl
*/

/* primitive types used in the USB stack */

#ifndef _TYPE_H_
#define _TYPE_H_

typedef unsigned char		U8;
typedef unsigned short int	U16;
typedef unsigned int		U32;

typedef int					BOOL;

#define	TRUE	1
#define FALSE	0

#ifndef NULL
#define NULL	((void*)0)
#endif

/* some other useful macros */
#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))


#endif /* _TYPE_H_ */

