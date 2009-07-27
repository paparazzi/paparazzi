/* r250.h	prototypes for r250 random number generator,

		Kirkpatrick, S., and E. Stoll, 1981; "A Very Fast
		Shift-Register Sequence Random Number Generator",
		Journal of Computational Physics, V.40

		also:

		see W.L. Maier, DDJ May 1991


*/

#ifndef _R250_H_
#define _R250_H_ 1.2

void         r250_init(int seed);
unsigned int r250( void );
double       dr250( void );

#endif
