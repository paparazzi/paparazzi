
/* randlcg.h	prototypes for the minimal standard random number generator,

   Linear Congruential Method, the "minimal standard generator"
   Park & Miller, 1988, Comm of the ACM, 31(10), pp. 1192-1201


  rcsid: @(#)randlcg.h	1.1 15:48:09 11/21/94   EFC

*/

#ifndef _RANDLCG_H_
#define _RANDLCG_H_ 1.1

long         set_seed(long);
long         get_seed(long);
unsigned long int randlcg();

#endif
