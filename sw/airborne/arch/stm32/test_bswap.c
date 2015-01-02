
#include <inttypes.h>
#include "mcu.h"


#define MyByteSwap16(n)            \
  ( ((((uint16_t) n) << 8) & 0xFF00) |         \
    ((((uint16_t) n) >> 8) & 0x00FF) )

#define MyByteSwap32(n)            \
  ( ((((uint32_t) n) << 24) & 0xFF000000) |      \
    ((((uint32_t) n) <<  8) & 0x00FF0000) |      \
    ((((uint32_t) n) >>  8) & 0x0000FF00) |      \
    ((((uint32_t) n) >> 24) & 0x000000FF) )

#define MyByteSwap32_1(in, out) \
  asm volatile ( \
                 "rev        %0, %1\n\t" \
                 : "=r" (out) \
                 : "r"(in) \
               );


#define MyByteSwap16_1(in, out) \
  asm volatile ( \
                 "rev16        %0, %1\n\t" \
                 : "=r" (out) \
                 : "r"(in) \
               );



int main(void)
{

  //  uint32_t foo = 0;
  //  uint32_t bar = __builtin_bswap32(*(uint32_t*)&foo);

  //  uint16_t foo = 12345;
  //  uint16_t bar = foo >> 8 | foo << 8;

  //  uint16_t foo = 12345;
  //  uint16_t bar = ByteSwap(foo);

  uint16_t foo = 12345;
  uint16_t bar = MyByteSwap16(foo);
  MyByteSwap16_1(foo, bar);
  bar = __REV16(foo);



  uint32_t a = 23456;
  uint32_t b = MyByteSwap32(a);
  MyByteSwap32_1(a, b);

  return 0;
}
