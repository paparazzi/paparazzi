/**
 * @file common.h
 * common internal api header.
 */

//#define ALT_BITSTREAM_WRITER
//#define ALIGNED_BITSTREAM_WRITER

#define ALT_BITSTREAM_READER
//#define LIBMPEG2_BITSTREAM_READER
//#define A32_BITSTREAM_READER
#define LIBMPEG2_BITSTREAM_READER_HACK //add BERO

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

#include <stddef.h>
#ifndef offsetof
# define offsetof(T,F) ((unsigned int)((char *)&((T *)0)->F))
#endif

#define restrict

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 0)
#    define always_inline __attribute__((always_inline)) inline
#else
#    define always_inline inline
#endif

#ifndef EMULATE_INTTYPES
#   include <inttypes.h>
#else
    typedef signed char  int8_t;
    typedef signed short int16_t;
    typedef signed int   int32_t;
    typedef unsigned char  uint8_t;
    typedef unsigned short uint16_t;
    typedef unsigned int   uint32_t;

    typedef signed long long   int64_t;
    typedef unsigned long long uint64_t;
#endif /* HAVE_INTTYPES_H */

#ifndef int64_t_C
#define int64_t_C(c)     (c ## LL)
#define uint64_t_C(c)    (c ## ULL)
#endif

#include "bswap.h"

#    if defined(__MINGW32__) || defined(__CYGWIN__) || \
        defined(__OS2__) || (defined (__OpenBSD__) && !defined(__ELF__))
#        define MANGLE(a) "_" #a
#    else
#        define MANGLE(a) #a
#    endif

#define NEG_USR32(a,s) (((uint32_t)(a))>>(32-(s)))

/* bit output */

struct PutBitContext;

typedef void (*WriteDataFunc)(void *, uint8_t *, int);

typedef struct PutBitContext {
#ifdef ALT_BITSTREAM_WRITER
    uint8_t *buf, *buf_end;
    int index;
#else
    uint32_t bit_buf;
    int bit_left;
    uint8_t *buf, *buf_ptr, *buf_end;
#endif
} PutBitContext;

void init_put_bits(PutBitContext *s, uint8_t *buffer, int buffer_size);

int get_bit_count(PutBitContext *s); /* XXX: change function name */
void align_put_bits(PutBitContext *s);
void flush_put_bits(PutBitContext *s);
void put_string(PutBitContext * pbc, char *s);

#if 0
/* bit input */

typedef struct GetBitContext {
    const uint8_t *buffer, *buffer_end;
#ifdef ALT_BITSTREAM_READER
    int index;
#elif defined LIBMPEG2_BITSTREAM_READER
    uint8_t *buffer_ptr;
    uint32_t cache;
    int bit_count;
#elif defined A32_BITSTREAM_READER
    uint32_t *buffer_ptr;
    uint32_t cache0;
    uint32_t cache1;
    int bit_count;
#endif
    int size_in_bits;
} GetBitContext;

static inline int get_bits_count(GetBitContext *s);

/* used to avoid missaligned exceptions on some archs (alpha, ...) */
#ifdef ARCH_X86
#    define unaligned32(a) (*(uint32_t*)(a))
#else
#    ifdef __GNUC__
static inline uint32_t unaligned32(const void *v) {
    struct Unaligned {
	uint32_t i;
    } __attribute__((packed));

    return ((const struct Unaligned *) v)->i;
}
#    else
static inline uint32_t unaligned32(const void *v) {
    return *(const uint32_t *) v;
}
#    endif
#endif //!ARCH_X86
#endif

#ifndef ALT_BITSTREAM_WRITER
static inline void put_bits(PutBitContext *s, int n, unsigned int value)
{
    unsigned int bit_buf;
    int bit_left;

#ifdef STATS
    st_out_bit_counts[st_current_index] += n;
#endif
    //    printf("put_bits=%d %x\n", n, value);
    //assert(n == 32 || value < (1U << n));
    
    bit_buf = s->bit_buf;
    bit_left = s->bit_left;

    //    printf("n=%d value=%x cnt=%d buf=%x\n", n, value, bit_cnt, bit_buf);
    /* XXX: optimize */
    if (n < bit_left) {
        bit_buf = (bit_buf<<n) | value;
        bit_left-=n;
    } else {
	bit_buf<<=bit_left;
        bit_buf |= value >> (n - bit_left);
#ifdef UNALIGNED_STORES_ARE_BAD
        if (3 & (int) s->buf_ptr) {
            s->buf_ptr[0] = bit_buf >> 24;
            s->buf_ptr[1] = bit_buf >> 16;
            s->buf_ptr[2] = bit_buf >>  8;
            s->buf_ptr[3] = bit_buf      ;
        } else
#endif
        *(uint32_t *)s->buf_ptr = be2me_32(bit_buf);
        //printf("bitbuf = %08x\n", bit_buf);
        s->buf_ptr+=4;
	bit_left+=32 - n;
        bit_buf = value;
    }

    s->bit_buf = bit_buf;
    s->bit_left = bit_left;
}
#endif


#ifdef ALT_BITSTREAM_WRITER
static inline void put_bits(PutBitContext *s, int n, unsigned int value)
{
#    ifdef ALIGNED_BITSTREAM_WRITER
#        ifdef ARCH_X86
    asm volatile(
	"movl %0, %%ecx			\n\t"
	"xorl %%eax, %%eax		\n\t"
	"shrdl %%cl, %1, %%eax		\n\t"
	"shrl %%cl, %1			\n\t"
	"movl %0, %%ecx			\n\t"
	"shrl $3, %%ecx			\n\t"
	"andl $0xFFFFFFFC, %%ecx	\n\t"
	"bswapl %1			\n\t"
	"orl %1, (%2, %%ecx)		\n\t"
	"bswapl %%eax			\n\t"
	"addl %3, %0			\n\t"
	"movl %%eax, 4(%2, %%ecx)	\n\t"
	: "=&r" (s->index), "=&r" (value)
	: "r" (s->buf), "r" (n), "0" (s->index), "1" (value<<(-n))
	: "%eax", "%ecx"
    );
#        else
    int index= s->index;
    uint32_t *ptr= ((uint32_t *)s->buf)+(index>>5);
    
    value<<= 32-n; 
    
    ptr[0] |= be2me_32(value>>(index&31));
    ptr[1]  = be2me_32(value<<(32-(index&31)));
//if(n>24) printf("%d %d\n", n, value);
    index+= n;
    s->index= index;
#        endif
#    else //ALIGNED_BITSTREAM_WRITER
#        ifdef ARCH_X86
    asm volatile(
	"movl $7, %%ecx			\n\t"
	"andl %0, %%ecx			\n\t"
	"addl %3, %%ecx			\n\t"
	"negl %%ecx			\n\t"
	"shll %%cl, %1			\n\t"
	"bswapl %1			\n\t"
	"movl %0, %%ecx			\n\t"
	"shrl $3, %%ecx			\n\t"
	"orl %1, (%%ecx, %2)		\n\t"
	"addl %3, %0			\n\t"
	"movl $0, 4(%%ecx, %2)		\n\t"
	: "=&r" (s->index), "=&r" (value)
	: "r" (s->buf), "r" (n), "0" (s->index), "1" (value)
	: "%ecx"
    );
#        else
    int index= s->index;
    uint32_t *ptr= (uint32_t*)(((uint8_t *)s->buf)+(index>>3));
    
    ptr[0] |= be2me_32(value<<(32-n-(index&7) ));
    ptr[1] = 0;
//if(n>24) printf("%d %d\n", n, value);
    index+= n;
    s->index= index;
#        endif
#    endif //!ALIGNED_BITSTREAM_WRITER
}
#endif


static inline uint8_t* pbBufPtr(PutBitContext *s)
{
#ifdef ALT_BITSTREAM_WRITER
	return s->buf + (s->index>>3);
#else
	return s->buf_ptr;
#endif
}

/* misc math functions */
extern const uint8_t ff_log2_tab[256];

static inline int av_log2(unsigned int v)
{
    int n;

    n = 0;
    if (v & 0xffff0000) {
        v >>= 16;
        n += 16;
    }
    if (v & 0xff00) {
        v >>= 8;
        n += 8;
    }
    n += ff_log2_tab[v];

    return n;
}

static inline int av_log2_16bit(unsigned int v)
{
    int n;

    n = 0;
    if (v & 0xff00) {
        v >>= 8;
        n += 8;
    }
    n += ff_log2_tab[v];

    return n;
}


/* median of 3 */
static inline int mid_pred(int a, int b, int c)
{
    int vmin, vmax;
    vmax = vmin = a;
    if (b < vmin)
        vmin = b;
    else
	vmax = b;

    if (c < vmin)
        vmin = c;
    else if (c > vmax)
        vmax = c;

    return a + b + c - vmin - vmax;
}

static inline int clip(int a, int amin, int amax)
{
    if (a < amin)
        return amin;
    else if (a > amax)
        return amax;
    else
        return a;
}

/* math */
extern const uint8_t ff_sqrt_tab[128];

int64_t ff_gcd(int64_t a, int64_t b);

static inline int ff_sqrt(int a)
{
    int ret=0;
    int s;
    int ret_sq=0;
    
    if(a<128) return ff_sqrt_tab[a];
    
    for(s=15; s>=0; s--){
        int b= ret_sq + (1<<(s*2)) + (ret<<s)*2;
        if(b<=a){
            ret_sq=b;
            ret+= 1<<s;
        }
    }
    return ret;
}
/* mpeg audio declarations for both encoder and decoder. */

/* max frame size, in samples */
#define MPA_FRAME_SIZE 1152 

/* max compressed frame size */
#define MPA_MAX_CODED_FRAME_SIZE 1792

#define MPA_MAX_CHANNELS 2

#define SBLIMIT 32 /* number of subbands */

#define MPA_STEREO  0
#define MPA_JSTEREO 1
#define MPA_DUAL    2
#define MPA_MONO    3

int l2_select_table(int bitrate, int nb_channels, int freq, int lsf);

extern const uint16_t mpa_bitrate_tab[2][3][15];
extern const uint16_t mpa_freq_tab[3];
extern const unsigned char *alloc_tables[5];
extern const double enwindow[512];
extern const int sblimit_table[5];
extern const int quant_steps[17];
extern const int quant_bits[17];
extern const int32_t mpa_enwindow[257];

/*
 * mpeg audio layer 2 tables. Most of them come from the mpeg audio
 * specification.
 * 
 * Copyright (c) 2000, 2001 Fabrice Bellard.
 *
 * The licence of this code is contained in file LICENCE found in the
 * same archive 
 */

#define SQRT2 1.41421356237309514547

/* currently, cannot change these constants (need to modify
   quantization stage) */
#define FRAC_BITS 15
#define WFRAC_BITS  14
#define MUL(a,b) (((int64_t)(a) * (int64_t)(b)) >> FRAC_BITS)
#define FIX(a)   ((int)((a) * (1 << FRAC_BITS)))

static const int costab32[30] = {
    FIX(0.54119610014619701222),
    FIX(1.3065629648763763537),
    
    FIX(0.50979557910415917998),
    FIX(2.5629154477415054814),
    FIX(0.89997622313641556513),
    FIX(0.60134488693504528634),
    
    FIX(0.5024192861881556782),
    FIX(5.1011486186891552563),
    FIX(0.78815462345125020249),
    FIX(0.64682178335999007679),
    FIX(0.56694403481635768927),
    FIX(1.0606776859903470633),
    FIX(1.7224470982383341955),
    FIX(0.52249861493968885462),
    
    FIX(10.19000812354803287),
    FIX(0.674808341455005678),
    FIX(1.1694399334328846596),
    FIX(0.53104259108978413284),
    FIX(2.0577810099534108446),
    FIX(0.58293496820613388554),
    FIX(0.83934964541552681272),
    FIX(0.50547095989754364798),
    FIX(3.4076084184687189804),
    FIX(0.62250412303566482475),
    FIX(0.97256823786196078263),
    FIX(0.51544730992262455249),
    FIX(1.4841646163141661852),
    FIX(0.5531038960344445421),
    FIX(0.74453627100229857749),
    FIX(0.5006029982351962726),
};

static const int bitinv32[32] = {
    0,  16,  8, 24,  4,  20,  12,  28,
    2,  18, 10, 26,  6,  22,  14,  30,
    1,  17,  9, 25,  5,  21,  13,  29,
    3,  19, 11, 27,  7,  23,  15,  31
};


static int16_t filter_bank[512];

static int scale_factor_table[64];
#ifdef USE_FLOATS
static float scale_factor_inv_table[64];
#else
static int8_t scale_factor_shift[64];
static unsigned short scale_factor_mult[64];
#endif
static unsigned char scale_diff_table[128];

/* total number of bits per allocation group */
static unsigned short total_quant_bits[17];

/* signal to noise ratio of each quantification step (could be
   computed from quant_steps[]). The values are dB multiplied by 10 
*/
static unsigned short quant_snr[17] = { 
     70, 110, 160, 208,
    253, 316, 378, 439,
    499, 559, 620, 680, 
    740, 800, 861, 920, 
    980
};

/* fixed psycho acoustic model. Values of SNR taken from the 'toolame'
   project */
static const float fixed_smr[SBLIMIT] =  {
    30, 17, 16, 10, 3, 12, 8, 2.5,
    5, 5, 6, 6, 5, 6, 10, 6,
    -4, -10, -21, -30, -42, -55, -68, -75,
    -75, -75, -75, -75, -91, -107, -110, -108
};

static const unsigned char nb_scale_factors[4] = { 3, 2, 1, 2 };

#define SAMPLES_BUF_SIZE 4096

typedef struct MpegAudioContext {
    PutBitContext pb;
    int nb_channels;
    int freq, bit_rate;
    int lsf;           /* 1 if mpeg2 low bitrate selected */
    int bitrate_index; /* bit rate */
    int freq_index;
    int frame_size; /* frame size, in bits, without padding */
    /* padding computation */
    int frame_frac, frame_frac_incr, do_padding;
    short samples_buf[MPA_MAX_CHANNELS][SAMPLES_BUF_SIZE]; /* buffer for filter */
    int samples_offset[MPA_MAX_CHANNELS];       /* offset in samples_buf */
    int sb_samples[MPA_MAX_CHANNELS][3][12][SBLIMIT];
    unsigned char scale_factors[MPA_MAX_CHANNELS][SBLIMIT][3]; /* scale factors */
    /* code to group 3 scale factors */
    unsigned char scale_code[MPA_MAX_CHANNELS][SBLIMIT];       
    int sblimit; /* number of used subbands */
    const unsigned char *alloc_table;
} MpegAudioContext;

