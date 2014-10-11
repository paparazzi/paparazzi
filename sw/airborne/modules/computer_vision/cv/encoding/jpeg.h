
#ifndef __MY_JPEG_HEADER__
#define __MY_JPEG_HEADER__

#include <stdint.h>

#define        FOUR_ZERO_ZERO          0
#define        FOUR_TWO_ZERO           1
#define        FOUR_TWO_TWO            2
#define        FOUR_FOUR_FOUR          3
#define        RGB                     4

unsigned char *encode_image (
    uint8_t* in,
    uint8_t* out,
    uint32_t q,                       // image quality 1-8
    uint32_t fmt,                     // image format code
    uint32_t width,                   // image width
    uint32_t height,                  // image height
    uint8_t add_dri_header            // data only or full jpeg file
);

unsigned char *encode_image_rtp (
    uint8_t* in,
    uint8_t* out,
    uint32_t q,                       // image quality 1-8
    uint32_t fmt,                     // image format code
    uint32_t width,                   // image width
    uint32_t height,                  // image height
    uint8_t add_dri_header            // data only or full jpeg file
);

unsigned char *encode_image_std_qt (
    uint8_t* in,
    uint8_t* out,
    uint32_t q,                       // image quality 0 to 99
    uint32_t fmt,                     // image format code
    uint32_t width,                   // image width
    uint32_t height,                  // image height
    uint8_t add_dri_header            // data only or full jpeg file
);

int create_svs_jpeg_header(unsigned char* buf, int32_t size, int w);

#endif
