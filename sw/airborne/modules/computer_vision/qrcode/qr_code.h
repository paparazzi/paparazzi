
#ifndef QR_CODE_MODULE
#define QR_CODE_MODULE


#include <stdint.h>

#include "../computer_vision/lib/vision/image.h"

extern void qrcode_periodic(void);
extern void qrscan(struct image_t *img);


#endif
