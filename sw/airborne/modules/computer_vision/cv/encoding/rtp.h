


#include <stdint.h>
#include "udp/socket.h"

void send_rtp_frame(
    struct UdpSocket *sock,             // socket
    uint8_t * Jpeg, uint32_t JpegLen,   // jpeg data
    int w, int h,                       // width and height
    uint8_t format_code,                // 0=422, 1=421
    uint8_t quality_code,               // 0-99 of 128 for custom (include
    uint8_t has_dri_header,             // Does Jpeg data include Header Info?
    uint32_t delta_t                    // time step 90kHz
);

void test_rtp_frame(struct UdpSocket *sock);
