#ifndef PPM_TELEMETRY_H
#define PPM_TELEMETRY_H


#include BOARD_CONFIG


static inline void ppm_init ( void ) {
  ppm_valid = FALSE;
}

#define PPM_ISR() {}

#define PPM_IT 0x00

void ppm_datalink( uint8_t throttle_mode,
                   int8_t roll,
                   int8_t pitch)
;

#endif /* PPM_TELEMETRY_H */
