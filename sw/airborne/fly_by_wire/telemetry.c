#include "telemetry.h"
#include <avr/io.h>
uint8_t ck_a, ck_b;

#define DL_FBW_STATUS 0x04

void telemetry_send_fbw_status(uint8_t* nb_spi_err, uint8_t* rc_status, uint8_t* mode) {  if (TELEMETRY_CHECK_FREE_SPACE(7)) {
    TELEMETRY_START_MESSAGE(DL_FBW_STATUS);			    
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)(nb_spi_err));
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)(rc_status));
    TELEMETRY_PUT_1_DATA_BYTE_BY_ADDR((uint8_t*)(mode));    
    TELEMETRY_END_MESSAGE();
  }
}
