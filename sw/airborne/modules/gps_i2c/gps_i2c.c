#include "gps_i2c.h"
#include "i2c.h"

uint8_t gps_i2c_buf[GPS_I2C_BUF_SIZE];
uint8_t gps_i2c_insert_idx, gps_i2c_extract_idx;


/* u-blox5 protocole, page 4 */
#define GPS_I2C_SLAVE_ADDR 0x42
#define GPS_I2C_ADDR_NB_AVAIL_BYTES 0xFD
#define GPS_I2C_ADDR_DATA 0xFF

#define GPS_I2C_STATUS_IDLE                   0
#define GPS_I2C_STATUS_ASKING_DATA            1
#define GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES  2
#define GPS_I2C_STATUS_READING_NB_AVAIL_BYTES 3
#define GPS_I2C_STATUS_READING_DATA           4

static uint8_t gps_i2c_status;
static bool_t gps_i2c_done;
static uint8_t gps_i2c_nb_avail_bytes;
static uint8_t data_buf_len;

void
gps_i2c_init(void) {
  gps_i2c_status = GPS_I2C_STATUS_IDLE;
  gps_i2c_done = TRUE;
}

static inline void
gps_i2c_ask_read(uint8_t addr) {
  i2c0_buf[0] = addr;
  i2c0_transmit(GPS_I2C_SLAVE_ADDR, 1, &gps_i2c_done);
  gps_i2c_done = FALSE;
}

static inline void
continue_reading(void) {
  if (gps_i2c_nb_avail_bytes > 0) {
    gps_i2c_ask_read(GPS_I2C_ADDR_DATA);
    gps_i2c_status = GPS_I2C_STATUS_ASKING_DATA;
  } else {
    gps_i2c_status = GPS_I2C_STATUS_IDLE;
  }
}

void
gps_i2c_periodic(void) {
  // We should check gps_i2c_done == true
  gps_i2c_ask_read(GPS_I2C_ADDR_NB_AVAIL_BYTES);
  gps_i2c_status = GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES;
}

void
gps_i2c_event(void) {
  if (gps_i2c_done) {
    switch (gps_i2c_status) {
    case GPS_I2C_STATUS_ASKING_NB_AVAIL_BYTES:
      i2c0_receive(GPS_I2C_SLAVE_ADDR, 2, &gps_i2c_done);
      gps_i2c_done = FALSE;
      gps_i2c_status = GPS_I2C_STATUS_READING_NB_AVAIL_BYTES;
      break;

    case GPS_I2C_STATUS_READING_NB_AVAIL_BYTES:
      gps_i2c_nb_avail_bytes = (i2c0_buf[0]<<8) | i2c0_buf[1];
      continue_reading();
      break;

    case GPS_I2C_STATUS_ASKING_DATA:
      data_buf_len = Min(gps_i2c_nb_avail_bytes, I2C0_BUF_LEN);
      gps_i2c_nb_avail_bytes -= data_buf_len;

      i2c0_receive(GPS_I2C_SLAVE_ADDR, data_buf_len, &gps_i2c_done);
      gps_i2c_done = FALSE;
      gps_i2c_status = GPS_I2C_STATUS_READING_DATA;
      break;
      
    case GPS_I2C_STATUS_READING_DATA: {
      uint8_t i;
      for(i = 0; i < data_buf_len; i++) {
	gps_i2c_AddCharToBuf(i2c0_buf[i]);
      }
      continue_reading();
      break;
    }
    }
  }
}
