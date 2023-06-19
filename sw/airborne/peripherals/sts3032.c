#include "sts3032.h"
#include "string.h"
#include "mcu_periph/uart.h"

#define BUF_MAX_LENGHT 15
//void uart_put_buffer(struct uart_periph *p, long fd, const uint8_t *data, uint16_t len);


static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun);

void sts3032_write_pos(struct sts3032 *sts, uint8_t id, int16_t position){
  if (position < 0)
	{
		position = -position;
		position |= (1 << 15);
	}
  uint8_t buf[3];
  buf[0] = SMS_STS_GOAL_POSITION_L;
  buf[1] = position & 0xff; 
  buf[2] = position >> 8;
  write_buf(sts, id, buf, 3, INST_WRITE);
}

static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun) {
  
	uint8_t buf[BUF_MAX_LENGHT];
	uint8_t checksum = id + fun + len + 2;
	buf[0] = 0xff;
	buf[1] = 0xff;
	buf[2] = id;
  buf[3] = 2 + len;
	buf[4] = fun;
  memcpy(&buf[5], data, len);
  for (int i = 0; i < len; i++){
		checksum += data[i];
	}
  buf[len + 5] = ~checksum;
  uart_put_buffer(sts->periph, 0, buf, len + 6);
}

