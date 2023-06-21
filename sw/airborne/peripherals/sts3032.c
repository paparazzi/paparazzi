#include "sts3032.h"
#include "string.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/sys_time_arch.h"

#define STS_DELAY_MSG 1000	// to adjust

#define BUF_MAX_LENGHT 15
//void uart_put_buffer(struct uart_periph *p, long fd, const uint8_t *data, uint16_t len);

uint8_t buf_rx_param[255];


static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun);
static void handle_reply(struct sts3032 *sts, uint8_t id, uint8_t state, uint8_t* buf_param, uint8_t length);
static void flush_input(struct sts3032 *sts);
static uint8_t id_idx(struct sts3032 *sts, uint8_t id);

void sts3032_init(struct sts3032 *sts, struct uart_periph *periph, uint8_t* cbuf, size_t cbuf_len) {
	sts->periph = periph;
	sts->rx_state = STS3032_RX_IDLE;
	sts->nb_failed_checksum = 0;
	cir_buf_init(&sts->msg_buf, cbuf, cbuf_len);
}

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

void sts3032_enable_torque(struct sts3032 *sts, uint8_t id, uint8_t enable) {
	uint8_t buf[2];
  buf[0] = SMS_STS_TORQUE_ENABLE;
  buf[1] = enable; 
	write_buf(sts, id, buf, 2, INST_WRITE);
}

void sts3032_read_pos(struct sts3032 *sts, uint8_t id) {
	sts3032_read_mem(sts, id, SMS_STS_PRESENT_POSITION_L, 2);
}

void sts3032_read_mem(struct sts3032 *sts, uint8_t id, uint8_t addr, uint8_t len) {
	uint8_t buf[2] = {addr, len};
	write_buf(sts, id, buf, 2, INST_READ);
	sts->read_addr = addr;
}

static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun) {
  
	//flush_input(sts);
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

	int ret = cir_buf_put(&sts->msg_buf, buf, len + 6);
	if(ret != 0) {
		asm("NOP");
	}
}

void sts3032_event(struct sts3032 *sts) {
	
	uint32_t now = get_sys_time_usec();
	if(now - sts->time_last_msg > STS_DELAY_MSG) {
		uint8_t buf[BUF_MAX_LENGHT];
		int size = cir_buf_get(&sts->msg_buf, buf, BUF_MAX_LENGHT);
		if(size > 0) {
			uart_put_buffer(sts->periph, 0, buf, size);
			sts->time_last_msg = now;
			sts->echo = true;
		}
	}
	
	switch (sts->rx_state)
	{
	case STS3032_RX_IDLE:
		if(uart_char_available(sts->periph) > 0){
			sts->buf_header[1] = sts->buf_header[0];
			sts->buf_header[0] = uart_getch(sts->periph);
			if(sts->buf_header[0] == 0xFF && sts->buf_header[1]== 0xFF) {
				sts->rx_state = STS3032_RX_HEAD_OK;
				sts->nb_bytes_expected = 2;
				sts->buf_header[0] = 0;
				sts->buf_header[1] = 0;
			}
		}
		break;
	case STS3032_RX_HEAD_OK:
		if(uart_char_available(sts->periph) >= sts->nb_bytes_expected){
			sts->rx_id = uart_getch(sts->periph);
			sts->nb_bytes_expected = uart_getch(sts->periph); //length
			sts->rx_checksum = sts->rx_id + sts->nb_bytes_expected;
			sts->rx_state = STS3032_RX_GOT_LENGTH;
		}
		break;
	case STS3032_RX_GOT_LENGTH:
		if(uart_char_available(sts->periph) >= sts->nb_bytes_expected){
			uint8_t current_state = uart_getch(sts->periph);
			sts->rx_checksum += current_state;
			for(int i = 0; i < sts->nb_bytes_expected-2; i++) {
				buf_rx_param[i] = uart_getch(sts->periph);
				sts->rx_checksum += buf_rx_param[i];
			}
			uint8_t checksum = uart_getch(sts->periph);
			sts->rx_checksum = ~sts->rx_checksum;
			if(sts->rx_checksum == checksum) {
				if(!sts->echo) {
					handle_reply(sts, sts->rx_id, current_state, buf_rx_param, sts->nb_bytes_expected-2);
				}
			}
			else {
			  sts->nb_failed_checksum++;
			}
			sts->echo = false;
			sts->rx_state = STS3032_RX_IDLE;
		}
		break;
	default:
		break;
	}
}

static void handle_reply(struct sts3032 *sts, uint8_t id, uint8_t state, uint8_t* buf_param, uint8_t length) {
	uint8_t index = id_idx(sts, id);
	if(index == 255) {
		return;
	}
	sts->states[index] = state; 
	uint8_t offset = 0;
	while(length > offset) {
		switch (sts->read_addr + offset)
		{
		case SMS_STS_PRESENT_POSITION_L:
			if(offset + 2 < length) {
				return;
			}
			sts->pos[index] =  buf_param[offset] | (buf_param[offset + 1] << 8);
			offset += 2;
			break;
		
		default:
			break;
		}
	}
}

// static void flush_input(struct sts3032 *sts) {
// 	while(uart_char_available(sts->periph)){
// 		uart_getch(sts->periph);
// 	}
// }

static uint8_t id_idx(struct sts3032 *sts, uint8_t id) {
	for(int i = 0; i<STS3032_NB_SERVO; i++) {
		if(sts->ids[i] == id){
			return i;
		}
	}
	return 255;
}
