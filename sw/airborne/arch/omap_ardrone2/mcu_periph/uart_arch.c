/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "fms/fms_serial_port.h"


void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud, bool_t hw_flow_control __attribute__ ((unused))) {
	int fd = 0;
	struct termios tio;

	// Close port if already open
	if (p->reg_addr != NULL) {
		fd = (int)(p->reg_addr);
		close(fd);
	}

	// Open serial port
	fd = open(p->dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Set baud rate
	tio.c_iflag= IGNBRK;
	tio.c_oflag= 0;
	tio.c_cflag= CS8 | CLOCAL | CREAD;
	tio.c_cflag|= baud;
	tio.c_lflag= 0;
	tio.c_cc[VTIME]= 0;
	tio.c_cc[VMIN]= 1;
	tcsetattr(fd, TCSANOW, &tio);

	// uUe register address to store file pointer
	p->reg_addr = (void*)fd;
}

void uart_transmit(struct uart_periph* p, uint8_t data ) {
	uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

	if (temp == p->tx_extract_idx)
		return;                          // no room

	// check if in process of sending data
	if (p->tx_running) { // yes, add to queue
		p->tx_buf[p->tx_insert_idx] = data;
		p->tx_insert_idx = temp;
	}
	else { // no, set running flag and write to output register
		p->tx_running = TRUE;
		int fd = (int)(p->reg_addr);
		write(fd, &data, 1);
		//printf("w %x\n",data);
	}
}

bool_t uart_recieve(struct uart_periph* p) {
	//Check if device is initialized
	if(p->reg_addr == NULL)
		return 0;

	int fd = (int)p->reg_addr;

	// check if more data to send
	if (p->tx_insert_idx != p->tx_extract_idx) {
		write(fd, &(p->tx_buf[p->tx_extract_idx]), 1);
		//printf("w %x\n",p->tx_buf[p->tx_extract_idx]);
		p->tx_extract_idx++;
		p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
	}
	else {
		p->tx_running = FALSE;   // clear running flag
	}

	//Read data
	unsigned char c = 0x0;
	if(read(fd, &c,1) > 0){
		//printf("r %x %c\n",c,c);
		uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
		p->rx_buf[p->rx_insert_idx] = c;
		// check for more room in queue
		if (temp != p->rx_extract_idx)
			p->rx_insert_idx = temp; // update insert index
	}

	return p->rx_insert_idx != p->rx_extract_idx;
}

#ifdef USE_UART0
void uart0_init( void ) {
	uart_periph_init(&uart0);
	strcpy(uart0.dev, UART0_DEV);
	UART0SetBaudrate(UART0_BAUD);
}
#endif

#ifdef USE_UART1
void uart1_init( void ) {
	uart_periph_init(&uart1);
	strcpy(uart1.dev, UART1_DEV);
	UART1SetBaudrate(UART1_BAUD);
}
#endif

