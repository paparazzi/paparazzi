/*
 * Copyright (C) 2012 Gerard Toonstra
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/datalink/w5100.c
 * W5100 ethernet chip I/O
 */

#include "mcu_periph/sys_time.h"
#include "modules/datalink/w5100.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

#define TXBUF_BASE 0x4000
#define RXBUF_BASE 0x6000
#define SOCKETS 4

#define CMD_SOCKET 1
#define TELEM_SOCKET 0

#define SOCK_OPEN 0x01
#define SOCK_CLOSE 0x10
#define SOCK_SEND 0x20
#define SOCK_RECV 0x40
#define SNMR_UDP 0x02
#define SNMR_MULTI 0x80
#define SNIR_SEND_OK 0x10
#define SNIR_TIMEOUT 0x08
#define CH_BASE 0x0400
#define CH_SIZE 0x0100
#define SMASK 0x07FF // Tx buffer MASK
#define RMASK 0x07FF // Tx buffer MASK

#define REG_MR 0x0000
#define REG_RX_MEM 0x001A
#define REG_TX_MEM 0x001B

#define REG_GAR 0x0001
#define REG_SUBR 0x0005
#define REG_SHAR 0x0009
#define REG_SIPR 0x000F

#define SOCK_MR 0x0000
#define SOCK_CR 0x0001
#define SOCK_IR 0x0002
#define SOCK_PORT 0x0004
#define SOCK_DHAR 0x0006
#define SOCK_DIPR 0x000C
#define SOCK_DPORT 0x0010
#define SOCK_TX_WR 0x0024
#define SOCK_RSR 0x0026
#define SOCK_RXRD 0x0028

#ifndef W5100_SPI_DEV
#define W5100_SPI_DEV spi1
#endif

#ifndef W5100_SLAVE_IDX
#define W5100_SLAVE_IDX SPI_SLAVE1
#endif

#ifndef W5100_DRDY_GPIO
#define W5100_DRDY_GPIO GPIOB
#endif

#ifndef W5100_DRDY_GPIO_PIN
#define W5100_DRDY_GPIO_PIN GPIO1
#endif

struct pprz_w5100_tp;
struct w5100_periph chip0;
uint8_t ck_a, ck_b;
uint8_t w5100_rx_buf[W5100_RX_BUFFER_SIZE];

// the media access control (ethernet hardware) address for the shield.
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

//the IP address for the shield:
static uint8_t ip[] = { W5100_IP };
static uint8_t dest[] = { W5100_MULTICAST_IP };
static uint8_t subnet[] = { W5100_SUBNET };
static uint16_t dport = W5100_MULTICAST_PORT;

static const uint8_t RST = 7; // Reset BIT

uint16_t SBASE[SOCKETS]; // Tx buffer base address
uint16_t RBASE[SOCKETS]; // Rx buffer base address
static const uint16_t SSIZE = 2048; // Max Tx buffer size
static const uint16_t RSIZE = 2048; // Max Rx buffer size

struct spi_transaction w5100_spi;

static void w5100_close_socket(uint8_t _s);
static void configure_socket(uint8_t _s, uint8_t _flag, uint16_t _lport, uint16_t _dport, uint8_t *_dest);
static void w5100_read_data(uint8_t s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len);
static uint16_t w5100_read(uint16_t _addr, uint8_t *_buf, uint16_t _len);

static inline void w5100_set(uint16_t _reg, uint8_t _val)
{
  w5100_spi.output_buf[0] = 0xF0;
  w5100_spi.output_buf[1] = _reg >> 8;
  w5100_spi.output_buf[2] = _reg & 0xFF;
  w5100_spi.output_buf[3] = _val;

  spi_submit(&(W5100_SPI_DEV), &w5100_spi);

  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while (w5100_spi.status != SPITransSuccess);
}

static inline uint8_t w5100_get(uint16_t _reg)
{
  w5100_spi.output_buf[0] = 0x0F;
  w5100_spi.output_buf[1] = _reg >> 8;
  w5100_spi.output_buf[2] = _reg & 0xFF;

  spi_submit(&(W5100_SPI_DEV), &w5100_spi);

  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while (w5100_spi.status != SPITransSuccess);

  return w5100_spi.input_buf[3];
}

static inline void w5100_set_buffer(uint16_t _reg, volatile uint8_t *_buf, uint16_t _len)
{
  for (int i = 0; i < _len; i++) {
    w5100_set(_reg, _buf[ i ]);
    _reg++;
  }
}

static inline void w5100_sock_set(uint8_t _sock, uint16_t _reg, uint8_t _val)
{
  w5100_set(CH_BASE + _sock * CH_SIZE + _reg, _val);
}

static inline uint8_t w5100_sock_get(uint8_t _sock, uint16_t _reg)
{
  return w5100_get(CH_BASE + _sock * CH_SIZE + _reg);
}

static inline uint16_t w5100_sock_get16(uint8_t _sock, uint16_t _reg)
{
  uint16_t res = w5100_sock_get(_sock, _reg);
  uint16_t res2 = w5100_sock_get(_sock, _reg + 1);
  res = res << 8;
  res2 = res2 & 0xFF;
  res = res | res2;
  return res;
}

// Functions for the generic device API
static int true_function(struct w5100_periph *p __attribute__((unused)), long *fd __attribute__((unused)), uint16_t len __attribute__((unused))) { return true; }
static void dev_transmit(struct w5100_periph *p __attribute__((unused)), long fd __attribute__((unused)), uint8_t byte) {  w5100_transmit(byte); }
static void dev_transmit_buffer(struct w5100_periph *p __attribute__((unused)), long fd __attribute__((unused)), uint8_t *data, uint16_t len) {  w5100_transmit_buffer(data, len); }
static void dev_send(struct w5100_periph *p __attribute__((unused)), long fd __attribute__((unused))) { w5100_send(); }
static int dev_char_available(struct w5100_periph *p __attribute__((unused))) { return w5100_ch_available; }
static uint8_t dev_getch(struct w5100_periph *p __attribute__((unused)))
{
  uint8_t c = 0;
  w5100_receive(&c, 1);
  return c;
}

void w5100_init(void)
{

  // configure the SPI bus.
  w5100_spi.slave_idx = W5100_SLAVE_IDX;
  w5100_spi.output_length = 4;
  w5100_spi.input_length = 4;
  w5100_spi.select = SPISelectUnselect;
  w5100_spi.cpol = SPICpolIdleLow;
  w5100_spi.cpha = SPICphaEdge1;
  w5100_spi.dss = SPIDss8bit;
  w5100_spi.bitorder = SPIMSBFirst;
  w5100_spi.cdiv = SPIDiv64;

  chip0.status = W5100StatusUninit;
  chip0.curbuf = 0;
  w5100_spi.input_buf = &chip0.work_rx[0];
  w5100_spi.output_buf = &chip0.work_tx[0];

  // wait one second for proper initialization (chip getting powered up).
  sys_time_usleep(1000000);

  // set DRDY pin
  gpio_setup_output(W5100_DRDY_GPIO, W5100_DRDY_GPIO_PIN);
  gpio_clear(W5100_DRDY_GPIO, W5100_DRDY_GPIO_PIN);
  sys_time_usleep(200);
  gpio_set(W5100_DRDY_GPIO, W5100_DRDY_GPIO_PIN);

  // allow some time for the chip to wake up.
  sys_time_usleep(20000);

  // write reset bit into mode register
  w5100_set(REG_MR, 1 << RST);

  // allow some time to wake up...
  sys_time_usleep(20000);

  // receive memory size
  w5100_set(REG_RX_MEM, 0x55);

  // transmit memory size
  w5100_set(REG_TX_MEM, 0x55);

  // Setting the required socket base addresses for reads and writes to/from sockets
  for (int i = 0; i < SOCKETS; i++) {
    SBASE[i] = TXBUF_BASE + SSIZE * i;
    RBASE[i] = RXBUF_BASE + RSIZE * i;
  }

  uint8_t gateway[4];
  gateway[0] = ip[0];
  gateway[1] = ip[1];
  gateway[2] = ip[2];
  gateway[3] = 1;

  // configure gateway, subnet, mac and ip on "NIC".
  w5100_set_buffer(REG_GAR, gateway, 4);
  w5100_set_buffer(REG_SUBR, subnet, 4);
  w5100_set_buffer(REG_SHAR, mac, 6);
  w5100_set_buffer(REG_SIPR, ip, 4);

  // create a socket to send telemetry through.
  configure_socket(TELEM_SOCKET, SNMR_MULTI, 1, dport, dest);

  // make dest zero and configure socket to receive data
  dest[ 0 ] = 0x00;
  configure_socket(CMD_SOCKET, 0, dport, dport, dest);

  // Configure generic device
  chip0.device.periph = (void *)(&chip0);
  chip0.device.check_free_space = (check_free_space_t) true_function;
  chip0.device.put_byte = (put_byte_t) dev_transmit;
  chip0.device.put_buffer = (put_buffer_t) dev_transmit_buffer;
  chip0.device.send_message = (send_message_t) dev_send;
  chip0.device.char_available = (char_available_t) dev_char_available;
  chip0.device.get_byte = (get_byte_t) dev_getch;

  // Init PPRZ transport
  pprz_transport_init(&pprz_w5100_tp);
}

void w5100_transmit(uint8_t data)
{

  uint16_t temp = (chip0.tx_insert_idx[ chip0.curbuf ] + 1) % W5100_TX_BUFFER_SIZE;

  if (temp == chip0.tx_extract_idx[ chip0.curbuf ]) {
    // no more room in this transaction.
    return;
  }

  // check if in process of sending data
  chip0.tx_buf[ chip0.curbuf ][ chip0.tx_insert_idx[ chip0.curbuf ] ] = data;
  chip0.tx_insert_idx[ chip0.curbuf ] = temp;
}

void w5100_transmit_buffer(uint8_t *data, uint16_t len)
{
  int i;
  for (i = 0; i < len; i++) {
    w5100_transmit(data[i]);
  }
}

void w5100_send()
{
  // Now send off spi transaction.
  uint16_t len = chip0.tx_insert_idx[ chip0.curbuf ];
  uint8_t curbuf = chip0.curbuf;

  // switch to other buffer to accept more chars.
  chip0.curbuf++;
  if (chip0.curbuf >= W5100_BUFFER_NUM) {
    chip0.curbuf = 0;
  }

  uint16_t ptr = w5100_sock_get16(TELEM_SOCKET, SOCK_TX_WR);
  uint16_t offset = ptr & SMASK;
  uint16_t dstAddr = offset + SBASE[ TELEM_SOCKET ];

  chip0.tx_insert_idx[ chip0.curbuf ] = 0;
  chip0.tx_extract_idx[ chip0.curbuf ] = 0;

  if (offset + len > SSIZE) {
    // Wrap around circular buffer
    uint16_t size = SSIZE - offset;
    w5100_set_buffer(dstAddr, &chip0.tx_buf[curbuf][0], size);
    w5100_set_buffer(SBASE[ TELEM_SOCKET ], &chip0.tx_buf[curbuf][0] + size, len - size);
  } else {
    w5100_set_buffer(dstAddr, &chip0.tx_buf[curbuf][0], len);
  }

  // Reset write pointer
  ptr += len;
  w5100_sock_set(TELEM_SOCKET, SOCK_TX_WR, ptr >> 8);
  w5100_sock_set(TELEM_SOCKET, SOCK_TX_WR + 1, ptr & 0xFF);

  // send
  w5100_sock_set(TELEM_SOCKET, SOCK_CR, SOCK_SEND);

  uint8_t complete = w5100_sock_get(TELEM_SOCKET, SOCK_CR);
  while (complete != 0x00) {
    complete = w5100_sock_get(TELEM_SOCKET, SOCK_CR);
  }
}

uint16_t w5100_rx_size(uint8_t _s)
{
  uint16_t val = 0, val1 = 0;

  do {
    val1 = w5100_sock_get16(_s, SOCK_RSR);
    if (val1 != 0) {
      val = w5100_sock_get16(_s, SOCK_RSR);
    }
  } while (val != val1);
  return val;
}

static void w5100_close_socket(uint8_t _s)
{
  // send command to close socket
  w5100_sock_set(_s, SOCK_CR, SOCK_CLOSE);
  // clear interrupt registers
  w5100_sock_set(_s, SOCK_IR, 0xFF);
}

static void configure_socket(uint8_t _s, uint8_t _flag, uint16_t _lport, uint16_t _dport, uint8_t *_dest)
{
  // Configure socket that receives data on 1234
  w5100_close_socket(_s);
  // configure type of socket
  w5100_sock_set(_s, SOCK_MR, SNMR_UDP | _flag);
  // configure MSB+LSB of local port number
  w5100_sock_set(_s, SOCK_PORT, _lport >> 8);
  w5100_sock_set(_s, SOCK_PORT + 1, _lport & 0xFF);
  if (_dest[0] != 0x00) {
    // configure destination to send stuff to.
    w5100_set_buffer(CH_BASE + _s * CH_SIZE + SOCK_DIPR, _dest, 4);
  }

  // this mac corresponds to a mac for multicasting....
  uint8_t mac_multi[] = { 0x01, 0x00, 0x5E, 0x01, 0x01, 0x0B };
  w5100_set_buffer(CH_BASE + _s * CH_SIZE + SOCK_DHAR, mac_multi, 6);

  // configure destination port
  w5100_sock_set(_s, SOCK_DPORT, _dport >> 8);
  w5100_sock_set(_s, SOCK_DPORT + 1, _dport & 0xFF);

  // send command to open the socket
  w5100_sock_set(_s, SOCK_CR, SOCK_OPEN);
}

bool w5100_ch_available()
{
  if (w5100_rx_size(CMD_SOCKET) > 0) {
    return true;
  }
  return false;
}

uint16_t w5100_receive(uint8_t *buf, uint16_t len __attribute__((unused)))
{
  uint8_t head[8];
  uint16_t data_len = 0;
  uint16_t ptr = 0;

  // Get socket read pointer
  ptr = w5100_sock_get16(CMD_SOCKET, SOCK_RXRD);
  w5100_read_data(CMD_SOCKET, (uint8_t *)ptr, head, 0x08);
  ptr += 8;
  data_len = head[6];
  data_len = (data_len << 8) + head[7];

  // read data from buffer.
  w5100_read_data(CMD_SOCKET, (uint8_t *)ptr, buf, data_len);  // data copy.
  ptr += data_len;

  // update read pointer
  w5100_sock_set(CMD_SOCKET, SOCK_RXRD, ptr >> 8);
  w5100_sock_set(CMD_SOCKET, SOCK_RXRD + 1, ptr & 0xFF);

  // finalize read.
  w5100_sock_set(CMD_SOCKET, SOCK_CR, SOCK_RECV);
  return data_len;
}

static void w5100_read_data(uint8_t s __attribute__((unused)), volatile uint8_t *src, volatile uint8_t *dst,
                            uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = (uint16_t)src & RMASK;
  src_ptr = RBASE[CMD_SOCKET] + src_mask;

  if ((src_mask + len) > RSIZE) {
    size = RSIZE - src_mask;
    w5100_read(src_ptr, (uint8_t *)dst, size);
    dst += size;
    w5100_read(RBASE[CMD_SOCKET], (uint8_t *) dst, len - size);
  } else {
    w5100_read(src_ptr, (uint8_t *) dst, len);
  }
}

static uint16_t w5100_read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  for (int i = 0; i < _len; i++) {
    _buf[i] = w5100_get(_addr);
    _addr++;
  }
  return _len;
}

