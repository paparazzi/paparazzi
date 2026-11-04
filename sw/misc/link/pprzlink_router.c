/*
 * Paparazzi link router
 *
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*
./pprzlink_router_cc -p -e uart:///dev/ttyS2:57600 -e udp://192.168.0.10:5243:192.168.0.11:5242
./pprzlink_router_cc -p -e udp://192.168.0.11:5242:192.168.0.10:5243 -e udp://0.0.0.0:4243:192.168.43.255:4242

The first endpoint faces the aircraft: what it receives is downlink, what the other endpoints receive is uplink.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sched.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/syscall.h>

#define UDP_BUFFER_SIZE   1024
#define UDP_SNDBUF        32768   ///< Small send buffer: on a slow link, drop frames instead of delaying them
#define MAX_ENDPOINTS     32
#define PPRZ_STX          0x99
#define PPRZ_CLASS_TELEMETRY  1
#define LINK_ROUTER_REPORT_ID 51  ///< LINK_ROUTER_REPORT in the telemetry class
#define VALUE_UNKNOWN     127     ///< RSSI or SNR not available
#define RSSI_TIMEOUT_MS   5000    ///< RSSI from stdin older than this is not reported

/* Parser for pprzlink v2 frames: STX, LEN (whole frame), payload, CK_A, CK_B */
struct pprz_parser_t {
  uint8_t status;
  uint8_t len;
  uint8_t idx;
  uint8_t ck_a;
  uint8_t ck_b;
  uint8_t frame[256];
};

struct endpoint_udp_t {
  char server_addr[128];
  uint16_t server_port;
  char client_addr[128];
  uint16_t client_port;
};

struct endpoint_uart_t {
  char devname[512];
  int baudrate;
};

enum endpoint_type_t {
  ENDPOINT_TYPE_NONE,
  ENDPOINT_TYPE_UDP,
  ENDPOINT_TYPE_UART
};

/* Direction of a frame: received on the first endpoint (the aircraft side) is downlink */
enum direction_t {
  DIR_DOWN = 0,
  DIR_UP = 1
};

struct endpoint_t {
  enum endpoint_type_t type;
  int idx;
  int fd;
  pthread_t thread;
  struct pprz_parser_t parser;
  union {
    struct endpoint_udp_t udp;
    struct endpoint_uart_t uart;
  } ep;
};

/* Statistics of one direction, sent in LINK_ROUTER_REPORT */
struct dir_stats_t {
  uint32_t rx_msgs;
  uint32_t rx_err;
  uint32_t tx_msgs;
  uint32_t tx_err;
  uint32_t dropped;
  uint64_t last_rx_ms;  ///< time of the last received frame, 0 if none yet
  uint32_t max_gap_ms;  ///< longest time between two frames since the last report
};

enum node_t {
  NODE_AIR = 0,
  NODE_GROUND = 1,
  NODE_AUTO
};

static bool verbose = false;
static bool keep_awake = true;
static bool pprz_frames = false;
static int stats_period_ms = 1000;
static enum node_t node = NODE_AUTO;
static struct endpoint_t endpoints[MAX_ENDPOINTS];
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t stats_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct dir_stats_t stats[2];
static int ac_id = -1;  ///< learned from the downlink frames
static int8_t rssi = VALUE_UNKNOWN, snr = VALUE_UNKNOWN;
static uint64_t rssi_time_ms = 0;

void packet_handler(struct endpoint_t *src, uint8_t *data, uint16_t len);
void input_handler(struct endpoint_t *ep, uint8_t *data, uint16_t len);

/**
 * Monotonic time in milliseconds
 */
static uint64_t time_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static const char *endpoint_name(struct endpoint_t *ep, char *buf, size_t size) {
  if(ep->type == ENDPOINT_TYPE_UDP)
    snprintf(buf, size, "%.100s:%d", ep->ep.udp.server_addr, ep->ep.udp.server_port);
  else
    snprintf(buf, size, "%.100s:%d", ep->ep.uart.devname, ep->ep.uart.baudrate);
  return buf;
}

static enum direction_t direction_of(struct endpoint_t *ep) {
  return (ep->idx == 0) ? DIR_DOWN : DIR_UP;
}

/**
 * Create an UDP endpoint thread
 */
void *udp_endpoint(void *arg) {
  struct endpoint_t *e = (struct endpoint_t *)arg;
  struct endpoint_udp_t *ep = &e->ep.udp;

  /* Create the socket */
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(fd < 0) {
    fprintf(stderr, "Could not create socket for %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Enable broadcasting */
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

  /* Keep the send buffer small, frames that do not fit are dropped (and counted) */
  int sndbuf = UDP_SNDBUF;
  setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  /* Create the input address */
  struct sockaddr_in server;
  memset(&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(ep->server_addr);
  server.sin_port = htons(ep->server_port);

  /* Bind the socket with the server address  */
  if(bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Could not bind to address %s:%d\r\n", ep->server_addr, ep->server_port);
    close(fd);
    return NULL;
  }
  e->fd = fd;

  /* Wait for messages */
  while(true) {
    struct sockaddr_in client;
    uint8_t buffer[UDP_BUFFER_SIZE];
    socklen_t len = sizeof(client);
    int n = recvfrom(fd, buffer, UDP_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&client, &len);

    // Ignore errors
    if(n < 0)
      continue;

    if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->server_addr, ep->server_port, n);

    // Send the message to the handler
    input_handler(e, buffer, n);
  }
}

/**
 * Send a message out of an UDP endpoint
 * @return true when the whole message was sent
 */
bool udp_endpoint_send(struct endpoint_t *e, uint8_t *buffer, uint16_t len) {
  struct endpoint_udp_t *ep = &e->ep.udp;

  // Check if file descriptor is valid
  if(e->fd < 0)
    return false;

  struct sockaddr_in client;
  memset(&client, 0, sizeof(client));
  client.sin_family = AF_INET;
  client.sin_addr.s_addr = inet_addr(ep->client_addr);
  client.sin_port = htons(ep->client_port);

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->client_addr, ep->client_port, len);
  return sendto(e->fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)&client, sizeof(client)) == len;
}

/**
 * Find a free endpoint
 */
static struct endpoint_t *endpoint_new(void) {
  for(int i = 0; i < MAX_ENDPOINTS; ++i) {
    if(endpoints[i].type == ENDPOINT_TYPE_NONE) {
      endpoints[i].idx = i;
      endpoints[i].fd = -1;
      return &endpoints[i];
    }
  }
  return NULL;
}

/**
 * Create and add an UDP endpoint
 */
void udp_create(char *server_addr, uint16_t server_port, char *client_addr, uint16_t client_port) {
  struct endpoint_t *e = endpoint_new();

  /* No space in endpoints anymore */
  if(e == NULL) {
    fprintf(stderr, "Could not create UDP endpoint [%s:%d %s:%d], out of space\r\n", server_addr, server_port, client_addr, client_port);
    return;
  }

  /* Create and add the endpoint */
  e->type = ENDPOINT_TYPE_UDP;
  struct endpoint_udp_t *ep = &e->ep.udp;
  strncpy(ep->server_addr, server_addr, sizeof(ep->server_addr) - 1);
  ep->server_port = server_port;
  strncpy(ep->client_addr, client_addr, sizeof(ep->client_addr) - 1);
  ep->client_port = client_port;

  /* Start the endpoint thread */
  if(verbose) printf("Created UDP endpoint with server [%s:%d] and client [%s:%d]\r\n", ep->server_addr, ep->server_port, ep->client_addr, ep->client_port);
  pthread_create(&e->thread, NULL, udp_endpoint, e);
}

/**
 * Get the baudrate based ont he argument
 */
int get_baud(unsigned int baud_rate)
{
  int baud = 0;
  switch (baud_rate)
  {
#ifdef B921600
    case 921600:
      baud = B921600;
      break;
#endif
#ifdef B460800
    case 460800:
      baud = B460800;
      break;
#endif
    case 230400:
      baud = B230400;
      break;
    case 115200:
      baud = B115200;
      break;
    case 57600:
      baud = B57600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 19200:
      baud  = B19200;
      break;
    case 9600:
      baud  = B9600;
      break;
    case 4800:
      baud  = B4800;
      break;
    case 2400:
      baud  = B2400;
      break;
    case 1800:
      baud  = B1800;
      break;
    case 1200:
      baud  = B1200;
      break;
    case 600:
      baud  = B600;
      break;
    case 300:
      baud  = B300;
      break;
    case 200:
      baud  = B200;
      break;
    case 150:
      baud  = B150;
      break;
    case 134:
      baud  = B134;
      break;
    case 110:
      baud  = B110;
      break;
    case 75:
      baud  = B75;
      break;
    case 50:
      baud  = B50;
      break;
    default:
      baud = -1;
  }  //end of switch baud_rate
  return baud;
}

/**
 * Create an UART endpoint thread
 */
void *uart_endpoint(void *arg) {
  struct endpoint_t *e = (struct endpoint_t *)arg;
  struct endpoint_uart_t *ep = &e->ep.uart;

  /* Open de uart device in blocking Read/Write mode */
  int fd = open(ep->devname, O_RDWR | O_NOCTTY);
  if(fd < 0) {
    fprintf(stderr, "Could not open uart for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Conver the baudrate */
  int baud = get_baud(ep->baudrate);
  if(baud < 0) {
    fprintf(stderr, "Could set baudrate for %s:%d\r\n", ep->devname, ep->baudrate);
    close(fd);
    return NULL;
  }

  /* Configure the UART */
  struct termios options;
  tcgetattr(fd, &options);
  options.c_cflag = baud | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);
  e->fd = fd;

  /* Try to read bytes */
  while(true) {
    uint8_t buffer[512];
    int len = read(fd, buffer, 512);

    // Read got an error
    if(len < 0) {
      if(verbose) printf("Got read error (%d) at endpoint [%s:%d]\r\n", len, ep->devname, ep->baudrate);
      usleep(20);
    }
    // No bytes to read
    else if(len == 0) {
      usleep(20);
    }
    // We succsfully received some bytes
    else if(len > 0) {
      if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);

      // Send the message to the handler
      input_handler(e, buffer, len);
    }
  }
}

/**
 * Send a message out of an UART endpoint
 * @return true when the whole message was written
 */
bool uart_endpoint_send(struct endpoint_t *e, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(e->fd < 0)
    return false;

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", e->ep.uart.devname, e->ep.uart.baudrate, len);
  return write(e->fd, buffer, len) == len;
}

/**
 * Create and add an UART endpoint
 */
void uart_create(char *devname, uint32_t baudrate) {
  struct endpoint_t *e = endpoint_new();

  /* No space in endpoints anymore */
  if(e == NULL) {
    fprintf(stderr, "Could not create UART endpoint [%s:%d], out of space\r\n", devname, baudrate);
    return;
  }

  /* Create and add the endpoint */
  e->type = ENDPOINT_TYPE_UART;
  struct endpoint_uart_t *ep = &e->ep.uart;
  strncpy(ep->devname, devname, sizeof(ep->devname) - 1);
  ep->baudrate = baudrate;

  /* Start the endpoint thread */
  if(verbose) printf("Created UART endpoint [%s:%d]\r\n", ep->devname, ep->baudrate);
  pthread_create(&e->thread, NULL, uart_endpoint, e);
}

/**
 * Keep the CPU awake
 * The Herelink ground unit holds the forwarding for ~35 ms once per second when its CPU is idle.
 * A busy thread at the lowest priority avoids this without delaying the other threads; letting it
 * sleep part of the time (even 50%) does not help. It runs on CPU 0, a low power core on the Herelink.
 */
void *keep_awake_thread(void *arg __attribute__((unused))) {
  pid_t tid = syscall(SYS_gettid);
  setpriority(PRIO_PROCESS, tid, 19);

  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(0, &cpus);
  sched_setaffinity(tid, sizeof(cpus), &cpus);

  volatile uint32_t counter = 0;
  while(true) {
    counter++;
  }
  return NULL;
}

/**
 * Read the link quality from stdin, one value per line: "rssi <dBm>" or "snr <dB>"
 * (written by the Herelink app). Without input (stdin closed) the values stay unknown.
 */
void *stdin_thread(void *arg __attribute__((unused))) {
  char line[128];
  while(fgets(line, sizeof(line), stdin) != NULL) {
    int v;
    pthread_mutex_lock(&stats_mutex);
    if(sscanf(line, "rssi %d", &v) == 1) {
      rssi = (v < -127) ? -127 : (v > 126) ? 126 : v;
      rssi_time_ms = time_ms();
    }
    else if(sscanf(line, "snr %d", &v) == 1) {
      snr = (v < -127) ? -127 : (v > 126) ? 126 : v;
      rssi_time_ms = time_ms();
    }
    pthread_mutex_unlock(&stats_mutex);
  }
  return NULL;
}

/**
 * A complete frame with a valid checksum was received
 */
static void frame_received(struct endpoint_t *ep, uint8_t *frame, uint8_t len) {
  enum direction_t dir = direction_of(ep);
  uint64_t now = time_ms();

  pthread_mutex_lock(&stats_mutex);
  struct dir_stats_t *s = &stats[dir];
  s->rx_msgs++;
  if(s->last_rx_ms != 0 && now - s->last_rx_ms > s->max_gap_ms)
    s->max_gap_ms = now - s->last_rx_ms;
  s->last_rx_ms = now;

  // Learn the aircraft id from its telemetry (sender id of a telemetry frame)
  if(dir == DIR_DOWN && len >= 8 && (frame[4] & 0x0F) == PPRZ_CLASS_TELEMETRY && frame[2] != ac_id) {
    if(verbose || ac_id >= 0) printf("Aircraft id %d (was %d)\r\n", frame[2], ac_id);
    ac_id = frame[2];
  }
  pthread_mutex_unlock(&stats_mutex);

  if(verbose && len >= 8) {
    char name[160];
    printf("%s frame at [%s]: sender %d receiver %d class %d component %d msg %d length %d\r\n",
           dir == DIR_DOWN ? "Downlink" : "Uplink", endpoint_name(ep, name, sizeof(name)),
           frame[2], frame[3], frame[4] & 0x0F, frame[4] >> 4, frame[5], len);
  }

  // In frame mode, forward each frame on its own
  if(pprz_frames)
    packet_handler(ep, frame, len);
}

static void frame_error(struct endpoint_t *ep) {
  pthread_mutex_lock(&stats_mutex);
  stats[direction_of(ep)].rx_err++;
  pthread_mutex_unlock(&stats_mutex);
}

/**
 * Parse one byte of a pprzlink v2 frame and handle the frame when it is complete and valid.
 * After a bad length or checksum the bytes after the STX are parsed again, so a valid frame that
 * starts inside a broken one is not lost.
 */
static void pprz_parse(struct endpoint_t *ep, struct pprz_parser_t *p, uint8_t c) {
  switch(p->status) {
    case 0: // Wait for STX
      if(c == PPRZ_STX) {
        p->frame[0] = c;
        p->idx = 1;
        p->status = 1;
      }
      return;
    case 1: // Length of the whole frame
      p->frame[p->idx++] = c;
      if(c < 4)
        break;
      p->len = c;
      p->ck_a = p->ck_b = c;
      p->status = (c > 4) ? 2 : 3;
      return;
    case 2: // Payload
      p->frame[p->idx++] = c;
      p->ck_a += c;
      p->ck_b += p->ck_a;
      if(p->idx == p->len - 2)
        p->status = 3;
      return;
    case 3: // Checksum A
      p->frame[p->idx++] = c;
      if(c != p->ck_a)
        break;
      p->status = 4;
      return;
    case 4: // Checksum B
      p->frame[p->idx++] = c;
      if(c != p->ck_b)
        break;
      p->status = 0;
      frame_received(ep, p->frame, p->len);
      return;
    default:
      p->status = 0;
      return;
  }

  // Invalid frame: parse the bytes after its STX again
  if(verbose) printf("Dropped invalid frame of %d bytes\r\n", p->idx);
  frame_error(ep);
  uint8_t retry[256];
  uint8_t n = p->idx - 1;
  memcpy(retry, &p->frame[1], n);
  p->status = 0;
  for(uint8_t i = 0; i < n; ++i)
    pprz_parse(ep, p, retry[i]);
}

/**
 * Handle received bytes: forward them as they are, or with --pprz only complete frames with a
 * valid checksum, one frame at a time (so frames from different endpoints are never mixed).
 * The frames are always parsed for the statistics and to learn the aircraft id.
 */
void input_handler(struct endpoint_t *ep, uint8_t *data, uint16_t len) {
  if(!pprz_frames)
    packet_handler(ep, data, len);

  for(uint16_t i = 0; i < len; ++i)
    pprz_parse(ep, &ep->parser, data[i]);
}

/**
 * Send a packet out of all endpoints except the one it came from
 */
void packet_handler(struct endpoint_t *src, uint8_t *data, uint16_t len) {
  struct dir_stats_t *s = &stats[direction_of(src)];

  pthread_mutex_lock(&send_mutex);
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    struct endpoint_t *dst = &endpoints[i];
    // Don't send to own endpoint
    if(dst == src || dst->type == ENDPOINT_TYPE_NONE)
      continue;

    bool sent = false;
    if(dst->type == ENDPOINT_TYPE_UDP)
      sent = udp_endpoint_send(dst, data, len);
    else if(dst->type == ENDPOINT_TYPE_UART)
      sent = uart_endpoint_send(dst, data, len);

    // Count every copy sent (tx counters are only changed with the send mutex)
    if(sent)
      s->tx_msgs++;
    else
      s->tx_err++;
  }
  pthread_mutex_unlock(&send_mutex);
}

static uint8_t *put_u32(uint8_t *b, uint32_t v) {
  memcpy(b, &v, 4); // pprzlink is little endian, as the Herelink and x86
  return b + 4;
}

static uint8_t *put_u16(uint8_t *b, uint16_t v) {
  memcpy(b, &v, 2);
  return b + 2;
}

/**
 * Longest gap since the last report, including the current one, and restart the measurement
 */
static uint16_t take_max_gap(struct dir_stats_t *s, uint64_t now) {
  uint64_t gap = (s->last_rx_ms == 0) ? UINT16_MAX : now - s->last_rx_ms;
  if(s->max_gap_ms > gap)
    gap = s->max_gap_ms;
  s->max_gap_ms = 0;
  return (gap > UINT16_MAX) ? UINT16_MAX : gap;
}

/**
 * Send LINK_ROUTER_REPORT towards the ground (out of all endpoints except the aircraft side)
 */
static void send_report(uint64_t start_ms) {
  uint8_t frame[64];
  uint8_t *b = frame;
  uint64_t now = time_ms();

  pthread_mutex_lock(&stats_mutex);
  if(ac_id < 0) {
    pthread_mutex_unlock(&stats_mutex);
    return;
  }
  bool quality = (rssi_time_ms != 0 && now - rssi_time_ms < RSSI_TIMEOUT_MS);

  *b++ = PPRZ_STX;
  *b++ = 0; // length, set below
  *b++ = ac_id;
  *b++ = 0; // receiver (ground)
  *b++ = PPRZ_CLASS_TELEMETRY;
  *b++ = LINK_ROUTER_REPORT_ID;
  *b++ = node;
  b = put_u32(b, (now - start_ms) / 1000);
  b = put_u32(b, stats[DIR_DOWN].rx_msgs);
  b = put_u32(b, stats[DIR_DOWN].rx_err);
  b = put_u32(b, stats[DIR_DOWN].tx_msgs);
  b = put_u32(b, stats[DIR_DOWN].tx_err);
  b = put_u32(b, stats[DIR_UP].rx_msgs);
  b = put_u32(b, stats[DIR_UP].rx_err);
  b = put_u32(b, stats[DIR_UP].tx_msgs);
  b = put_u32(b, stats[DIR_UP].tx_err);
  b = put_u32(b, stats[DIR_UP].dropped);
  b = put_u16(b, take_max_gap(&stats[DIR_DOWN], now));
  b = put_u16(b, take_max_gap(&stats[DIR_UP], now));
  *b++ = 0; // nb_gcs (no GCS discovery yet)
  *b++ = quality ? (uint8_t)rssi : VALUE_UNKNOWN;
  *b++ = quality ? (uint8_t)snr : VALUE_UNKNOWN;
  pthread_mutex_unlock(&stats_mutex);

  uint8_t len = (b - frame) + 2;
  frame[1] = len;
  uint8_t ck_a = 0, ck_b = 0;
  for(uint8_t i = 1; i < len - 2; ++i) {
    ck_a += frame[i];
    ck_b += ck_a;
  }
  *b++ = ck_a;
  *b++ = ck_b;

  packet_handler(&endpoints[0], frame, len);
}

int main(int argc, char** argv) {
  /* Make sure all endpoints are clear */
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    endpoints[i].type = ENDPOINT_TYPE_NONE;
    endpoints[i].fd = -1;
  }

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"endpoint", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {"no-keep-awake", no_argument, NULL, 'n'},
    {"pprz", no_argument, NULL, 'p'},
    {"stats", required_argument, NULL, 's'},
    {"node", required_argument, NULL, 'N'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -e --endpoint [endpoint_str]           Endpoint address to route to/from, the first one faces the aircraft\n"
    "                                          uart://DEVICE:BAUD | udp://BIND_ADDR:BIND_PORT:SEND_ADDR:SEND_PORT\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information (also the header of every frame)\n"
    "   -n --no-keep-awake                     Do not keep the CPU awake (see keep_awake_thread)\n"
    "   -p --pprz                              Only forward complete pprzlink v2 frames, one at a time\n"
    "   -s --stats PERIOD_MS                   Period of LINK_ROUTER_REPORT (default 1000, 0 = off)\n"
    "   -N --node air|ground                   Node in LINK_ROUTER_REPORT (default: air if the first endpoint is an UART)\n"
    " Link quality for LINK_ROUTER_REPORT can be given on stdin, one line per value: 'rssi <dBm>' or 'snr <dB>'\n";

  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "e:hvnps:N:", long_options, &option_index)) != -1) {
    switch (c) {
      case 'e':
        // Parse the endpoint argument UDP
        if(!strncmp(optarg, "udp", 3)) {
          char serv_addr[125], cli_addr[125];
          uint16_t serv_port, cli_port;
          if(sscanf(optarg, "udp://%124[^:]:%hu:%124[^:]:%hu", serv_addr, &serv_port, cli_addr, &cli_port) != 4) {
            fprintf(stderr, "UDP endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          udp_create(serv_addr, serv_port, cli_addr, cli_port);
        }
        // Parse the endpoint argument UART
        else if(!strncmp(optarg, "uart", 4)) {
          char devname[256];
          uint32_t baudrate;
          if(sscanf(optarg, "uart://%255[^:]:%u", devname, &baudrate) != 2) {
            fprintf(stderr, "UART endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          uart_create(devname, baudrate);
        }
        else {
          fprintf(stderr, "Endpoint %s has incorrect type only uart and udp are supported\r\n", optarg);
          return 2;
        }
        break;

      case 'v':
        verbose = true;
        break;

      case 'n':
        keep_awake = false;
        break;

      case 'p':
        pprz_frames = true;
        break;

      case 's':
        stats_period_ms = atoi(optarg);
        break;

      case 'N':
        if(!strcmp(optarg, "air"))
          node = NODE_AIR;
        else if(!strcmp(optarg, "ground"))
          node = NODE_GROUND;
        else {
          fprintf(stderr, "Node %s is not air or ground\r\n", optarg);
          return 2;
        }
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        return 0;

      default: /* ’?’ */
        printf("?? getopt returned character code %c ??\r\n", c);
        fprintf(stderr, usage, argv[0]);
        return 1;
    }
  }

  if(endpoints[0].type == ENDPOINT_TYPE_NONE) {
    fprintf(stderr, usage, argv[0]);
    return 1;
  }
  if(node == NODE_AUTO)
    node = (endpoints[0].type == ENDPOINT_TYPE_UART) ? NODE_AIR : NODE_GROUND;

  /* Start the keep awake thread */
  if(keep_awake) {
    pthread_t thread;
    if(verbose) printf("Keeping the CPU awake\r\n");
    pthread_create(&thread, NULL, keep_awake_thread, NULL);
  }

  /* Read the link quality from stdin */
  pthread_t stdin_th;
  pthread_create(&stdin_th, NULL, stdin_thread, NULL);

  uint64_t start_ms = time_ms();
  uint64_t next_report_ms = start_ms + stats_period_ms;
  while(true) {
    usleep(50000);
    uint64_t now = time_ms();
    if(stats_period_ms > 0 && now >= next_report_ms) {
      send_report(start_ms);
      next_report_ms += stats_period_ms;
      if(next_report_ms < now)
        next_report_ms = now + stats_period_ms;
    }
  }

  return 0;
}
