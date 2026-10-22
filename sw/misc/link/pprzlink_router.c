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
./pprzlink_router_cc -e uart:///dev/ttyS2:57600 -e udp://192.168.0.10:5243:192.168.0.11:5242
./pprzlink_router_cc -e udp://192.168.0.11:5242:192.168.0.10:5243 -e udp://0.0.0.0:4243:192.168.43.255:4242
 */

#define _GNU_SOURCE
#include <stdio.h>  
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
#include <sys/resource.h>
#include <sys/syscall.h>

#define UDP_BUFFER_SIZE   1024
#define MAX_ENDPOINTS     32
#define PPRZ_STX          0x99

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

  int fd;
  pthread_t thread;
  struct pprz_parser_t parser;
};

struct endpoint_uart_t {
  char devname[512];
  int baudrate;

  int fd;
  pthread_t thread;
  struct pprz_parser_t parser;
};

enum endpoint_type_t {
  ENDPOINT_TYPE_NONE,
  ENDPOINT_TYPE_UDP,
  ENDPOINT_TYPE_UART
};

struct endpoint_t {
  enum endpoint_type_t type;
  union {
    struct endpoint_udp_t udp;
    struct endpoint_uart_t uart;
  } ep;
};

static bool verbose = false;
static bool keep_awake = true;
static bool pprz_frames = false;
static struct endpoint_t endpoints[MAX_ENDPOINTS];
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;

void packet_handler(void *ep, uint8_t *data, uint16_t len);
void input_handler(void *ep, struct pprz_parser_t *parser, uint8_t *data, uint16_t len);

/**
 * Create an UDP endpoint thread
 */
void *udp_endpoint(void *arg) {
  struct endpoint_udp_t *ep = (struct endpoint_udp_t *)arg;

  /* Create the socket */
	ep->fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(ep->fd < 0) {
		fprintf(stderr, "Could not create socket for %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Enable broadcasting */
  int one = 1;
  setsockopt(ep->fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  
  /* Create the input address */
	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = inet_addr(ep->server_addr);
	server.sin_port = htons(ep->server_port);

  /* Bind the socket with the server address  */
  if(bind(ep->fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Could not bind to address %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Wait for messages */
  while(true) {
    struct sockaddr_in client;
    uint8_t buffer[UDP_BUFFER_SIZE];
    socklen_t len = sizeof(client);
    int n = recvfrom(ep->fd, buffer, UDP_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&client, &len);

    // Ignore errors
    if(n < 0)
      continue;

    if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->server_addr, ep->server_port, n);
    
    // Send the message to the handler
    input_handler(ep, &ep->parser, buffer, n);
  }
}

/**
 * Send a message out of an UDP endpoint
 */
void udp_endpoint_send(struct endpoint_udp_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  struct sockaddr_in client;
	client.sin_family = AF_INET;
	client.sin_addr.s_addr = inet_addr(ep->client_addr);
	client.sin_port = htons(ep->client_port);

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->client_addr, ep->client_port, len);
  sendto(ep->fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)&client, sizeof(client));
}

/**
 * Create and add an UDP endpoint
 */
void udp_create(char *server_addr, uint16_t server_port, char *client_addr, uint16_t client_port) {
  uint16_t i = 0;
  /* Find a free space in the endpoints */
  for(i = 0; i < MAX_ENDPOINTS; ++i) {
    if(endpoints[i].type == ENDPOINT_TYPE_NONE)
      break;
  }

  /* No space in endpoints anymore */
  if(i == MAX_ENDPOINTS) {
    fprintf(stderr, "Could not create UDP endpoint [%s:%d %s:%d], out of space\r\n", server_addr, server_port, client_addr, client_port);
    return;
  }

  /* Create and add the endpoint */
  endpoints[i].type = ENDPOINT_TYPE_UDP;
  struct endpoint_udp_t *ep = &endpoints[i].ep.udp;
  strncpy(ep->server_addr, server_addr, 128);
  ep->server_port = server_port;
  strncpy(ep->client_addr, client_addr, 128);
  ep->client_port = client_port;

  /* Start the endpoint thread */
  if(verbose) printf("Created UDP endpoint with server [%s:%d] and client [%s:%d]\r\n", ep->server_addr, ep->server_port, ep->client_addr, ep->client_port);
  pthread_create(&ep->thread, NULL, udp_endpoint, ep);
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
  struct endpoint_uart_t *ep = (struct endpoint_uart_t *)arg;

  /* Open de uart device in blocking Read/Write mode */
  ep->fd = open(ep->devname, O_RDWR | O_NOCTTY);
  if(ep->fd < 0) {
    fprintf(stderr, "Could not open uart for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Conver the baudrate */
  int baud = get_baud(ep->baudrate);
  if(baud < 0) {
    fprintf(stderr, "Could set baudrate for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Configure the UART */
  struct termios options;
	tcgetattr(ep->fd, &options);
	options.c_cflag = baud | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(ep->fd, TCIFLUSH);
	tcsetattr(ep->fd, TCSANOW, &options);

  /* Try to read bytes */
  while(true) {
    uint8_t buffer[512];
    int len = read(ep->fd, buffer, 512);

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
      input_handler(ep, &ep->parser, buffer, len);
    }
  }
}

/**
 * Send a message out of an UART endpoint
 */
void uart_endpoint_send(struct endpoint_uart_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);
  write(ep->fd, buffer, len);
}

/**
 * Create and add an UART endpoint
 */
void uart_create(char *devname, uint32_t baudrate) {
  uint16_t i = 0;
  /* Find a free space in the endpoints */
  for(i = 0; i < MAX_ENDPOINTS; ++i) {
    if(endpoints[i].type == ENDPOINT_TYPE_NONE)
      break;
  }

  /* No space in endpoints anymore */
  if(i == MAX_ENDPOINTS) {
    fprintf(stderr, "Could not create UART endpoint [%s:%d], out of space\r\n", devname, baudrate);
    return;
  }

  /* Create and add the endpoint */
  endpoints[i].type = ENDPOINT_TYPE_UART;
  struct endpoint_uart_t *ep = &endpoints[i].ep.uart;
  strncpy(ep->devname, devname, 512);
  ep->baudrate = baudrate;

  /* Start the endpoint thread */
  if(verbose) printf("Created UART endpoint [%s:%d]\r\n", ep->devname, ep->baudrate);
  pthread_create(&ep->thread, NULL, uart_endpoint, ep);
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
 * Parse one byte of a pprzlink v2 frame and forward the frame when it is complete and valid.
 * After a bad length or checksum the bytes after the STX are parsed again, so a valid frame that
 * starts inside a broken one is not lost.
 */
static void pprz_parse(void *ep, struct pprz_parser_t *p, uint8_t c) {
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
      packet_handler(ep, p->frame, p->len);
      p->status = 0;
      return;
    default:
      p->status = 0;
      return;
  }

  // Invalid frame: parse the bytes after its STX again
  if(verbose) printf("Dropped invalid frame of %d bytes\r\n", p->idx);
  uint8_t retry[256];
  uint8_t n = p->idx - 1;
  memcpy(retry, &p->frame[1], n);
  p->status = 0;
  for(uint8_t i = 0; i < n; ++i)
    pprz_parse(ep, p, retry[i]);
}

/**
 * Handle received bytes: forward them as they are, or with --pprz only complete frames with a
 * valid checksum, one frame at a time (so frames from different endpoints are never mixed)
 */
void input_handler(void *ep, struct pprz_parser_t *p, uint8_t *data, uint16_t len) {
  if(!pprz_frames) {
    packet_handler(ep, data, len);
    return;
  }

  for(uint16_t i = 0; i < len; ++i)
    pprz_parse(ep, p, data[i]);
}

/**
 * Handle incoming packets from server side
 */
void packet_handler(void *ep, uint8_t *data, uint16_t len) {
  pthread_mutex_lock(&send_mutex);
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    switch(endpoints[i].type) {
      case ENDPOINT_TYPE_UDP:
        // Don't send to own endpoint
        if(ep == &endpoints[i].ep.udp)
          continue;

        // Send message to the other endpoints
        udp_endpoint_send(&endpoints[i].ep.udp, data, len);
        break;
      case ENDPOINT_TYPE_UART:
        // Don't send to own endpoint
        if(ep == &endpoints[i].ep.uart)
          continue;
        
        // Send the message to the other endpoints
        uart_endpoint_send(&endpoints[i].ep.uart, data, len);
        break;
      default:
        break;
    }
  }
  pthread_mutex_unlock(&send_mutex);
}

int main(int argc, char** argv) {
  /* Make sure all endpoints are clear */
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    endpoints[i].type = ENDPOINT_TYPE_NONE;
  }

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"endpoint", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {"no-keep-awake", no_argument, NULL, 'n'},
    {"pprz", no_argument, NULL, 'p'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -e --endpoint [endpoint_str]           Endpoint address to route to/from\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n"
    "   -n --no-keep-awake                     Do not keep the CPU awake (see keep_awake_thread)\n"
    "   -p --pprz                              Only forward complete pprzlink v2 frames, one at a time\n";
  
  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "e:hvnp", long_options, &option_index)) != -1) {  
    switch (c) {
      case 'e':
        // Parse the endpoint argument UDP
        if(!strncmp(optarg, "udp", 3)) {
          char serv_addr[125], cli_addr[125];
          uint16_t serv_port, cli_port;
          if(sscanf(optarg, "udp://%[^:]:%hu:%[^:]:%hu", serv_addr, &serv_port, cli_addr, &cli_port) != 4) {
            fprintf(stderr, "UDP endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          udp_create(serv_addr, serv_port, cli_addr, cli_port);
        }
        // Parse the endpoint argument UART
        else if(!strncmp(optarg, "uart", 4)) {
          char devname[256];
          uint32_t baudrate;
          if(sscanf(optarg, "uart://%[^:]:%u", devname, &baudrate) != 2) {
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

      case 'h':
        fprintf(stderr, usage, argv[0]);
        return 0;

      default: /* ’?’ */
        printf("?? getopt returned character code %c ??\r\n", c);
        fprintf(stderr, usage, argv[0]);
        return 1;
    }
  }

  /* Start the keep awake thread */
  if(keep_awake) {
    pthread_t thread;
    if(verbose) printf("Keeping the CPU awake\r\n");
    pthread_create(&thread, NULL, keep_awake_thread, NULL);
  }

  while(true) {
    usleep(50000);
  }
  
  return 0;
}