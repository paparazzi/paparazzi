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
 * Routes pprzlink frames between endpoints (UART and UDP). The first endpoint faces the aircraft: what it
 * receives is downlink and goes out of all other endpoints, what the others receive is uplink and goes out of
 * the first endpoint only. See pprzlink_router.md. Examples:

Herelink air unit:     pprzlink_router -p -e uart:///dev/ttyS2:460800 -e udp://192.168.0.10:5243:192.168.0.11:5242
Herelink ground unit:  pprzlink_router -p -e udp://192.168.0.11:5242:192.168.0.10:5243 -e udp://0.0.0.0:4243:auto:4242
Companion on a mesh (own ports on the mesh, 4243:auto:4242 if the mesh is bridged to the GCS network):
                       pprzlink_router -p -e uart:///dev/ttyAMA0:460800 -e udp://0.0.0.0:4245:auto:4244
Ground computer between the mesh and the GCS network (several aircraft):
                       pprzlink_router -p -e udp://0.0.0.0:4244:auto:4245 -e udp://0.0.0.0:4243:auto:4242
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
#include <ifaddrs.h>
#include <net/if.h>

#define UDP_BUFFER_SIZE   1024
#define UDP_SNDBUF        32768   ///< Small send buffer: on a slow link, drop frames instead of delaying them
#define MAX_ENDPOINTS     32
#define PPRZ_STX          0x99
#define PPRZ_CLASS_TELEMETRY  1
#define PPRZ_BROADCAST_ID 0xFF    ///< receiver id of uplink for all aircraft
#define LINK_ROUTER_REPORT_ID 51  ///< LINK_ROUTER_REPORT in the telemetry class
#define VALUE_UNKNOWN     127     ///< RSSI or SNR not available
#define RSSI_TIMEOUT_MS   5000    ///< RSSI from stdin older than this is not reported
#define MAX_PEERS         16      ///< GCS computers or aircraft per auto endpoint
#define PEER_TIMEOUT_MS   15000   ///< A peer is forgotten after this time without frames (link pings every 5 s)
#define BEACON_PERIOD_MS  1000    ///< Period of the discovery beacon when the statistics are off
#define MAX_BROADCASTS    8
#define MAX_AC            32      ///< aircraft with statistics
#define AC_ALL            -1      ///< frame for all aircraft (uplink broadcast, or forwarded without -p)
#define AC_NONE           -2      ///< frame of no known aircraft

/* Parser for pprzlink v2 frames: STX, LEN (whole frame), payload, CK_A, CK_B */
struct pprz_parser_t {
  uint8_t status;
  uint8_t len;
  uint8_t idx;
  uint8_t ck_a;
  uint8_t ck_b;
  uint8_t frame[256];
};

/* A GCS computer (ground side) or an aircraft (aircraft side) found by an auto endpoint */
struct peer_t {
  struct sockaddr_in addr;
  uint64_t last_seen_ms;  ///< 0 if the slot is free
  uint32_t ac[8];         ///< aircraft ids seen in the telemetry from this peer (bit set)
};

struct endpoint_udp_t {
  char server_addr[128];
  uint16_t server_port;
  char client_addr[128];  ///< "auto": send to the peers found (and a beacon to the broadcast addresses)
  uint16_t client_port;   ///< in auto mode: port of the beacon
  bool auto_peers;
  struct sockaddr_in last_src;   ///< sender of the datagram being parsed
  struct peer_t peers[MAX_PEERS]; ///< protected by the send mutex
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

/* Statistics of one aircraft and direction, sent in LINK_ROUTER_REPORT */
struct dir_stats_t {
  uint32_t rx_msgs;
  uint32_t tx_msgs;
  uint32_t tx_err;
  uint64_t last_rx_ms;  ///< time of the last received frame, 0 if none yet
  uint32_t max_gap_ms;  ///< longest time between two frames since the last report
};

struct ac_stats_t {
  int id;               ///< -1: free
  struct dir_stats_t dir[2];
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
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;   ///< sending and the peer tables
static pthread_mutex_t stats_mutex = PTHREAD_MUTEX_INITIALIZER;  ///< everything below (lock after send_mutex)
static struct ac_stats_t ac_stats[MAX_AC];
static uint32_t rx_err[2];    ///< rejected frames (the aircraft is not known)
static uint32_t up_dropped;   ///< uplink not forwarded: for an unknown aircraft
static int8_t rssi = VALUE_UNKNOWN, snr = VALUE_UNKNOWN;
static uint64_t rssi_time_ms = 0;
static struct in_addr extra_broadcasts[MAX_BROADCASTS];  ///< -b: beacon addresses besides the interfaces
static int nb_extra_broadcasts = 0;

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

static const char *addr_name(struct sockaddr_in *a, char *buf, size_t size) {
  char ip[INET_ADDRSTRLEN];
  snprintf(buf, size, "%s:%d", inet_ntop(AF_INET, &a->sin_addr, ip, sizeof(ip)), ntohs(a->sin_port));
  return buf;
}

static enum direction_t direction_of(struct endpoint_t *ep) {
  return (ep->idx == 0) ? DIR_DOWN : DIR_UP;
}

static bool is_auto(struct endpoint_t *ep) {
  return ep->type == ENDPOINT_TYPE_UDP && ep->ep.udp.auto_peers;
}

/********************************************************************************************************
 * Statistics per aircraft (called with the stats mutex)
 */

static struct ac_stats_t *ac_find(int id, bool create) {
  struct ac_stats_t *free_slot = NULL;
  for(int i = 0; i < MAX_AC; ++i) {
    if(ac_stats[i].id == id)
      return &ac_stats[i];
    if(ac_stats[i].id < 0 && free_slot == NULL)
      free_slot = &ac_stats[i];
  }
  if(create && free_slot != NULL) {
    memset(free_slot, 0, sizeof(*free_slot));
    free_slot->id = id;
    printf("Aircraft %d found\r\n", id);
    fflush(stdout);
  }
  return create ? free_slot : NULL;
}

static bool ac_any(void) {
  for(int i = 0; i < MAX_AC; ++i)
    if(ac_stats[i].id >= 0)
      return true;
  return false;
}

static void dir_rx(struct dir_stats_t *d, uint64_t now) {
  d->rx_msgs++;
  if(d->last_rx_ms != 0 && now - d->last_rx_ms > d->max_gap_ms)
    d->max_gap_ms = now - d->last_rx_ms;
  d->last_rx_ms = now;
}

/* ac: aircraft id, AC_ALL or AC_NONE */
static void count_rx(int ac, enum direction_t dir) {
  uint64_t now = time_ms();
  for(int i = 0; i < MAX_AC; ++i)
    if(ac_stats[i].id >= 0 && (ac == AC_ALL || ac_stats[i].id == ac))
      dir_rx(&ac_stats[i].dir[dir], now);
}

static void count_tx(int ac, enum direction_t dir, bool sent) {
  for(int i = 0; i < MAX_AC; ++i) {
    if(ac_stats[i].id >= 0 && (ac == AC_ALL || ac_stats[i].id == ac)) {
      if(sent)
        ac_stats[i].dir[dir].tx_msgs++;
      else
        ac_stats[i].dir[dir].tx_err++;
    }
  }
}

/********************************************************************************************************
 * Peers of the auto endpoints (called with the send mutex)
 */

static bool peer_alive(struct peer_t *p, uint64_t now) {
  return p->last_seen_ms != 0 && now - p->last_seen_ms < PEER_TIMEOUT_MS;
}

static bool peer_has_ac(struct peer_t *p, int ac) {
  return (p->ac[ac >> 5] >> (ac & 31)) & 1;
}

/**
 * Remember the peer that sent a valid frame to an auto endpoint, and the aircraft behind it
 * @return true if the peer is new
 */
static bool peer_seen(struct endpoint_udp_t *ep, struct sockaddr_in *addr, int ac) {
  uint64_t now = time_ms();
  struct peer_t *slot = NULL;
  for(int i = 0; i < MAX_PEERS; ++i) {
    struct peer_t *p = &ep->peers[i];
    if(p->last_seen_ms != 0 && p->addr.sin_addr.s_addr == addr->sin_addr.s_addr && p->addr.sin_port == addr->sin_port) {
      slot = p;
      break;
    }
  }
  bool known = (slot != NULL && peer_alive(slot, now));
  if(slot == NULL) {
    // a free or expired slot, else the oldest one
    for(int i = 0; i < MAX_PEERS; ++i) {
      struct peer_t *p = &ep->peers[i];
      if(slot == NULL || !peer_alive(p, now) || (peer_alive(slot, now) && p->last_seen_ms < slot->last_seen_ms))
        slot = p;
      if(!peer_alive(slot, now))
        break;
    }
  }
  if(!known) {
    memset(slot, 0, sizeof(*slot));
    slot->addr = *addr;
  }
  slot->last_seen_ms = now;
  if(ac >= 0)
    slot->ac[ac >> 5] |= 1u << (ac & 31);
  return !known;
}

static int nb_gcs_alive(void) {
  int n = 0;
  uint64_t now = time_ms();
  for(int i = 1; i < MAX_ENDPOINTS; ++i) {
    if(!is_auto(&endpoints[i]))
      continue;
    for(int j = 0; j < MAX_PEERS; ++j)
      n += peer_alive(&endpoints[i].ep.udp.peers[j], now);
  }
  return n;
}

/********************************************************************************************************
 * UDP endpoints
 */

/**
 * Did this datagram come from one of this router's own sockets (e.g. its beacon received by an endpoint
 * bound to 0.0.0.0 on the same port)? It must not be routed again.
 */
static bool from_self(struct sockaddr_in *src) {
  bool own_port_any = false;
  for(int i = 0; i < MAX_ENDPOINTS; ++i) {
    struct endpoint_udp_t *u = &endpoints[i].ep.udp;
    if(endpoints[i].type != ENDPOINT_TYPE_UDP || u->server_port != ntohs(src->sin_port))
      continue;
    in_addr_t bound = inet_addr(u->server_addr);
    if(bound == src->sin_addr.s_addr)
      return true;
    own_port_any |= (bound == INADDR_ANY);
  }
  if(!own_port_any)
    return false;
  // a socket bound to 0.0.0.0 sends from the address of an interface
  bool local = false;
  struct ifaddrs *ifas;
  if(getifaddrs(&ifas) == 0) {
    for(struct ifaddrs *i = ifas; i != NULL; i = i->ifa_next) {
      if(i->ifa_addr != NULL && i->ifa_addr->sa_family == AF_INET) {
        struct sockaddr_in a;
        memcpy(&a, i->ifa_addr, sizeof(a));
        local |= (a.sin_addr.s_addr == src->sin_addr.s_addr);
      }
    }
    freeifaddrs(ifas);
  }
  return local;
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

    // Ignore errors, and what this router sent itself
    if(n < 0 || from_self(&client))
      continue;

    if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->server_addr, ep->server_port, n);

    // Send the message to the handler
    ep->last_src = client;
    input_handler(e, buffer, n);
  }
}

static bool udp_sendto(struct endpoint_t *e, uint8_t *buffer, uint16_t len, struct sockaddr_in *to) {
  if(e->fd < 0)
    return false;
  if(verbose) {
    char a[64];
    printf("Send packet at endpoint [%s] with length %d\r\n", addr_name(to, a, sizeof(a)), len);
  }
  return sendto(e->fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)to, sizeof(*to)) == len;
}

/**
 * Is this the address of one of the other (fixed) UDP endpoints, e.g. the Herelink radio interface?
 */
static bool is_fixed_endpoint_addr(struct in_addr a) {
  for(int i = 0; i < MAX_ENDPOINTS; ++i) {
    if(endpoints[i].type == ENDPOINT_TYPE_UDP && !endpoints[i].ep.udp.auto_peers &&
        inet_addr(endpoints[i].ep.udp.server_addr) == a.s_addr)
      return true;
  }
  return false;
}

/**
 * Broadcast addresses for the beacon of an auto endpoint: those of the interfaces that are up (not loopback,
 * not the interface of another endpoint such as the radio, only the bound one if the endpoint is bound to
 * an address), and the ones given with -b.
 */
static int beacon_addresses(struct endpoint_udp_t *ep, struct in_addr *out, int max) {
  int n = 0;
  in_addr_t bind_addr = inet_addr(ep->server_addr);
  struct ifaddrs *ifas;
  if(getifaddrs(&ifas) == 0) {
    for(struct ifaddrs *i = ifas; i != NULL && n < max; i = i->ifa_next) {
      if(i->ifa_addr == NULL || i->ifa_addr->sa_family != AF_INET || i->ifa_broadaddr == NULL)
        continue;
      if(!(i->ifa_flags & IFF_UP) || !(i->ifa_flags & IFF_BROADCAST) || (i->ifa_flags & IFF_LOOPBACK))
        continue;
      struct sockaddr_in a, b;
      memcpy(&a, i->ifa_addr, sizeof(a));
      memcpy(&b, i->ifa_broadaddr, sizeof(b));
      if(is_fixed_endpoint_addr(a.sin_addr) || (bind_addr != INADDR_ANY && a.sin_addr.s_addr != bind_addr))
        continue;
      out[n++] = b.sin_addr;
    }
    freeifaddrs(ifas);
  }
  for(int i = 0; i < nb_extra_broadcasts && n < max; ++i)
    out[n++] = extra_broadcasts[i];
  return n;
}

/**
 * Send a frame out of an UDP endpoint (called with the send mutex)
 * @param ac aircraft of the frame for the statistics, AC_ALL or AC_NONE
 * @param beacon the router's report: broadcast only from an auto endpoint
 * @return false if an uplink frame has no aircraft to go to
 */
static bool udp_endpoint_send(struct endpoint_t *e, uint8_t *buffer, uint16_t len, enum direction_t dir, int ac,
                              bool beacon) {
  struct endpoint_udp_t *ep = &e->ep.udp;
  struct sockaddr_in to;
  memset(&to, 0, sizeof(to));
  to.sin_family = AF_INET;
  to.sin_port = htons(ep->client_port);

  if(!ep->auto_peers) {
    to.sin_addr.s_addr = inet_addr(ep->client_addr);
    count_tx(ac, dir, udp_sendto(e, buffer, len, &to));
    return true;
  }

  if(beacon) {
    struct in_addr bcast[MAX_BROADCASTS + 8];
    int n = beacon_addresses(ep, bcast, MAX_BROADCASTS + 8);
    for(int i = 0; i < n; ++i) {
      to.sin_addr = bcast[i];
      count_tx(ac, dir, udp_sendto(e, buffer, len, &to));
    }
    return true;
  }

  // Downlink: to every GCS found. Uplink: to the peers the aircraft is behind (all of them for a broadcast).
  uint64_t now = time_ms();
  bool sent_any = false;
  for(int i = 0; i < MAX_PEERS; ++i) {
    struct peer_t *p = &ep->peers[i];
    if(!peer_alive(p, now) || (dir == DIR_UP && ac >= 0 && !peer_has_ac(p, ac)))
      continue;
    bool sent = udp_sendto(e, buffer, len, &p->addr);
    if(dir == DIR_UP && ac == AC_ALL) {
      for(int j = 0; j < 256; ++j)  // a broadcast counts for the aircraft behind this peer
        if(peer_has_ac(p, j))
          count_tx(j, dir, sent);
    }
    else
      count_tx(ac, dir, sent);
    sent_any = true;
  }
  return dir == DIR_DOWN || sent_any;
}

/**
 * Create and add an UDP endpoint
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
  ep->auto_peers = !strcmp(client_addr, "auto");

  /* Start the endpoint thread */
  if(verbose) printf("Created UDP endpoint with server [%s:%d] and client [%s:%d]\r\n", ep->server_addr, ep->server_port, ep->client_addr, ep->client_port);
  pthread_create(&e->thread, NULL, udp_endpoint, e);
}

/********************************************************************************************************
 * UART endpoints
 */

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
 * Send a message out of an UART endpoint (called with the send mutex)
 */
static void uart_endpoint_send(struct endpoint_t *e, uint8_t *buffer, uint16_t len, enum direction_t dir, int ac) {
  bool sent = false;
  if(e->fd >= 0) {
    if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", e->ep.uart.devname, e->ep.uart.baudrate, len);
    sent = (write(e->fd, buffer, len) == len);
  }
  count_tx(ac, dir, sent);
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

/********************************************************************************************************
 * Threads
 */

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
 * (e.g. written by the Herelink app). Without input (stdin closed) the values stay unknown.
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

/********************************************************************************************************
 * Routing
 */

/**
 * Send a frame: downlink out of all endpoints except the first one, uplink out of the first one.
 * @param ac aircraft of the frame, AC_ALL or AC_NONE
 * @param report the router's own report: the beacon of the auto endpoints
 * @param beacon_only only send the report as beacon (statistics off)
 * @return false if an uplink frame had no aircraft to go to
 */
static bool send_frame(struct endpoint_t *src, uint8_t *data, uint16_t len, enum direction_t dir, int ac,
                       bool report, bool beacon_only) {
  bool routed = true;
  pthread_mutex_lock(&send_mutex);
  pthread_mutex_lock(&stats_mutex);
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    struct endpoint_t *dst = &endpoints[i];
    if(dst == src || dst->type == ENDPOINT_TYPE_NONE || (dir == DIR_UP) != (i == 0))
      continue;

    bool beacon = report && is_auto(dst);
    if(beacon_only && !beacon)
      continue;

    if(dst->type == ENDPOINT_TYPE_UDP)
      routed &= udp_endpoint_send(dst, data, len, dir, ac, beacon);
    else if(dst->type == ENDPOINT_TYPE_UART)
      uart_endpoint_send(dst, data, len, dir, ac);
  }
  pthread_mutex_unlock(&stats_mutex);
  pthread_mutex_unlock(&send_mutex);
  return routed;
}

/**
 * A complete frame with a valid checksum was received
 */
static void frame_received(struct endpoint_t *ep, uint8_t *frame, uint8_t len) {
  enum direction_t dir = direction_of(ep);
  bool telemetry = (len >= 8 && (frame[4] & 0x0F) == PPRZ_CLASS_TELEMETRY);
  int ac = (dir == DIR_DOWN) ? AC_NONE : AC_ALL;
  bool forward = true;

  pthread_mutex_lock(&stats_mutex);
  if(dir == DIR_DOWN) {
    // Downlink: the aircraft is the sender (a new one is added from its telemetry)
    if(len >= 8 && ac_find(frame[2], telemetry) != NULL)
      ac = frame[2];
    if(ac >= 0)
      count_rx(ac, dir);
  }
  else if(len >= 8) {
    // Uplink: to one aircraft (receiver id) or to all (0xFF or 0)
    int receiver = frame[3];
    if(receiver != PPRZ_BROADCAST_ID && receiver != 0) {
      if(ac_find(receiver, false) != NULL)
        ac = receiver;
      else if(ac_any() && pprz_frames)
        forward = false;  // an aircraft this router does not serve
    }
    if(forward)
      count_rx(ac, dir);
    else
      up_dropped++;
  }
  pthread_mutex_unlock(&stats_mutex);

  // An auto endpoint learns its peers: GCS computers, or aircraft on the aircraft side
  if(is_auto(ep)) {
    pthread_mutex_lock(&send_mutex);
    bool is_new = peer_seen(&ep->ep.udp, &ep->ep.udp.last_src, (dir == DIR_DOWN && telemetry) ? frame[2] : -1);
    pthread_mutex_unlock(&send_mutex);
    if(is_new) {
      char a[64];
      printf("%s %s found\r\n", dir == DIR_DOWN ? "Aircraft link" : "GCS", addr_name(&ep->ep.udp.last_src, a, sizeof(a)));
      fflush(stdout);
    }
  }

  if(verbose && len >= 8) {
    char name[160];
    printf("%s frame at [%s]: sender %d receiver %d class %d component %d msg %d length %d\r\n",
           dir == DIR_DOWN ? "Downlink" : "Uplink", endpoint_name(ep, name, sizeof(name)),
           frame[2], frame[3], frame[4] & 0x0F, frame[4] >> 4, frame[5], len);
  }

  // In frame mode, forward each frame on its own
  if(pprz_frames && forward) {
    if(!send_frame(ep, frame, len, dir, ac, false, false)) {
      pthread_mutex_lock(&stats_mutex);
      up_dropped++;  // no peer behind the aircraft side for this aircraft
      pthread_mutex_unlock(&stats_mutex);
    }
  }
}

static void frame_error(struct endpoint_t *ep) {
  pthread_mutex_lock(&stats_mutex);
  rx_err[direction_of(ep)]++;
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
 * The frames are always parsed for the statistics and to learn the aircraft.
 */
void input_handler(struct endpoint_t *ep, uint8_t *data, uint16_t len) {
  if(!pprz_frames)
    send_frame(ep, data, len, direction_of(ep), AC_ALL, false, false);

  for(uint16_t i = 0; i < len; ++i)
    pprz_parse(ep, &ep->parser, data[i]);
}

/********************************************************************************************************
 * Report
 */

static uint8_t *put_u32(uint8_t *b, uint32_t v) {
  memcpy(b, &v, 4); // pprzlink is little endian, as ARM and x86
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
 * Send LINK_ROUTER_REPORT of every aircraft towards the ground; it is the beacon of the auto endpoints
 */
static void send_reports(uint64_t start_ms, bool beacon_only) {
  pthread_mutex_lock(&send_mutex);
  int nb_gcs = nb_gcs_alive();
  pthread_mutex_unlock(&send_mutex);

  for(int i = 0; i < MAX_AC; ++i) {
    uint8_t frame[64];
    uint8_t *b = frame;
    uint64_t now = time_ms();

    pthread_mutex_lock(&stats_mutex);
    struct ac_stats_t *s = &ac_stats[i];
    if(s->id < 0) {
      pthread_mutex_unlock(&stats_mutex);
      continue;
    }
    int ac = s->id;
    bool quality = (rssi_time_ms != 0 && now - rssi_time_ms < RSSI_TIMEOUT_MS);
    *b++ = PPRZ_STX;
    *b++ = 0; // length, set below
    *b++ = ac;
    *b++ = 0; // receiver (ground)
    *b++ = PPRZ_CLASS_TELEMETRY;
    *b++ = LINK_ROUTER_REPORT_ID;
    *b++ = node;
    b = put_u32(b, (now - start_ms) / 1000);
    b = put_u32(b, s->dir[DIR_DOWN].rx_msgs);
    b = put_u32(b, rx_err[DIR_DOWN]);
    b = put_u32(b, s->dir[DIR_DOWN].tx_msgs);
    b = put_u32(b, s->dir[DIR_DOWN].tx_err);
    b = put_u32(b, s->dir[DIR_UP].rx_msgs);
    b = put_u32(b, rx_err[DIR_UP]);
    b = put_u32(b, s->dir[DIR_UP].tx_msgs);
    b = put_u32(b, s->dir[DIR_UP].tx_err);
    b = put_u32(b, up_dropped);
    b = put_u16(b, take_max_gap(&s->dir[DIR_DOWN], now));
    b = put_u16(b, take_max_gap(&s->dir[DIR_UP], now));
    *b++ = (nb_gcs > 255) ? 255 : nb_gcs;
    *b++ = quality ? (uint8_t)rssi : VALUE_UNKNOWN;
    *b++ = quality ? (uint8_t)snr : VALUE_UNKNOWN;
    pthread_mutex_unlock(&stats_mutex);

    uint8_t len = (b - frame) + 2;
    frame[1] = len;
    uint8_t ck_a = 0, ck_b = 0;
    for(uint8_t j = 1; j < len - 2; ++j) {
      ck_a += frame[j];
      ck_b += ck_a;
    }
    *b++ = ck_a;
    *b++ = ck_b;

    send_frame(&endpoints[0], frame, len, DIR_DOWN, ac, true, beacon_only);
  }
}

/********************************************************************************************************
 * Main
 */

int main(int argc, char** argv) {
  /* Make sure all endpoints are clear */
  for(uint16_t i = 0; i < MAX_ENDPOINTS; ++i) {
    endpoints[i].type = ENDPOINT_TYPE_NONE;
    endpoints[i].fd = -1;
  }
  for(int i = 0; i < MAX_AC; ++i)
    ac_stats[i].id = -1;

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"endpoint", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {"no-keep-awake", no_argument, NULL, 'n'},
    {"pprz", no_argument, NULL, 'p'},
    {"stats", required_argument, NULL, 's'},
    {"node", required_argument, NULL, 'N'},
    {"broadcast", required_argument, NULL, 'b'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -e --endpoint [endpoint_str]           Endpoint to route to/from. The first one faces the aircraft: what it\n"
    "                                          receives goes out of all others, what they receive goes out of it\n"
    "                                          uart://DEVICE:BAUD | udp://BIND_ADDR:BIND_PORT:SEND_ADDR:SEND_PORT\n"
    "                                          udp://BIND_ADDR:BIND_PORT:auto:PORT sends to the peers that send to it:\n"
    "                                          GCS computers (and a 1 Hz beacon to the broadcast addresses on PORT),\n"
    "                                          or as first endpoint aircraft, the uplink only to its aircraft\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information (also the header of every frame)\n"
    "   -n --no-keep-awake                     Do not keep the CPU awake (see keep_awake_thread)\n"
    "   -p --pprz                              Only forward complete pprzlink v2 frames, one at a time (always with auto)\n"
    "   -s --stats PERIOD_MS                   Period of LINK_ROUTER_REPORT (default 1000, 0 = off)\n"
    "   -N --node air|ground                   Node in LINK_ROUTER_REPORT (default: air if the first endpoint is an UART)\n"
    "   -b --broadcast ADDR                    Also send the beacon of auto endpoints to ADDR (repeatable)\n"
    " Link quality for LINK_ROUTER_REPORT can be given on stdin, one line per value: 'rssi <dBm>' or 'snr <dB>'\n";

  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "e:hvnps:N:b:", long_options, &option_index)) != -1) {
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

      case 'b':
        if(nb_extra_broadcasts >= MAX_BROADCASTS || inet_aton(optarg, &extra_broadcasts[nb_extra_broadcasts]) == 0) {
          fprintf(stderr, "Broadcast address %s is invalid or there are too many\r\n", optarg);
          return 2;
        }
        nb_extra_broadcasts++;
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

  /* An auto endpoint routes frames, so it needs the frame mode, and always sends the beacon */
  bool has_auto = false;
  for(int i = 0; i < MAX_ENDPOINTS; ++i)
    has_auto |= is_auto(&endpoints[i]);
  if(has_auto && !pprz_frames) {
    printf("auto endpoint: forwarding complete pprzlink frames only (-p)\r\n");
    pprz_frames = true;
  }
  bool beacon_only = (stats_period_ms <= 0);
  int period_ms = beacon_only ? (has_auto ? BEACON_PERIOD_MS : 0) : stats_period_ms;

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
  uint64_t next_report_ms = start_ms + period_ms;
  while(true) {
    usleep(50000);
    uint64_t now = time_ms();
    if(period_ms > 0 && now >= next_report_ms) {
      send_reports(start_ms, beacon_only);
      next_report_ms += period_ms;
      if(next_report_ms < now)
        next_report_ms = now + period_ms;
    }
  }

  return 0;
}
