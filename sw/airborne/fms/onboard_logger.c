#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include "packet_header.h"

static const char filter_exp[] = "dst port 4242 && udp";

#define MIN_MSG_LENGTH 3

static void got_pprz_message(const u_char *buf, const struct timeval *ts)
{
  int i = 0;
  u_char length;
  u_char ck_A = 0, ck_B = 0;
  static unsigned int start_secs = 0;

  if (start_secs == 0) {
    start_secs = ts->tv_sec;
  }

  length = buf[i];

  for (i = 0; i < length - 3; i++) {
    ck_A += buf[i];
    ck_B += ck_A;
  }

  if (ck_A != buf[length - 3] || ck_B != buf[length - 2]) {
    printf("checksum mismatch\n");
    return;
  }
  // printf("Got pprz msg len %i, ckA %02x, ckB %02x (%02x %02x)\n", length, ck_A, ck_B, buf[length - 3], buf[length - 2]);
  printf("%i.%06i ", (unsigned) ts->tv_sec - start_secs, (unsigned) ts->tv_usec);
  printf("%d ", (uint32_t) buf[1]);      // paparazzi timestamp; see udp_transport.h
  printf("%i %i ", buf[5], buf[6]);    // AC_ID MSG_ID
  for (i = 6; i < length - 3; i++) {
    printf("%02x ", buf[i]);
  }

  printf("\n");
}

static void got_packet(u_char *args, const struct pcap_pkthdr *header,
                       const u_char *packet)
{
  const u_char *payload;
  const struct ethernet_header *ethernet;
  const struct ip_header *ip;
  const struct udp_header *udp;
  unsigned int msg_length;

  u_short udp_length;
  u_int size_ip;
  int i;

  ethernet = (struct ethernet_header *) packet;
  ip = (struct ip_header *)(packet + ETHERNET_HEADER_LENGTH);
  size_ip = IP_HL_WORDS(ip) * 4;
  if (size_ip < 20) {
    printf("invalid IP hdr length: %u bytes\n", size_ip);
    return;
  }

  udp = (struct udp_header *)((u_char *)ip + size_ip);

  payload = (u_char *)((u_char *)udp + sizeof(struct udp_header));

  udp_length = htons(udp->uh_len);
  //printf ("Got udp packet length %i\n", udp_length);
  //for ( i = 0; i < udp_length; i++)
  //printf ("[%02i] \n", payload[i]);

  i = 0;
  while (i < udp_length) {
    if (payload[i] != 0x98) {
      // printf("missing start byte\n");
      break;
    }
    i++;
    msg_length = payload[i];
    if ((i + msg_length <= udp_length) && msg_length >= MIN_MSG_LENGTH) {
      got_pprz_message(payload + i, &header->ts);
      i += payload[i] - 1;
    }
  }
}

static int init_capture(char *device, pcap_t **handle)
{
  char errbuf[PCAP_ERRBUF_SIZE];

  bpf_u_int32 mask;
  bpf_u_int32 net;

  struct bpf_program fp;

  if (pcap_lookupnet(device, &net, &mask, errbuf) == -1) {
    fprintf(stderr, "Couldn't get netmask for device %s %s\n", device, errbuf);
    net = 0;
    mask = 0;
  }

  *handle = pcap_open_live(device, BUFSIZ, 1, 1000, errbuf);
  if (handle == NULL) {
    fprintf(stderr, "Couldn't open device: %s\n", errbuf);
    return 2;
  }

  if (pcap_compile(*handle, &fp, filter_exp, 0, net) == -1) {
    fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp, pcap_geterr(*handle));
    return 2;
  }

  if (pcap_setfilter(*handle, &fp) == -1) {
    fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp, pcap_geterr(*handle));
    return 2;
  }

  return 0;
}

static void print_usage(char *appname)
{
  printf("Usage: %s INTERFACE\n", appname);
}

int main(int argc, char *argv[])
{
  pcap_t *handle;

  if (argc != 2) {
    print_usage(argv[0]);
    return 2;
  }

  init_capture(argv[1], &handle);

  pcap_loop(handle, -1, got_packet, NULL);

  pcap_close(handle);

  return 0;
}
