/* Ethernet addresses are 6 bytes */
#define ETHER_ADDR_LEN  6

#define ETHERNET_HEADER_LENGTH 14

/* Ethernet header */
struct ethernet_header {
  u_char ether_dhost[ETHER_ADDR_LEN]; /* Destination host address */
  u_char ether_shost[ETHER_ADDR_LEN]; /* Source host address */
  u_short ether_type; /* IP? ARP? RARP? etc */
} __attribute__((packed));;

/* IP header */
struct ip_header {
  u_char ip_vhl;    /* version << 4 | header length >> 2 */
  u_char ip_tos;    /* type of service */
  u_short ip_len;   /* total length */
  u_short ip_id;    /* identification */
  u_short ip_off;   /* fragment offset field */
#define IP_RF 0x8000    /* reserved fragment flag */
#define IP_DF 0x4000    /* dont fragment flag */
#define IP_MF 0x2000    /* more fragments flag */
#define IP_OFFMASK 0x1fff /* mask for fragmenting bits */
  u_char ip_ttl;    /* time to live */
  u_char ip_p;    /* protocol */
  u_short ip_sum;   /* checksum */
  struct in_addr ip_src, ip_dst; /* source and dest address */
} __attribute__((packed));
#define IP_HL_WORDS(ip)   (((ip)->ip_vhl) & 0x0f)
#define IP_V(ip)    (((ip)->ip_vhl) >> 4)

/* UDP header */
struct udp_header {
  u_short uh_sport; /* source port */
  u_short uh_dport; /* destination port */
  u_short uh_len;   /* window */
  u_short uh_sum;   /* checksum */
} __attribute__((packed));

