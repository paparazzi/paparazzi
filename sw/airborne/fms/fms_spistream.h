#ifndef FMS_SPISTREAM_H__
#define FMS_SPISTREAM_H__

#define CLIENT_SOCKET_PATH "./spistreamc.socket"
#define DAEMON_SOCKET_PATH "./spistreamd.socket"

#define min(a,b) ((a>b)? (b) : (a))

void print_message(char prefix[], uint8_t msg_id, uint8_t *data, uint16_t num_bytes);
void print_message(char prefix[], uint8_t msg_id, uint8_t *data, uint16_t num_bytes)
{
  /*
    struct tm * timeinfo;
    time_t c_time;
    char time_str[30];
  */
  uint8_t cnt;
  uint8_t log_bytes = num_bytes;
  if (log_bytes > 16) { log_bytes = 16; }
  /*
    time(&c_time);
    timeinfo = localtime(&c_time);
    strftime(time_str, 30, " %X ", timeinfo);

    printf("%s %s bytes: %3d | id: %3d | UART%d | ",
           prefix, time_str, num_bytes, msg_id, data[0]);
  */
  printf("%s bytes: %3d | id: %3d | UART%d | ",
         prefix, num_bytes, msg_id, data[0]);
  for (cnt = 1; cnt < log_bytes; cnt++) {
    printf("%02X ", data[cnt]);
    if (cnt >= 24 && cnt % 24 == 0 && cnt + 1 < log_bytes) {
      printf("\n                                                    ");
    }
  }
  printf("\n");
}

#endif

