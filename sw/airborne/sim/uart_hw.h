#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>

#define STDINOUT_BUFFER_SIZE 256
#define FD_STDIN 0

extern char stdinout_buffer[STDINOUT_BUFFER_SIZE];
extern uint8_t stdinout_rx_insert_idx;
extern uint8_t stdinout_rx_extract_idx;


static inline bool StdInOutChAvailable(void) {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(FD_STDIN, &fds);
  select(1, &fds, NULL, NULL, &tv);
  if (FD_ISSET(FD_STDIN, &fds)) {
    char tmp_buf[STDINOUT_BUFFER_SIZE];
    uint8_t n = read(FD_STDIN, tmp_buf, STDINOUT_BUFFER_SIZE);
    unsigned int i;
    for(i = 0; i < n; i++) {
      stdinout_buffer[stdinout_rx_insert_idx] = tmp_buf[i];
      stdinout_rx_insert_idx++; /* Auto overflow */
    }
  }
  return (stdinout_rx_insert_idx != stdinout_rx_extract_idx);
}

#define StdInOutTransmit(_char) putchar(_char)
#define StdInOutGetch() ({ \
  assert(stdinout_rx_insert_idx != stdinout_rx_extract_idx); \
  stdinout_buffer[stdinout_rx_extract_idx++]; \
})
