
#define ReceiveUart(cb) \
  SIGNAL( SIG_UART_RECV ) { \
    uint8_t c = UDR; \
    cb(c); \
}
