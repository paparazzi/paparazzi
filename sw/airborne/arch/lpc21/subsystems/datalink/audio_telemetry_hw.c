#include "audio_telemetry.h"
#include "types.h"
#include "armVIC.h"

uint8_t           audio_telemetry_nb_ovrn;
uint8_t           tx_head;
volatile uint8_t  tx_tail;
uint8_t           tx_buf[ TX_BUF_SIZE ];
uint8_t ck_a, ck_b;
uint8_t tx_byte;
uint8_t tx_bit_idx;


#define NB_STATE 2
#define NB_PHASE 2

static const uint16_t audio_telemetry_sample[NB_STATE][NB_PHASE][SAMPLES_PER_PERIOD] = {
  { {512, 1023, 512, 1},
    {512 , 1 , 512 , 1023}
  },
  { {512, 874, 1023, 874},
    {512 , 150 , 1 , 150}
  }
};

static uint8_t audio_telemetry_sample_idx = 0;
static uint8_t audio_telemetry_phase = 0;


static inline uint8_t get_next_bit(void)
{
  uint8_t ret;
  /*  start bit         */
  if (tx_bit_idx == 0) {
    ret = 0;
  }
  /*  data bits         */
  else if (tx_bit_idx < 9) {
    ret = tx_byte & 0x01;
    tx_byte >>= 1;
  }
  /* stop_bit           */
  else {
    ret = 1;
  }
  tx_bit_idx++;

  if (tx_bit_idx >= 10) {
    /*  if we have nothing left to transmit */
    if (tx_head == tx_tail) {
      /* hack to stay with data = 1            */
      tx_bit_idx--;
    } else {
      /* else load next byte                  */
      tx_byte = tx_buf[tx_tail];
      tx_bit_idx = 0;
      tx_tail++;
      if (tx_tail >= TX_BUF_SIZE) {
        tx_tail = 0;
      }
    }
  }
  return ret;
}

void TIMER1_ISR(void)
{
  ISR_ENTRY();
  static uint8_t audio_telemetry_bit;

  DACR = audio_telemetry_sample[audio_telemetry_bit][audio_telemetry_phase][audio_telemetry_sample_idx] << 6;
  audio_telemetry_sample_idx++;
  if (audio_telemetry_sample_idx == SAMPLES_PER_PERIOD) {
    audio_telemetry_sample_idx = 0;
    audio_telemetry_phase ^= audio_telemetry_bit;
    audio_telemetry_bit =  get_next_bit();
  }

  /* clear interrupt */
  T1IR = TIR_MR0I;
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}



