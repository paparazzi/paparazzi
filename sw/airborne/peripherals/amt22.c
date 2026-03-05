#include "amt22.h"


#ifndef AMT22_SPI_CDIV
#define AMT22_SPI_CDIV SPIDiv256
#endif

static bool amt22_checkbit(uint8_t p0, uint8_t p1);

static void amt22_thd(void* arg);

void amt22_init(amt22_t* amt, amt22_config_t* conf) {
  amt->config = conf;
  // Set up SPI peripheral and transaction
  amt->trans.slave_idx = amt->config->slave_idx;
  amt->trans.input_buf = amt->spi_input_buf;
  amt->trans.output_buf = amt->spi_output_buf;
  amt->trans.select = SPISelectUnselect;
  amt->trans.cpol = SPICpolIdleLow;
  amt->trans.cpha = SPICphaEdge1;
  amt->trans.dss = SPIDss8bit;
  amt->trans.bitorder = SPIMSBFirst;
  amt->trans.cdiv = AMT22_SPI_CDIV;
  amt->trans.before_cb = NULL;
  amt->trans.after_cb = NULL;
  amt->trans.status = SPITransDone;
  amt->position = 0;
  amt->turns = 0;


  if(amt->config->type == AMT22_12_SINGLE || amt->config->type == AMT22_14_SINGLE) {
    //single turn
    amt->trans.output_buf[0] = 0x00;
    amt->trans.output_buf[1] = 0x00;
    amt->trans.output_length = 2;
    amt->trans.input_length = 2;
  } else {
    //multi_turn
    amt->trans.output_buf[0] = 0x00;
    amt->trans.output_buf[1] = 0xA0;
    amt->trans.output_buf[2] = 0x00;
    amt->trans.output_buf[3] = 0x00;
    amt->trans.output_length = 4;
    amt->trans.input_length = 4;
  }

  pprz_bsem_init(&amt->bsem_amt22_read, true);
  pprz_mtx_init(&amt->mtx);
  pprz_thread_create(&amt->thd_handle, 512, "amt22", PPRZ_NORMAL_PRIO+1, amt22_thd, amt);
}

void amt22_periodic(amt22_t* amt) {
  // trigger read
  pprz_bsem_signal(&amt->bsem_amt22_read);
}


bool amt22_read(amt22_t* amt) {
  if(spi_blocking_transceive(amt->config->p, &amt->trans, 0.5) != SPITransSuccess) {
    return false;
  }

  uint8_t p0 = amt->trans.input_buf[0];
  uint8_t p1 = amt->trans.input_buf[1];

  if(!amt22_checkbit(p0,p1)) {
    return false;
  }

  uint16_t position = (p0 << 8 | p1) & 0x3fff;
  int16_t turns = 0;

  // 12 bits so shift 2
  if(amt->config->type == AMT22_12_SINGLE || amt->config->type == AMT22_12_MULTI) {
    position >>= 2;
  }

  if(amt->config->type == AMT22_12_MULTI || amt->config->type == AMT22_14_MULTI) {
    uint8_t t0 = amt->trans.input_buf[2];
    uint8_t t1 = amt->trans.input_buf[3];
    turns = (t0 << 8 | t1);
  }

  pprz_mtx_lock(&amt->mtx);
  amt->position = position;
  amt->turns = turns;
  pprz_mtx_unlock(&amt->mtx);
  return true;
}

static bool amt22_checkbit(uint8_t p0, uint8_t p1) {
  uint16_t data = ((p0 << 8 | p1) & 0x3fff) >> 2;
  uint8_t odd = 0;
  uint8_t even = 0;
  while (data)
  {
    even ^= data & 1;
    data >>= 1;
    odd ^= data & 1;
    data >>= 1;
  }
  even = !even;
  odd = !odd;
  if((even == ((p0 & 0x40) >> 6)) && (odd == ((p0 & 0x80) >> 7))) {
    return 1;
  } else {
    return 0;
  }
}

uint16_t amt22_get_position(amt22_t* amt) {
  pprz_mtx_lock(&amt->mtx);
  uint16_t pos = amt->position;
  pprz_mtx_unlock(&amt->mtx);
  return pos;
}

int16_t amt22_get_turns(amt22_t* amt) {
  pprz_mtx_lock(&amt->mtx);
  int16_t turns = amt->turns;
  pprz_mtx_unlock(&amt->mtx);
  return turns;
}


static void amt22_thd(void* arg) {
  amt22_t* amt = (amt22_t*)arg;

  while(true) {
    pprz_bsem_wait(&amt->bsem_amt22_read);
    amt22_read(amt);
  }
}