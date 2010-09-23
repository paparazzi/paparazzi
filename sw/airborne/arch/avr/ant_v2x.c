#include "ant_v2x.h"

#include <avr/io.h>
#include <avr/interrupt.h>

//#include "stdlib.h"
#include "string.h"
#include "std.h"
//#include "systime.h"
//#include "signalisation.h"
//#include "utils.h"

#include "ant_spi.h"

//#include "downlink.h"

volatile bool_t ant_v2x_data_available;
struct  Ant_V2xData ant_v2x_data;


#define MAG_S_RESET           0
#define MAG_S_UNINIT          1
#define MAG_S_READY           2
#define MAG_S_WAIT_MEAS       3
volatile uint8_t ant_v2x_status;

#define MAG_CS_IDLE              0
#define MAG_CS_READING           1
#define MAG_CS_WRITING           2
volatile uint8_t ant_v2x_com_status;


uint8_t ant_v2x_req[64];
uint8_t ant_v2x_req_len;
volatile uint8_t ant_v2x_req_idx;

uint8_t ant_v2x_res[128];
uint8_t ant_v2x_res_len;
volatile uint8_t ant_v2x_res_idx;

volatile uint8_t ant_v2x_periodic_count;

/******************************Ant_V2x define************************************/

#define ANT_V2X_DDR  DDRB
#define ANT_V2X_PORT PORTB
#define ANT_V2X_PIN  4

#define SYNC_FLAG  0xAA
#define TERMINATOR 0x00

#define GET_MODE_INFO       0x01
#define MOD_INFO_RESP       0x02
#define SET_DATA_COMPONENTS 0x03
#define GET_DATA            0x04
#define DATA_RESP           0x05
#define SET_CONFIG          0x06
#define GET_CONFIG          0x07
#define CONFIG_RESP         0x08
#define SAVE_CONFIG         0x09
#define START_CAL           0x0A
#define STOP_CAL            0x0B
#define GET_CAL_DATA        0x0C
#define CAL_DATA_RESP       0x0D
#define SET_CAL_DATA        0x0E

#define DATA_XRAW        0x01 // Slnt32 counts  32768 to 32767
#define DATA_YRAW        0x02 // Slnt32 counts  32768 to 32767
#define DATA_XCAL        0x03 // Float32 scaled to 1.0
#define DATA_YCAL        0x04 // Float32 scaled to 1.0
#define DATA_HEADING     0x05 // Float32 degrees 0.0 <B0> to 359.9 <B0>
#define DATA_MAGNITUDE   0x06 // Float32 scalebreak;
#define DATA_TEMPERATURE 0x07 // Float32 <B0> Celsius
#define DATA_DISTORTION  0x08 // Boolean
#define DATA_CAL_STATUS  0x09 // Boolean


extern void ant_v2x_periodic_initialise(void);
extern void ant_v2x_send_req ( const uint8_t* req, uint8_t len);
extern void ant_v2x_send_get_data ( void );


/***************************Initialisation ant_v2x*******************************/

void ant_v2x_init( void )
{
  SPI_master_init();
  /* set sync as ouptut */
  SetBit(ANT_V2X_DDR, ANT_V2X_PIN);
  /* pull it down */
  SetBit(ANT_V2X_PORT, ANT_V2X_PIN);

  ant_v2x_data_available = FALSE;
  ant_v2x_com_status = MAG_CS_IDLE;

  ant_v2x_data.heading = 0.;

  ant_v2x_reset();
}


void ant_v2x_periodic_initialise( void ) {
  static uint8_t init_status = 0;
  if (ant_v2x_com_status != MAG_CS_IDLE)
    return;
  switch (init_status) {
  case 0 :  /* set data response format */
    {
      const uint8_t req[] = {SET_DATA_COMPONENTS, 0x08, DATA_XRAW, DATA_YRAW, DATA_XCAL, DATA_YCAL, DATA_HEADING, DATA_MAGNITUDE, DATA_DISTORTION, DATA_CAL_STATUS };
      ant_v2x_send_req(req, sizeof(req));
    }
    break;
  case 1 :    /*  set little endian */
    {
      const uint8_t req[] = {SET_CONFIG, 0x06, 0x00};
      ant_v2x_send_req(req, sizeof(req));
    }
    break;
  case 2 :
    {          /* set period */
      const uint8_t req[] = {SET_CONFIG, 0x05, 0x07};
      ant_v2x_send_req(req, sizeof(req));
    }
    break;
  default:
    ant_v2x_status = MAG_S_READY;
  }
  init_status++;
}
void ant_v2x_periodic( void ) {    /* Run initialisation and communication request */
  switch (ant_v2x_status) {
  case MAG_S_RESET:
    SetBit(ANT_V2X_PORT, ANT_V2X_PIN);
    ant_v2x_status = MAG_S_UNINIT;
    break;
  case MAG_S_UNINIT:
    ant_v2x_periodic_initialise();
    break;
  case MAG_S_READY:                      /* Ready to receive request */
    /*GREEN_LED_ON();*/
    /*YELLOW_LED_OFF();*/
    //  if (ant_v2x_data_available) return
    ant_v2x_send_get_data();
    ant_v2x_status = MAG_S_WAIT_MEAS;
    ant_v2x_periodic_count = 0;
    break;
  case MAG_S_WAIT_MEAS: {     /* Waiting for measures */
    ant_v2x_periodic_count++;
    if (ant_v2x_periodic_count > 5) {
      ant_v2x_com_status = MAG_CS_READING;
      SPI_start();
      ant_v2x_res_idx = 0;
      ant_v2x_res_len = 43;
      SPI_transmit(0x00);
    }
  }
    break;
  }
}

/*****************************Ant_V2x reset****************************************/
void ant_v2x_reset (void){
  ClearBit(ANT_V2X_PORT, ANT_V2X_PIN);
  ant_v2x_status = MAG_S_RESET;
  ant_v2x_com_status = MAG_CS_IDLE;
}

/**************************Ant_V2x data communication******************************/
void ant_v2x_send_get_data ( void ) {
  const uint8_t req[] = {GET_DATA};
  ant_v2x_send_req(req, sizeof(req));
}

/********************************Request procedure***********************************/
void ant_v2x_send_req(const uint8_t* req, uint8_t len) {
  memcpy(ant_v2x_req, req, len);
  ant_v2x_req_len = len;
  ant_v2x_req_idx = 0;
  ant_v2x_com_status = MAG_CS_WRITING;
  SPI_start();
  SPI_transmit(SYNC_FLAG);/* transmit SYNC_FLAG first for every beginning of transmition */
}

void ant_v2x_read_data( void ) {
  ant_v2x_data.xraw = *(int32_t*)&ant_v2x_res[4];
  ant_v2x_data.yraw = *(int32_t*)&ant_v2x_res[9];
  ant_v2x_data.xcal = *(float*)&ant_v2x_res[14];
  ant_v2x_data.ycal = *(float*)&ant_v2x_res[19];
  ant_v2x_data.heading = *(float*)&ant_v2x_res[24];
  ant_v2x_data.magnitude = *(float*)&ant_v2x_res[29];
  ant_v2x_data.temp = *(float*)&ant_v2x_res[34];
  ant_v2x_data.distor = *(int8_t*)&ant_v2x_res[39];
  ant_v2x_data.cal_status = *(int8_t*)&ant_v2x_res[41];
}


#define SPI_SIG_ON_WRITING() {                                          \
    uint8_t c __attribute__ ((unused)) = SPI_read();                    \
    if (ant_v2x_req_idx < ant_v2x_req_len) {                            \
      SPI_transmit(ant_v2x_req[ant_v2x_req_idx]);                       \
    }                                                                   \
    else if (ant_v2x_req_idx == ant_v2x_req_len) {                      \
      SPI_transmit(TERMINATOR);                                         \
    }                                                                   \
    else {                                                              \
      ant_v2x_com_status = MAG_CS_IDLE;                                 \
      SPI_stop();                                                       \
    }                                                                   \
    ant_v2x_req_idx++;                                                  \
  }


static uint8_t nb_retry = 0;
#define MAX_RETRY 10

#define SPI_SIG_ON_READING() {                                         \
    ant_v2x_res[ant_v2x_res_idx] = SPI_read();                         \
    if (ant_v2x_res_idx == 0) {                                        \
      if (nb_retry > MAX_RETRY) {                                      \
        ant_v2x_reset();                                               \
        nb_retry = 0;                                                  \
        /*YELLOW_LED_ON();*/					       \
        goto sig_exit;                                                 \
      }                                                                \
      if (ant_v2x_res[ant_v2x_res_idx] != SYNC_FLAG) {                 \
        nb_retry++;                                                    \
        SPI_transmit(0x00);                                            \
        goto sig_exit;                                                 \
      }                                                                \
      else {                                                           \
        nb_retry = 0;                                                  \
      }                                                                \
    }                                                                  \
    ant_v2x_res_idx++;                                                 \
    if (ant_v2x_res_idx < ant_v2x_res_len) {                           \
      SPI_transmit(0x00);                                              \
    }                                                                  \
    else {                                                             \
      ant_v2x_com_status = MAG_CS_IDLE;                                \
      SPI_stop();                                                      \
      ant_v2x_status = MAG_S_READY;                                    \
      ant_v2x_data_available = TRUE;                                   \
      /*GREEN_LED_OFF();*/					       \
    }                                                                  \
  }                                                                    \

SIGNAL(SIG_SPI) {
  switch ( ant_v2x_com_status) {
  case MAG_CS_WRITING:
    SPI_SIG_ON_WRITING();
    break;
  case MAG_CS_READING:
    SPI_SIG_ON_READING();
  }
 sig_exit:
  /*GREEN_LED_OFF();*/
  asm("nop");
}
