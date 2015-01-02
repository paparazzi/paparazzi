#ifndef MICROMAG_FW_H
#define MICROMAG_FW_H


#include "std.h"
#define MM_NB_AXIS 3

extern void micromag_init(void);
extern void micromag_read(void);

extern void micromag_reset(void);
extern void micromag_periodic(void);
extern void micromag_event(void);

#define MM_IDLE            0
#define MM_BUSY            1
#define MM_SENDING_REQ     2
#define MM_WAITING_EOC     3
#define MM_GOT_EOC         4
#define MM_READING_RES     5
#define MM_DATA_AVAILABLE  6

/* ssp input clock 468.75kHz, clock that divided by SCR+1 */
#define SSP_CLOCK 468750

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#define SS_PIN   20
#define SS_IODIR IO0DIR
#define SS_IOSET IO0SET
#define SS_IOCLR IO0CLR

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)

#define SSP_Enable()     SetBit(SSPCR1, SSE);
#define SSP_Disable()    ClearBit(SSPCR1, SSE);
#define SSP_EnableRxi()  SetBit(SSPIMSC, RXIM)
#define SSP_DisableRxi() ClearBit(SSPIMSC, RXIM)
#define SSP_EnableTxi()  SetBit(SSPIMSC, TXIM)
#define SSP_DisableTxi() ClearBit(SSPIMSC, TXIM)
#define SSP_EnableRti()  SetBit(SSPIMSC, RTIM);
#define SSP_DisableRti() ClearBit(SSPIMSC, RTIM);
#define SSP_ClearRti()   SetBit(SSPICR, RTIC);

extern volatile uint8_t micromag_status;
extern volatile int16_t micromag_values[MM_NB_AXIS];

#endif /* MICROMAG_H */
