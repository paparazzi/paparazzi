
#ifndef MAX11040_HW_H
#define MAX11040_HW_H


#include "LPC21xx.h"
#include "ssp_hw.h"
#include "generated/airframe.h"
#include BOARD_CONFIG


#define MAXM_DIVISOR_128  2
#define MAXM_DIVISOR_256  3
#define MAXM_DIVISOR_512  4
#define MAXM_DIVISOR_1024 5

#define MAXM_DIVISOR MAXM_DIVISOR_512

/* ssp input clock 468.75kHz, clock that divided by SCR+1 */
#define SSP_CLOCK 468750

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA 0x01 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x00 << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)
#define SSP_PINSEL1_SSEL (2<<8)

#define SSP_Enable()     SetBit(SSPCR1, SSE);
#define SSP_Disable()    ClearBit(SSPCR1, SSE);
#define SSP_EnableRxi()  SetBit(SSPIMSC, RXIM)
#define SSP_DisableRxi() ClearBit(SSPIMSC, RXIM)
#define SSP_ClearRxi()   SetBit(SSPICR, RXIM);
#define SSP_EnableTxi()  SetBit(SSPIMSC, TXIM)
#define SSP_DisableTxi() ClearBit(SSPIMSC, TXIM)
#define SSP_ClearTxi()   SetBit(SSPICR, TXIM);
#define SSP_EnableRti()  SetBit(SSPIMSC, RTIM);
#define SSP_DisableRti() ClearBit(SSPIMSC, RTIM);
#define SSP_ClearRti()   SetBit(SSPICR, RTIC);

#define MaxmSelect() SetBit(MAXM_SS_IOCLR, MAXM_SS_PIN)
#define MaxmUnselect() SetBit(MAXM_SS_IOSET, MAXM_SS_PIN)


void max11040_hw_init(void);

#endif /* MAX11040_HW_H */

