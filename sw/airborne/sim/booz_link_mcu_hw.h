#ifndef BOOZ_LINK_MCU_HW_H
#define BOOZ_LINK_MCU_HW_H

#if 0
/* FIXME: not sure what that does here !!!! */
#define Spi0InitBuf() {}

#define BoozLinkMcuSetUnavailable() { }
#define BoozLinkMcuSetAvailable()   { spi_message_received = TRUE; }
#endif


#ifdef  BOOZ_FILTER_MCU

#define BoozLinkMcuHwSend() {}

#endif


#ifdef  BOOZ_CONTROLLER_MCU

#define BoozLinkMcuHwRestart() {}

#endif

#endif /* BOOZ_LINK_MCU_HW_H */
