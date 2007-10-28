#ifndef BOOZ_LINK_MCU_HW_H
#define BOOZ_LINK_MCU_HW_H

/* FIXME: not sure what that does here !!!! */
#define Spi0InitBuf() {}

#define BoozLinkMcuSetUnavailable() { }
#define BoozLinkMcuSetAvailable()   { spi_message_received = TRUE; }



#endif /* BOOZ_LINK_MCU_HW_H */
