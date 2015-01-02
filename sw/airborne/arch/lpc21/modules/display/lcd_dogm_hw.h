#ifndef LCD_DOGM_HW_H
#define LCD_DOGM_HW_H

#include <stdbool.h>
#include "std.h"
#include "mcu_periph/spi.h"
#include "led.h"


#define LCDDOGM_SS_PORT 0
#define LCDDOGM_SS_PIN 20

#define LCDDOGM_RS_PORT 0
#define LCDDOGM_RS_PIN 16

#define LCDDOGM_IO__(port, reg) IO ## port ## reg
#define LCDDOGM_IO_(port, reg) LCDDOGM_IO__(port, reg)

#define LCDDOGM_SS_IOCLR LCDDOGM_IO_(LCDDOGM_SS_PORT, CLR)
#define LCDDOGM_SS_IODIR LCDDOGM_IO_(LCDDOGM_SS_PORT, DIR)
#define LCDDOGM_SS_IOSET LCDDOGM_IO_(LCDDOGM_SS_PORT, SET)

#define LCDDOGM_RS_IOCLR LCDDOGM_IO_(LCDDOGM_RS_PORT, CLR)
#define LCDDOGM_RS_IODIR LCDDOGM_IO_(LCDDOGM_RS_PORT, DIR)
#define LCDDOGM_RS_IOSET LCDDOGM_IO_(LCDDOGM_RS_PORT, SET)

#define lcddogmSelect() { \
    SetBit(LCDDOGM_SS_IOCLR, LCDDOGM_SS_PIN); \
  }

#define lcddogmUnselect() { \
    SetBit(LCDDOGM_SS_IOSET, LCDDOGM_SS_PIN); \
  }

#define lcddogmCmdMode() {  \
    SetBit(LCDDOGM_RS_IOCLR, LCDDOGM_RS_PIN); \
  }

#define lcddogmDataMode() { \
    SetBit(LCDDOGM_RS_IOSET, LCDDOGM_RS_PIN); \
  }

extern void lcd_spi_tx(uint8_t data);
extern void lcd_dogm_init_hw(void);


#endif /* LCD_DOGM_HW_H */

