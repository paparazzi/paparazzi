#ifndef LCD_DOGM_H
#define LCD_DOGM_H

#include <stdbool.h>
#include "std.h"
#include "mcu_periph/spi.h"
#include "led.h"

/* EA DOGM163, 3 line LCD at 3.3V */
#define DOGM_FUN_SET_1  0x39
#define DOGM_BIAS_SET   0x15
#define DOGM_PWR_CTRL   0x55
#define DOGM_FOLLOWER   0x6E
#define DOGM_CONTRAST   0x70
#define DOGM_FUN_SET_2  0x38
#define DOGM_DISP_ON  0x0C
#define DOGM_CLEAR  0x01
#define DOGM_ENTRY_MODE 0x06

extern void lcd_cmd(uint8_t command);
extern void lcd_data(uint8_t data);
extern void lcd_dogm_init(void);

#endif /* LCD_DOGM_H */

