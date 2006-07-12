#ifndef CONFIG_DISCBOARD_H
#define CONFIG_DISCBOARD_H

#define LED_1_BANK A
#define LED_1_PIN  7

#define LED_2_BANK A
#define LED_2_PIN  6

#define LED_3_BANK A
#define LED_3_PIN  5

#define LED_4_BANK A
#define LED_4_PIN  4

#define LED_5_BANK A
#define LED_5_PIN  3

#define LED_6_BANK A
#define LED_6_PIN  2

#define LED_7_BANK A
#define LED_7_PIN  1

#define LED_8_BANK A
#define LED_8_PIN  0

/* clock in MHz */
#define CLOCK 16

/* PPM input on mega128 ICP  */
#define PPM_DDR DDRD
#define PPM_PORT PORTD
#define PPM_PIN  PD4

#endif /* CONFIG_DISCBOARD_H */
