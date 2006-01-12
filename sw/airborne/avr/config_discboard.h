#ifndef CONFIG_DISCBOARD_H
#define CONFIG_DISCBOARD_H

#define LED_1_BANK A
#define LED_1_PIN  6

#define LED_2_BANK A
#define LED_2_PIN  7

/* clock in MHz */
#define CLOCK 16

/* PPM input on mega128 ICP  */
#define PPM_DDR  DDRD
#define PPM_PORT PORTD
#define PPM_PIN  PD4

#endif /* CONFIG_DISCBOARD_H */
