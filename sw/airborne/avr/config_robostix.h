#ifndef CONFIG_ROBOSTIX_H
#define CONFIG_ROBOSTIX_H

#define LED_1_BANK G
#define LED_1_PIN  4

#define LED_2_BANK G
#define LED_2_PIN  3

#define LED_3_BANK B
#define LED_3_PIN  4

/* clock in MHz */
#define CLOCK 16

/* PPM input on mega128 ICP  */
#define PPM_DDR  DDRD
#define PPM_PORT PORTD
#define PPM_PIN  PD4

#endif /* CONFIG_ROBOSTIX_H */
