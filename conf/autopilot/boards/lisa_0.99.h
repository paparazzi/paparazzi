#ifndef CONFIG_LISA_V1_0_H
#define CONFIG_LISA_V1_0_H


#define AHB_CLK 72000000

/* Lisa uses an external clock instead of a crystal */
#define HSE_TYPE_EXT_CLK

/* Onboard LEDs */
#define LED_1_BANK 
#define LED_STP08

// FIXME, this is just to make it compile
#define POWER_SWITCH_LED 5



#endif /* CONFIG_LISA_V1_0_H */
