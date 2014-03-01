#ifndef CONFIG_OLIMEX_STM32_H103_H
#define CONFIG_OLIMEX_STM32_H103_H

/* Olimex STM32-H103 board has an 8MHz external clock and 72MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 72000000

/* Onboard LEDs */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO_Pin_12

#endif /* CONFIG_OLIMEX_STM32_H103_H */
