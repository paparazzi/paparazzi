#ifndef CONFIG_OLIMEX_STM32_H103_H
#define CONFIG_OLIMEX_STM32_H103_H


#define AHB_CLK 72000000

/* this board uses a crystal for HSE */
//#define HSE_TYPE RCC_HSE_ON

/* Onboard LEDs */
#define LED_1_BANK
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOC
#define LED_1_GPIO_PIN GPIO_Pin_12



#endif /* CONFIG_OLIMEX_STM32_H103_H */
