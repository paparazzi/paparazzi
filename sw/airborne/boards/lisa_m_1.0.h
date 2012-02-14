#ifndef CONFIG_LISA_M_1_0_H
#define CONFIG_LISA_M_1_0_H

#define BOARD_LISA_M

/* Lisa/M has a 12MHz external clock and 72MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 72000000

/* Onboard LEDs */
#define LED_1_BANK
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_CLK RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN
#define LED_1_GPIO_PIN GPIO4
#define LED_1_AFIO_REMAP AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST

#define LED_2_BANK
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_2_GPIO_PIN GPIO5
#define LED_2_AFIO_REMAP ((void)0)

#define LED_3_BANK
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_3_GPIO_PIN GPIO2
#define LED_3_AFIO_REMAP ((void)0)


/* configuration for aspirin - and more generaly IMUs */
#define IMU_ACC_DRDY_RCC_GPIO         RCC_APB2Periph_GPIOB
#define IMU_ACC_DRDY_GPIO             GPIOB
#define IMU_ACC_DRDY_GPIO_PORTSOURCE  GPIO_PortSourceGPIOB

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 2
#endif
#define DefaultVoltageOfAdc(adc) (0.00485*adc)

/* Onboard ADCs */
/*
   ADC1 PC3/ADC13
   ADC2 PA0/ADC0
   ADC3 PC0/ADC10
   ADC4 PC1/ADC11
   ADC5 PC5/ADC15
   ADC6 PA1/ADC1
   ADC7 PC2/ADC12
   BATT PC4/ADC14
*/
#define BOARD_ADC_CHANNEL_1 ADC_Channel_13
#define BOARD_ADC_CHANNEL_2 ADC_Channel_0
// FIXME - removed for now and used for battery monitoring
//#define BOARD_ADC_CHANNEL_3 ADC_Channel_10
#define BOARD_ADC_CHANNEL_3 ADC_Channel_14
#define BOARD_ADC_CHANNEL_4 ADC_Channel_11

#define BOARD_HAS_BARO

#define USE_OPENCM3 1

// not needed with USE_OPENCM3:
//#define HSE_TYPE_EXT_CLK
//#define STM32_RCC_MODE RCC_HSE_ON
//#define STM32_PLL_MULT RCC_PLLMul_6

// Remap the servos 5 and 6 to TIM5 CH1 and CH2
#define REMAP_SERVOS_5AND6 1

#endif /* CONFIG_LISA_M_1_0_H */
