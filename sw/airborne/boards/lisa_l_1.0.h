#ifndef CONFIG_LISA_V1_0_H
#define CONFIG_LISA_V1_0_H

#define BOARD_LISA_L

#define AHB_CLK 72000000

/* Lisa uses an external clock instead of a crystal */
#define HSE_TYPE_EXT_CLK
#define STM32_RCC_MODE RCC_HSE_Bypass
#define STM32_PLL_MULT RCC_PLLMul_9

/* Onboard LEDs */
#define LED_1_BANK
#define LED_STP08

// FIXME, this is just to make it compile
#define POWER_SWITCH_LED 5

/* configuration for aspirin - and more generaly IMUs */
#define IMU_ACC_DRDY_RCC_GPIO         RCC_APB2Periph_GPIOD
#define IMU_ACC_DRDY_GPIO             GPIOD
#define IMU_ACC_DRDY_GPIO_PORTSOURCE  GPIO_PortSourceGPIOD


#define ADC_CHANNEL_VSUPPLY 2
#define DefaultVoltageOfAdc(adc) (0.0059*adc)

#define BOARD_HAS_BARO

#endif /* CONFIG_LISA_V1_0_H */
