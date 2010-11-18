#ifndef CONFIG_LISA_M_1_0_H
#define CONFIG_LISA_M_1_0_H


#define AHB_CLK 72000000

/* Onboard LEDs */
#define LED_1_BANK
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOB
#define LED_1_GPIO_PIN GPIO_Pin_4

#define LED_2_BANK
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_CLK RCC_APB2Periph_GPIOC
#define LED_2_GPIO_PIN GPIO_Pin_13

/* configuration for aspirin - and more generaly IMUs */
#define IMU_ACC_DRDY_RCC_GPIO         RCC_APB2Periph_GPIOB
#define IMU_ACC_DRDY_GPIO             GPIOB
#define IMU_ACC_DRDY_GPIO_PORTSOURCE  GPIO_PortSourceGPIOB

#define ADC_CHANNEL_VSUPPLY 4
#define DefaultVoltageOfAdc(adc) (0.01787109375*adc)


#endif /* CONFIG_LISA_M_1_0_H */
