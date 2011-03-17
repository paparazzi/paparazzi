#ifndef CONFIG_LISA_M_1_0_H
#define CONFIG_LISA_M_1_0_H


#define AHB_CLK 72000000

/* Onboard LEDs */
#define LED_1_BANK
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOC
#define LED_1_GPIO_PIN GPIO_Pin_13
/* doesnt work */
//#define LED_1_BANK
//#define LED_1_GPIO GPIOB
//#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOB
//#define LED_1_GPIO_PIN GPIO_Pin_4


/* configuration for aspirin - and more generaly IMUs */
#define IMU_ACC_DRDY_RCC_GPIO         RCC_APB2Periph_GPIOB
#define IMU_ACC_DRDY_GPIO             GPIOB
#define IMU_ACC_DRDY_GPIO_PORTSOURCE  GPIO_PortSourceGPIOB


#define ADC_CHANNEL_VSUPPLY 2
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


#endif /* CONFIG_LISA_M_1_0_H */
