#ifndef CONFIG_APOGEE_0_99_H
#define CONFIG_APOGEE_0_99_H

#define BOARD_APOGEE

/* Apogee has a 16MHz external clock and 168MHz internal. */
#define EXT_CLK 16000000
#define AHB_CLK 168000000

#define USE_OPENCM3 1

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_1_GPIO_PIN GPIO13
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_2_GPIO_PIN GPIO14
#define LED_2_AFIO_REMAP ((void)0)

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOA
#define LED_3_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_3_GPIO_PIN GPIO15
#define LED_3_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */

/* PB4, Camera power On/Off */
#define CAM_SW_GPIO GPIOB
#define CAM_SW_GPIO_CLK RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO
#define CAM_SW_GPIO_PIN GPIO_Pin_4
#define CAM_SW_AFIO_REMAP GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE)

/* PC2, Camera shot */
#define CAM_SH_GPIO GPIOC
#define CAM_SH_GPIO_CLK RCC_APB2Periph_GPIOC
#define CAM_SH_GPIO_PIN GPIO_Pin_2
#define CAM_SH_AFIO_REMAP ((void)0)

/* PC15, Camera video */
#define CAM_V_GPIO GPIOC
#define CAM_V_GPIO_CLK RCC_APB2Periph_GPIOC
#define CAM_V_GPIO_PIN GPIO_Pin_15
#define CAM_V_AFIO_REMAP ((void)0)

#define BEEPER_GPIO GPIOC
#define BEEPER_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define BEEPER_GPIO_PIN GPIO14
#define BEEPER_AFIO_REMAP ((void)0)

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#define DefaultVoltageOfAdc(adc) (0.006185*adc)

/* Onboard ADCs */
/*
   ADC1 PC3/ADC13
   ADC2 PC0/ADC10
   ADC3 PC1/ADC11
   ADC4 PC5/ADC15
   ADC6 PC2/ADC12
   BATT PC4/ADC14
*/
#define BOARD_ADC_CHANNEL_1 12
#define BOARD_ADC_CHANNEL_2 10
#define BOARD_ADC_CHANNEL_3 11
#define BOARD_ADC_CHANNEL_4 13 //15
#define BOARD_ADC_CHANNEL_5 14
// we can only use ADC1,2,3; the last channel is for bat monitoring
#define BOARD_ADC_CHANNEL_6 15 //13

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 0
#define ADC_2 1
#define ADC_3 2
#define ADC_4 3
#define ADC_5 4
#define ADC_6 5

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define BOARD_HAS_BARO 1

#define PWM_1_4_TIMER TIM3
#define PWM_1_4_RCC_TIM RCC_APB1ENR_TIM3EN
#define PWM_1_4_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_1_4_GPIO GPIOC
#define PWM_1_4_Pins GPIO6|GPIO7|GPIO8|GPIO9
#define PWM1_OC TIM_OC1
#define PWM2_OC TIM_OC2
#define PWM3_OC TIM_OC3
#define PWM4_OC TIM_OC4

#define PWM_5AND6_TIMER TIM4
#define PWM_5AND6_RCC_TIM RCC_APB1ENR_TIM4EN
#define PWM_5AND6_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_5AND6_GPIO GPIOB
#define PWM_5AND6_Pins GPIO6|GPIO7
#define PWM5_OC TIM_OC1
#define PWM6_OC TIM_OC2

#define PWM_7AND8_TIMER TIM5
#define PWM_7AND8_RCC_TIM RCC_APB1ENR_TIM5EN
#define PWM_7AND8_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_7AND8_GPIO GPIOA
#define PWM_7AND8_Pins GPIO0|GPIO1
#define PWM7_OC TIM_OC1
#define PWM8_OC TIM_OC2

#define PWM_9AND10_TIMER TIM5
#define PWM_9AND10_RCC_TIM RCC_APB1ENR_TIM5EN
#define PWM_9AND10_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_9AND10_GPIO GPIOA
#define PWM_9AND10_Pins GPIO2|GPIO3
#define PWM9_OC  TIM_OC3
#define PWM10_OC TIM_OC4

#define PWM_11_TIMER TIM2
#define PWM_11_RCC_TIM RCC_APB1ENR_TIM2EN
#define PWM_11_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_11_GPIO GPIOB
#define PWM_11_Pin GPIO3
#define PWM11_OC TIM_OC2

#endif /* CONFIG_APOGEE_0_99_H */
