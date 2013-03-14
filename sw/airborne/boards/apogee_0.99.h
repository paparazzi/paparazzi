#ifndef CONFIG_APOGEE_0_99_H
#define CONFIG_APOGEE_0_99_H

#define BOARD_APOGEE

/* Apogee has a 16MHz external clock and 168MHz internal. */
#define EXT_CLK 16000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PC0 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#define DefaultVoltageOfAdc(adc) (0.006185*adc)

/* Onboard ADCs */
/*
   ADC1 PB0/?ADC13
   ADC2 PB1/?ADC10
   ADC3 PB15/?ADC11
   ADC4 PC4/?ADC15
   BATT PA4/ADC4
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
