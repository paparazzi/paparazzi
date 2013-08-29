#ifndef CONFIG_LISA_M_2_0_H
#define CONFIG_LISA_M_2_0_H

#include "boards/lisa_m_common.h"

#define BOARD_LISA_M

/* Lisa/M has a 12MHz external clock and 72MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 72000000

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_CLK RCC_APB2ENR_IOPAEN
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_CLK RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN
#define LED_2_GPIO_PIN GPIO4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_3_GPIO_PIN GPIO2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* red, shared with ADC15 (ADC_4 on connector ANALOG2) */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_4_GPIO_PIN GPIO5
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

/* green, on PC15 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOC
#define LED_5_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_5_GPIO_PIN GPIO15
#define LED_5_GPIO_ON gpio_clear
#define LED_5_GPIO_OFF gpio_set
#define LED_5_AFIO_REMAP ((void)0)

/*
 * LEDs not populated by default
 */
/* PC3, ADC13 on ADC_1 */
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_6_GPIO_PIN GPIO3
#define LED_6_GPIO_ON gpio_clear
#define LED_6_GPIO_OFF gpio_set
#define LED_6_AFIO_REMAP ((void)0)

/* PC0, ADC10 on ADC_2 */
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_7_GPIO_PIN GPIO0
#define LED_7_GPIO_ON gpio_clear
#define LED_7_GPIO_OFF gpio_set
#define LED_7_AFIO_REMAP ((void)0)

/* PC1, ADC11 on ADC_3 */
#define LED_8_GPIO GPIOC
#define LED_8_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_8_GPIO_PIN GPIO1
#define LED_8_GPIO_ON gpio_clear
#define LED_8_GPIO_OFF gpio_set
#define LED_8_AFIO_REMAP ((void)0)


/*
 * not actual LEDS, used as GPIOs
 */

/* PB1, DRDY on EXT SPI connector*/
#define LED_BODY_GPIO GPIOB
#define LED_BODY_GPIO_CLK RCC_APB2ENR_IOPBEN
#define LED_BODY_GPIO_PIN GPIO1
#define LED_BODY_GPIO_ON gpio_set
#define LED_BODY_GPIO_OFF gpio_clear
#define LED_BODY_AFIO_REMAP ((void)0)

/* PC12, on GPIO connector*/
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_12_GPIO_PIN GPIO12
#define LED_12_GPIO_ON gpio_clear
#define LED_12_GPIO_OFF gpio_set
#define LED_12_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#define DefaultVoltageOfAdc(adc) (0.0045*adc)

/* Onboard ADCs */
/*
   ADC1 PC3/ADC13
   ADC2 PC0/ADC10
   ADC3 PC1/ADC11
   ADC4 PC5/ADC15
   ADC6 PC2/ADC12
   BATT PC4/ADC14
*/
#define BOARD_ADC_CHANNEL_1 13
#define BOARD_ADC_CHANNEL_2 10
#define BOARD_ADC_CHANNEL_3 11
// we can only use ADC1,2,3; the last channel is for bat monitoring
#define BOARD_ADC_CHANNEL_4 14

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 0
#define ADC_2 1
#define ADC_3 2

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
#endif

/* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c */
// FIXME, this is not very nice, is also locm3 lib specific
#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) {                                          \
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT,                               \
		  GPIO_CNF_INPUT_ANALOG,                                \
		  GPIO3 | GPIO0 | GPIO1 | GPIO4);                       \
  }
#endif // USE_AD1


#define BOARD_HAS_BARO 1

#endif /* CONFIG_LISA_M_2_0_H */
