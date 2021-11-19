#ifndef CONFIG_LIA_1_1_H
#define CONFIG_LIA_1_1_H

#include "boards/lisa_m_common.h"

#define BOARD_LIA

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
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP {                            \
    rcc_periph_clock_enable(RCC_AFIO);                  \
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;  \
  }

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

#if USE_LED_3 && USE_ADC_6
#error "You can't use LED_3 and ADC_6 at the same time"
#endif

/* red, shared with ADC15 (ADC_4 on connector ANALOG2) */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO5
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

#if USE_LED_4 && USE_ADC_4
#error "You can't use LED_4 and ADC_4 at the same time"
#endif

/* green, on PC15 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOC
#define LED_5_GPIO_PIN GPIO15
#define LED_5_GPIO_ON gpio_clear
#define LED_5_GPIO_OFF gpio_set
#define LED_5_AFIO_REMAP ((void)0)

/*
 * LEDs not populated by default
 */
/* PC3, ADC13 on ADC_1 */
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_PIN GPIO3
#define LED_6_GPIO_ON gpio_clear
#define LED_6_GPIO_OFF gpio_set
#define LED_6_AFIO_REMAP ((void)0)

/* PC0, ADC10 on ADC_2 */
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_PIN GPIO0
#define LED_7_GPIO_ON gpio_clear
#define LED_7_GPIO_OFF gpio_set
#define LED_7_AFIO_REMAP ((void)0)

/* PC1, ADC11 on ADC_3 */
#define LED_8_GPIO GPIOC
#define LED_8_GPIO_PIN GPIO1
#define LED_8_GPIO_ON gpio_clear
#define LED_8_GPIO_OFF gpio_set
#define LED_8_AFIO_REMAP ((void)0)


/*
 * not actual LEDS, used as GPIOs
 */

/* PB1, DRDY on EXT SPI connector*/
#define LED_BODY_GPIO GPIOB
#define LED_BODY_GPIO_PIN GPIO1
#define LED_BODY_GPIO_ON gpio_set
#define LED_BODY_GPIO_OFF gpio_clear
#define LED_BODY_AFIO_REMAP ((void)0)

/* PC12, on GPIO connector*/
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_PIN GPIO12
#define LED_12_GPIO_ON gpio_clear
#define LED_12_GPIO_OFF gpio_set
#define LED_12_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#define DefaultVoltageOfAdc(adc) (0.0045*adc)

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif


#endif /* CONFIG_LIA_1_1_H */
