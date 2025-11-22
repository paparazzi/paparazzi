#ifndef CONFIG_CHIMERA_1_00_H
#define CONFIG_CHIMERA_1_00_H

#define BOARD_CHIMERA

/**
 * ChibiOS board file
 */
#include "boards/chimera/chibios/v1.0/board.h"

/**
 * PPRZ definitions
 */

/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

/*
 * LEDs
 */
/* red, on PB12, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO12
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear

/* orange, on PB13, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear

/* green, on PD10, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOD
#define LED_3_GPIO_PIN GPIO10
#define LED_3_GPIO_ON gpio_set
#define LED_3_GPIO_OFF gpio_clear

/* yellow, on PD11, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOD
#define LED_4_GPIO_PIN GPIO11
#define LED_4_GPIO_ON gpio_set
#define LED_4_GPIO_OFF gpio_clear

/* AUX0, on PA5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOA
#define LED_5_GPIO_PIN GPIO5
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* AUX1, on PA3, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOA
#define LED_6_GPIO_PIN GPIO3
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* AUX2, on PA2, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOA
#define LED_7_GPIO_PIN GPIO2
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* AUX3, on PA0, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOA
#define LED_8_GPIO_PIN GPIO0
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear

/* AUX4, on PC3, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_9
#define USE_LED_9 0
#endif
#define LED_9_GPIO GPIOC
#define LED_9_GPIO_PIN GPIO3
#define LED_9_GPIO_ON gpio_set
#define LED_9_GPIO_OFF gpio_clear

/* AUX5, on PC2, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_10
#define USE_LED_10 0
#endif
#define LED_10_GPIO GPIOC
#define LED_10_GPIO_PIN GPIO2
#define LED_10_GPIO_ON gpio_set
#define LED_10_GPIO_OFF gpio_clear

/* AUX6, on PC6, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_11
#define USE_LED_11 0
#endif
#define LED_11_GPIO GPIOC
#define LED_11_GPIO_PIN GPIO6
#define LED_11_GPIO_ON gpio_set
#define LED_11_GPIO_OFF gpio_clear

/* AUX7, on PC7, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_12
#define USE_LED_12 0
#endif
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_PIN GPIO7
#define LED_12_GPIO_ON gpio_set
#define LED_12_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// AUX0
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN5
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO5
#endif

// AUX1
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN3
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO3
#endif

// AUX2
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN2
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO2
#endif

// AUX3
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN0
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO0
#endif

// AUX4
#if USE_ADC_5
#define AD1_5_CHANNEL ADC_CHANNEL_IN13
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO3
#endif

// AUX5
#if USE_ADC_6
#define AD1_6_CHANNEL ADC_CHANNEL_IN12
#define ADC_6 AD1_6
#define ADC_6_GPIO_PORT GPIOC
#define ADC_6_GPIO_PIN GPIO2
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_7
#define USE_ADC_7 1
#endif
#if USE_ADC_7
#define AD1_7_CHANNEL ADC_CHANNEL_IN4
#define ADC_7 AD1_7
#define ADC_7_GPIO_PORT GPIOA
#define ADC_7_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_7
#endif

/*
 * R1 = 3.3k
 * R2 = 22k
 * adc * (3.3 / 2^12) * ((R1 + R2) / R1)
 */
#define VBAT_R1 3300.0f
#define VBAT_R2 22000.0f
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*((VBAT_R1+VBAT_R2)/VBAT_R1)*adc)

//TODO configure DAC (ADC_1)

/*
 * PWM defines
 */
#if defined(LINE_SERVO1)
#if !defined(USE_PWM_SERVO1) && !defined(USE_ICU_SERVO1)
#define USE_PWM_SERVO1      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO1_TIM))
#endif
#define SERVO1              0
#define SERVO1_GPIO         PAL_PORT(LINE_SERVO1)
#define SERVO1_PIN          PAL_PAD(LINE_SERVO1)
#define SERVO1_AF           AF_LINE_SERVO1
#define SERVO1_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO1_TIM)
#define SERVO1_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO1_TIM)
#define SERVO1_CHANNEL      (SERVO1_TIM_CH-1)
#define SERVO1_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO1_TIM)
#endif

#if defined(LINE_SERVO2)
#if !defined(USE_PWM_SERVO2) && !defined(USE_ICU_SERVO2)
#define USE_PWM_SERVO2      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO2_TIM))
#endif
#define SERVO2              1
#define SERVO2_GPIO         PAL_PORT(LINE_SERVO2)
#define SERVO2_PIN          PAL_PAD(LINE_SERVO2)
#define SERVO2_AF           AF_LINE_SERVO2
#define SERVO2_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO2_TIM)
#define SERVO2_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO2_TIM)
#define SERVO2_CHANNEL      (SERVO2_TIM_CH-1)
#define SERVO2_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO2_TIM)
#endif

#if defined(LINE_SERVO3)
#if !defined(USE_PWM_SERVO3) && !defined(USE_ICU_SERVO3)
#define USE_PWM_SERVO3      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO3_TIM))
#endif
#define SERVO3              2
#define SERVO3_GPIO         PAL_PORT(LINE_SERVO3)
#define SERVO3_PIN          PAL_PAD(LINE_SERVO3)
#define SERVO3_AF           AF_LINE_SERVO3
#define SERVO3_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO3_TIM)
#define SERVO3_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO3_TIM)
#define SERVO3_CHANNEL      (SERVO3_TIM_CH-1)
#define SERVO3_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO3_TIM)
#endif

#if defined(LINE_SERVO4)
#if !defined(USE_PWM_SERVO4) && !defined(USE_ICU_SERVO4)
#define USE_PWM_SERVO4      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO4_TIM))
#endif
#define SERVO4              3
#define SERVO4_GPIO         PAL_PORT(LINE_SERVO4)
#define SERVO4_PIN          PAL_PAD(LINE_SERVO4)
#define SERVO4_AF           AF_LINE_SERVO4
#define SERVO4_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO4_TIM)
#define SERVO4_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO4_TIM)
#define SERVO4_CHANNEL      (SERVO4_TIM_CH-1)
#define SERVO4_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO4_TIM)
#endif

#if defined(LINE_SERVO5)
#if !defined(USE_PWM_SERVO5) && !defined(USE_ICU_SERVO5)
#define USE_PWM_SERVO5      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO5_TIM))
#endif
#define SERVO5              4
#define SERVO5_GPIO         PAL_PORT(LINE_SERVO5)
#define SERVO5_PIN          PAL_PAD(LINE_SERVO5)
#define SERVO5_AF           AF_LINE_SERVO5
#define SERVO5_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO5_TIM)
#define SERVO5_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO5_TIM)
#define SERVO5_CHANNEL      (SERVO5_TIM_CH-1)
#define SERVO5_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO5_TIM)
#endif

#if defined(LINE_SERVO6)
#if !defined(USE_PWM_SERVO6) && !defined(USE_ICU_SERVO6)
#define USE_PWM_SERVO6      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO6_TIM))
#endif
#define SERVO6              5
#define SERVO6_GPIO         PAL_PORT(LINE_SERVO6)
#define SERVO6_PIN          PAL_PAD(LINE_SERVO6)
#define SERVO6_AF           AF_LINE_SERVO6
#define SERVO6_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO6_TIM)
#define SERVO6_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO6_TIM)
#define SERVO6_CHANNEL      (SERVO6_TIM_CH-1)
#define SERVO6_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO6_TIM)
#endif

#if defined(LINE_SERVO7)
#if !defined(USE_PWM_SERVO7) && !defined(USE_ICU_SERVO7)
#define USE_PWM_SERVO7      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO7_TIM))
#endif
#define SERVO7              6
#define SERVO7_GPIO         PAL_PORT(LINE_SERVO7)
#define SERVO7_PIN          PAL_PAD(LINE_SERVO7)
#define SERVO7_AF           AF_LINE_SERVO7
#define SERVO7_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO7_TIM)
#define SERVO7_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO7_TIM)
#define SERVO7_CHANNEL      (SERVO7_TIM_CH-1)
#define SERVO7_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO7_TIM)
#endif

#if defined(LINE_SERVO8)
#if !defined(USE_PWM_SERVO8) && !defined(USE_ICU_SERVO8)
#define USE_PWM_SERVO8      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO8_TIM))
#endif
#define SERVO8              7
#define SERVO8_GPIO         PAL_PORT(LINE_SERVO8)
#define SERVO8_PIN          PAL_PAD(LINE_SERVO8)
#define SERVO8_AF           AF_LINE_SERVO8
#define SERVO8_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO8_TIM)
#define SERVO8_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO8_TIM)
#define SERVO8_CHANNEL      (SERVO8_TIM_CH-1)
#define SERVO8_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO8_TIM)
#endif

#if defined(LINE_SERVO9)
#if !defined(USE_PWM_SERVO9) && !defined(USE_ICU_SERVO9)
#define USE_PWM_SERVO9      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO9_TIM))
#endif
#define SERVO9              8
#define SERVO9_GPIO         PAL_PORT(LINE_SERVO9)
#define SERVO9_PIN          PAL_PAD(LINE_SERVO9)
#define SERVO9_AF           AF_LINE_SERVO9
#define SERVO9_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO9_TIM)
#define SERVO9_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO9_TIM)
#define SERVO9_CHANNEL      (SERVO9_TIM_CH-1)
#define SERVO9_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO9_TIM)
#endif

#if defined(LINE_SERVO10)
#if !defined(USE_PWM_SERVO10) && !defined(USE_ICU_SERVO10)
#define USE_PWM_SERVO10      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO10_TIM))
#endif
#define SERVO10              9
#define SERVO10_GPIO         PAL_PORT(LINE_SERVO10)
#define SERVO10_PIN          PAL_PAD(LINE_SERVO10)
#define SERVO10_AF           AF_LINE_SERVO10
#define SERVO10_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO10_TIM)
#define SERVO10_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO10_TIM)
#define SERVO10_CHANNEL      (SERVO10_TIM_CH-1)
#define SERVO10_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO10_TIM)
#endif

#if defined(LINE_SERVO11)
#if !defined(USE_PWM_SERVO11) && !defined(USE_ICU_SERVO11)
#define USE_PWM_SERVO11      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO11_TIM))
#endif
#define SERVO11              10
#define SERVO11_GPIO         PAL_PORT(LINE_SERVO11)
#define SERVO11_PIN          PAL_PAD(LINE_SERVO11)
#define SERVO11_AF           AF_LINE_SERVO11
#define SERVO11_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO11_TIM)
#define SERVO11_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO11_TIM)
#define SERVO11_CHANNEL      (SERVO11_TIM_CH-1)
#define SERVO11_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO11_TIM)
#endif

#if defined(LINE_SERVO12)
#if !defined(USE_PWM_SERVO12) && !defined(USE_ICU_SERVO12)
#define USE_PWM_SERVO12      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO12_TIM))
#endif
#define SERVO12              11
#define SERVO12_GPIO         PAL_PORT(LINE_SERVO12)
#define SERVO12_PIN          PAL_PAD(LINE_SERVO12)
#define SERVO12_AF           AF_LINE_SERVO12
#define SERVO12_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO12_TIM)
#define SERVO12_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO12_TIM)
#define SERVO12_CHANNEL      (SERVO12_TIM_CH-1)
#define SERVO12_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO12_TIM)
#endif

#if defined(LINE_SERVO13)
#if !defined(USE_PWM_SERVO13) && !defined(USE_ICU_SERVO13)
#define USE_PWM_SERVO13      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO13_TIM))
#endif
#define SERVO13              12
#define SERVO13_GPIO         PAL_PORT(LINE_SERVO13)
#define SERVO13_PIN          PAL_PAD(LINE_SERVO13)
#define SERVO13_AF           AF_LINE_SERVO13
#define SERVO13_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO13_TIM)
#define SERVO13_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO13_TIM)
#define SERVO13_CHANNEL      (SERVO13_TIM_CH-1)
#define SERVO13_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO13_TIM)
#endif

#if defined(LINE_SERVO14)
#if !defined(USE_PWM_SERVO14) && !defined(USE_ICU_SERVO14)
#define USE_PWM_SERVO14      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO14_TIM))
#endif
#define SERVO14              13
#define SERVO14_GPIO         PAL_PORT(LINE_SERVO14)
#define SERVO14_PIN          PAL_PAD(LINE_SERVO14)
#define SERVO14_AF           AF_LINE_SERVO14
#define SERVO14_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO14_TIM)
#define SERVO14_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO14_TIM)
#define SERVO14_CHANNEL      (SERVO14_TIM_CH-1)
#define SERVO14_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO14_TIM)
#endif

#if defined(LINE_SERVO15)
#if !defined(USE_PWM_SERVO15) && !defined(USE_ICU_SERVO15)
#define USE_PWM_SERVO15      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO15_TIM))
#endif
#define SERVO15              14
#define SERVO15_GPIO         PAL_PORT(LINE_SERVO15)
#define SERVO15_PIN          PAL_PAD(LINE_SERVO15)
#define SERVO15_AF           AF_LINE_SERVO15
#define SERVO15_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO15_TIM)
#define SERVO15_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO15_TIM)
#define SERVO15_CHANNEL      (SERVO15_TIM_CH-1)
#define SERVO15_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO15_TIM)
#endif

#if defined(LINE_SERVO16)
#if !defined(USE_PWM_SERVO16) && !defined(USE_ICU_SERVO16)
#define USE_PWM_SERVO16      !(CONCAT_BOARD_PARAM(USE_ICU_TIM, SERVO16_TIM))
#endif
#define SERVO16              15
#define SERVO16_GPIO         PAL_PORT(LINE_SERVO16)
#define SERVO16_PIN          PAL_PAD(LINE_SERVO16)
#define SERVO16_AF           AF_LINE_SERVO16
#define SERVO16_PWM_DRIVER   CONCAT_BOARD_PARAM(PWMD, SERVO16_TIM)  
#define SERVO16_ICU_DRIVER   CONCAT_BOARD_PARAM(ICUD, SERVO16_TIM)
#define SERVO16_CHANNEL      (SERVO16_TIM_CH-1)
#define SERVO16_PWM_CONF     CONCAT_BOARD_PARAM(pwmcfg, SERVO16_TIM)
#endif

/**
 * UART2 (with optional flow control activated by default)
 */
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_AF 7
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL TRUE
#endif

/**
 * UART3 (XBee slot), UART8 (GPS) and UART1 (Companion)
 * are configured as UART from ChibiOS board file by default
 */

#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_AF 7

#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_AF 7

#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO0
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO1
#define UART8_GPIO_AF 8

/**
 * SBUS / Spektrum port
 *
 * Recommended config:
 *
 * primary SBUS port is UART7, a.k.a. RC2 on Chimera board
 * secondary port (in dual driver) is UART4, a.k.a. RC1 on Chimera board
 *
 * primary Spektrum port is UART4, a.k.a. RC1 on Chimera board
 * secondary port is UART7, a.k.a. RC2 on Chimera board
 */

// In case, do dynamic config of UARTs
#define USE_UART7_RX TRUE
#ifndef USE_UART7_TX // may be used in half duplex mode
#define USE_UART7_TX FALSE
#endif
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_AF 8

#define USE_UART4_RX TRUE
#define USE_UART4_TX FALSE
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_AF 8

/* The line that is pulled low at power up to initiate the bind process
 * PC7: AUX7
 */
#define SPEKTRUM_BIND_PIN GPIO7
#define SPEKTRUM_BIND_PIN_PORT GPIOC

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/**
 * PPM radio defines
 *
 * available on RC1
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_2
#define PPM_TIMER ICUD5

/*
 * PWM input
 */
// PWM_INPUT 1 on PA0 (AUX3)
#define PWM_INPUT1_ICU            ICUD2
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO0
#define PWM_INPUT1_GPIO_AF        GPIO_AF1

// PWM_INPUT 2 on PC7 (AUX7)
#define PWM_INPUT2_ICU            ICUD8
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_2
#define PWM_INPUT2_GPIO_PORT      GPIOC
#define PWM_INPUT2_GPIO_PIN       GPIO7
#define PWM_INPUT2_GPIO_AF        GPIO_AF3

/**
 * I2C defines
 */
// Digital noise filter: 0 disabled, [0x1 - 0xF] enable up to n t_I2CCLK
#define STM32_CR1_DNF(n)          ((n & 0x0f) << 8)
// Timing register
#define I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(0U) | \
    STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(34U)  | STM32_TIMINGR_SCLL(86U))
#define I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(1U) | \
    STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(105U)  | STM32_TIMINGR_SCLL(153U))


#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif

#if I2C1_CLOCK_SPEED == 400000
#define I2C1_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_CFG_DEF { \
  .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#else
#error "Unknown I2C1 clock speed"
#endif


#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif

#if I2C2_CLOCK_SPEED == 400000
#define I2C2_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_CFG_DEF { \
  .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#else
#error "Unknown I2C2 clock speed"
#endif

/**
 * SPI Config
 */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5
#define SPI1_GPIO_PORT_SCK GPIO3
#define SPI1_GPIO_SCK GPIO3

// SLAVE0 on SPI connector
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15
// SLAVE1 on AUX0
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO5
// SLAVE2 on AUX1
#define SPI_SELECT_SLAVE2_PORT GPIOA
#define SPI_SELECT_SLAVE2_PIN GPIO3
// SLAVE3 on AUX2
#define SPI_SELECT_SLAVE3_PORT GPIOA
#define SPI_SELECT_SLAVE3_PIN GPIO2
// SLAVE4 on AUX3
#define SPI_SELECT_SLAVE4_PORT GPIOA
#define SPI_SELECT_SLAVE4_PIN GPIO0
// SLAVE5 on AUX4
#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO3

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/**
 * SDIO
 */
#define SDIO_D0_PORT GPIOC
#define SDIO_D0_PIN GPIO8
#define SDIO_D1_PORT GPIOC
#define SDIO_D1_PIN GPIO9
#define SDIO_D2_PORT GPIOC
#define SDIO_D2_PIN GPIO10
#define SDIO_D3_PORT GPIOC
#define SDIO_D3_PIN GPIO11
#define SDIO_CK_PORT GPIOC
#define SDIO_CK_PIN GPIO12
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN GPIO2
#define SDIO_AF 12
// bat monitoring for file closing
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_7_CHANNEL
// usb led status
#define SDLOG_USB_LED 4
#define SDLOG_USB_VBUS_PORT GPIOA
#define SDLOG_USB_VBUS_PIN GPIO9


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/**
 * For WS2812
 */
#define WS2812D1_GPIO GPIOC
#define WS2812D1_PIN  GPIO6
#define WS2812D1_AF 3
#define WS2812D1_CFG_DEF { \
  .dma_stream = STM32_PWM8_UP_DMA_STREAM, \
  .dma_channel = STM32_PWM8_UP_DMA_CHANNEL, \
  .dma_priority = STM32_PWM8_UP_DMA_PRIORITY, \
  .pwm_channel = 0, \
  .pwmp = &PWMD8 \
}

#endif /* CONFIG_CHIMERA_1_00_H */

