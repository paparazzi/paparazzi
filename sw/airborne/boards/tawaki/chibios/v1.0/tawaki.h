#ifndef CONFIG_TAWAKI_1_00_H
#define CONFIG_TAWAKI_1_00_H

#define BOARD_TAWAKI

/**
 * ChibiOS board file
 */
#include "boards/tawaki/chibios/v1.0/board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
/* red, on PD15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOD
#define LED_1_GPIO_PIN GPIO15
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear

/* orange, on PA10, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_PIN GPIO10
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear

/* green, on PC7, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO7
#define LED_3_GPIO_ON gpio_set
#define LED_3_GPIO_OFF gpio_clear

/* yellow, on PD10, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOD
#define LED_4_GPIO_PIN GPIO10
#define LED_4_GPIO_ON gpio_set
#define LED_4_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// AUXa1
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN0
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO0
#endif

// AUXa2
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN1
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO1
#endif

// AUXa3
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN2
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO2
#endif

// AUXa4
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN6
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO6
#endif

// AUXb1
#if USE_ADC_5
#define AD1_5_CHANNEL ADC_CHANNEL_IN3
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOA
#define ADC_5_GPIO_PIN GPIO3
#endif

// AUXb2
#if USE_ADC_6
#define AD1_6_CHANNEL ADC_CHANNEL_IN7
#define ADC_6 AD1_6
#define ADC_6_GPIO_PORT GPIOA
#define ADC_6_GPIO_PIN GPIO7
#endif

// AUXb3
#if USE_ADC_7
#define AD1_7_CHANNEL ADC_CHANNEL_IN8
#define ADC_7 AD1_7
#define ADC_7_GPIO_PORT GPIOB
#define ADC_7_GPIO_PIN GPIO0
#endif

// AUXb4
#if USE_ADC_8
#define AD1_8_CHANNEL ADC_CHANNEL_IN9
#define ADC_8 AD1_8
#define ADC_8_GPIO_PORT GPIOB
#define ADC_8_GPIO_PIN GPIO1
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_9
#define USE_ADC_9 1
#endif
#if USE_ADC_9
#define AD1_9_CHANNEL ADC_CHANNEL_IN10
#define ADC_9 AD1_9
#define ADC_9_GPIO_PORT GPIOC
#define ADC_9_GPIO_PIN GPIO0
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_9
#endif

/*
 * R1 = 2.2k
 * R2 = 12k
 * adc * (3.3 / 2^12) * ((R1 + R2) / R1)
 */
#define VBAT_R1 2200.0f
#define VBAT_R2 12000.0f
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*((VBAT_R1+VBAT_R2)/VBAT_R1)*adc)

/*
 * PWM defines
 */

// SRVa connectors, activated in PWM mode by default

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO9
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_DRIVER PWMD1
#define PWM_SERVO_1_CHANNEL 0
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO GPIOE
#define PWM_SERVO_2_PIN GPIO11
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_DRIVER PWMD1
#define PWM_SERVO_2_CHANNEL 1
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOE
#define PWM_SERVO_3_PIN GPIO13
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_DRIVER PWMD1
#define PWM_SERVO_3_CHANNEL 2
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO GPIOE
#define PWM_SERVO_4_PIN GPIO14
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_DRIVER PWMD1
#define PWM_SERVO_4_CHANNEL 3
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
#endif

// SRVb connector, PWM mode disabled by default (DShot is enabled by default)

#ifndef USE_PWM5
#define USE_PWM5 0
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO6
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_DRIVER PWMD4
#define PWM_SERVO_5_CHANNEL 0
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM6
#define USE_PWM6 0
#endif
#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO7
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_DRIVER PWMD4
#define PWM_SERVO_6_CHANNEL 1
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM7
#define USE_PWM7 0
#endif
#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO8
#define PWM_SERVO_7_AF GPIO_AF2
#define PWM_SERVO_7_DRIVER PWMD4
#define PWM_SERVO_7_CHANNEL 2
#define PWM_SERVO_7_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_7_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM8
#define USE_PWM8 0
#endif
#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO9
#define PWM_SERVO_8_AF GPIO_AF2
#define PWM_SERVO_8_DRIVER PWMD4
#define PWM_SERVO_8_CHANNEL 3
#define PWM_SERVO_8_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_8_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM9
#define USE_PWM9 0
#endif
#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_GPIO GPIOA
#define PWM_SERVO_9_PIN GPIO0
#define PWM_SERVO_9_AF GPIO_AF2
#define PWM_SERVO_9_DRIVER PWMD5
#define PWM_SERVO_9_CHANNEL 0
#define PWM_SERVO_9_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_9_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM10
#define USE_PWM10 0
#endif
#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_GPIO GPIOA
#define PWM_SERVO_10_PIN GPIO1
#define PWM_SERVO_10_AF GPIO_AF2
#define PWM_SERVO_10_DRIVER PWMD5
#define PWM_SERVO_10_CHANNEL 1
#define PWM_SERVO_10_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_10_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM11
#define USE_PWM11 0
#endif
#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_GPIO GPIOA
#define PWM_SERVO_11_PIN GPIO2
#define PWM_SERVO_11_AF GPIO_AF2
#define PWM_SERVO_11_DRIVER PWMD5
#define PWM_SERVO_11_CHANNEL 2
#define PWM_SERVO_11_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_11_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM12
#define USE_PWM12 0
#endif
#if USE_PWM12
#define PWM_SERVO_12 12
#define PWM_SERVO_12_GPIO GPIOA
#define PWM_SERVO_12_PIN GPIO6
#define PWM_SERVO_12_AF GPIO_AF2
#define PWM_SERVO_12_DRIVER PWMD3
#define PWM_SERVO_12_CHANNEL 0
#define PWM_SERVO_12_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_12_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM13
#define USE_PWM13 0
#endif
#if USE_PWM13
#define PWM_SERVO_13 13
#define PWM_SERVO_13_GPIO GPIOA
#define PWM_SERVO_13_PIN GPIO3
#define PWM_SERVO_13_AF GPIO_AF2
#define PWM_SERVO_13_DRIVER PWMD5
#define PWM_SERVO_13_CHANNEL 3
#define PWM_SERVO_13_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_13_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM14
#define USE_PWM14 0
#endif
#if USE_PWM14
#define PWM_SERVO_14 14
#define PWM_SERVO_14_GPIO GPIOA
#define PWM_SERVO_14_PIN GPIO7
#define PWM_SERVO_14_AF GPIO_AF2
#define PWM_SERVO_14_DRIVER PWMD3
#define PWM_SERVO_14_CHANNEL 1
#define PWM_SERVO_14_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_14_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM15
#define USE_PWM15 0
#endif
#if USE_PWM15
#define PWM_SERVO_15 15
#define PWM_SERVO_15_GPIO GPIOB
#define PWM_SERVO_15_PIN GPIO0
#define PWM_SERVO_15_AF GPIO_AF2
#define PWM_SERVO_15_DRIVER PWMD3
#define PWM_SERVO_15_CHANNEL 2
#define PWM_SERVO_15_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_15_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM16
#define USE_PWM16 0
#endif
#if USE_PWM16
#define PWM_SERVO_16 16
#define PWM_SERVO_16_GPIO GPIOB
#define PWM_SERVO_16_PIN GPIO1
#define PWM_SERVO_16_AF GPIO_AF2
#define PWM_SERVO_16_DRIVER PWMD3
#define PWM_SERVO_16_CHANNEL 3
#define PWM_SERVO_16_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_16_ACTIVE PWM_OUTPUT_DISABLED
#endif

// servo index starting at 1 + regular servos + aux servos
// so NB = 1+8+8
#define ACTUATORS_PWM_NB 17


#ifdef STM32_PWM_USE_TIM1
#define PWM_CONF_TIM1 STM32_PWM_USE_TIM1
#else
#define PWM_CONF_TIM1 1
#endif
#define PWM_CONF1_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM1_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_4_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM3
#define PWM_CONF_TIM3 STM32_PWM_USE_TIM3
#else
#define PWM_CONF_TIM3 1
#endif
#define PWM_CONF3_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM3_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_12_ACTIVE, NULL }, \
    { PWM_SERVO_14_ACTIVE, NULL }, \
    { PWM_SERVO_15_ACTIVE, NULL }, \
    { PWM_SERVO_16_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM4
#define PWM_CONF_TIM4 STM32_PWM_USE_TIM4
#else
#define PWM_CONF_TIM4 1
#endif
#define PWM_CONF4_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM4_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_5_ACTIVE, NULL }, \
    { PWM_SERVO_6_ACTIVE, NULL }, \
    { PWM_SERVO_7_ACTIVE, NULL }, \
    { PWM_SERVO_8_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM5
#define PWM_CONF_TIM5 STM32_PWM_USE_TIM5
#else
#define PWM_CONF_TIM5 1
#endif
#define PWM_CONF5_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM5_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_9_ACTIVE, NULL }, \
    { PWM_SERVO_10_ACTIVE, NULL }, \
    { PWM_SERVO_11_ACTIVE, NULL }, \
    { PWM_SERVO_13_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * DSHOT
 */
#ifndef DSHOT_TELEMETRY_DEV
#define DSHOT_TELEMETRY_DEV NULL
#endif

#ifndef USE_DSHOT_TIM4
#define USE_DSHOT_TIM4 1 // use SRVb for DShot by default
#endif

#if USE_DSHOT_TIM4 // Servo B1, B2, B3, B4 on TIM4

// Servo B1, B2, B3, B4 on TM4 are primary DSHOT connector
#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO GPIOB
#define DSHOT_SERVO_1_PIN GPIO6
#define DSHOT_SERVO_1_AF GPIO_AF2
#define DSHOT_SERVO_1_DRIVER DSHOTD4
#define DSHOT_SERVO_1_CHANNEL 0

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO GPIOB
#define DSHOT_SERVO_2_PIN GPIO7
#define DSHOT_SERVO_2_AF GPIO_AF2
#define DSHOT_SERVO_2_DRIVER DSHOTD4
#define DSHOT_SERVO_2_CHANNEL 1

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO GPIOB
#define DSHOT_SERVO_3_PIN GPIO8
#define DSHOT_SERVO_3_AF GPIO_AF2
#define DSHOT_SERVO_3_DRIVER DSHOTD4
#define DSHOT_SERVO_3_CHANNEL 2

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO GPIOB
#define DSHOT_SERVO_4_PIN GPIO9
#define DSHOT_SERVO_4_AF GPIO_AF2
#define DSHOT_SERVO_4_DRIVER DSHOTD4
#define DSHOT_SERVO_4_CHANNEL 3

#define DSHOT_CONF_TIM4 1
#define DSHOT_CONF4_DEF { \
  .dma_stream = STM32_PWM4_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM4_UP_DMA_CHANNEL, \
  .pwmp = &PWMD4,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV             \
}

#endif

#if USE_DSHOT_TIM1 // Servo A1, A2, A3, A4 on TIM1 only activated if needed

#define DSHOT_SERVO_5 5
#define DSHOT_SERVO_5_GPIO GPIOE
#define DSHOT_SERVO_5_PIN GPIO9
#define DSHOT_SERVO_5_AF GPIO_AF1
#define DSHOT_SERVO_5_DRIVER DSHOTD1
#define DSHOT_SERVO_5_CHANNEL 0

#define DSHOT_SERVO_6 6
#define DSHOT_SERVO_6_GPIO GPIOE
#define DSHOT_SERVO_6_PIN GPIO11
#define DSHOT_SERVO_6_AF GPIO_AF1
#define DSHOT_SERVO_6_DRIVER DSHOTD1
#define DSHOT_SERVO_6_CHANNEL 1

#define DSHOT_SERVO_7 7
#define DSHOT_SERVO_7_GPIO GPIOE
#define DSHOT_SERVO_7_PIN GPIO13
#define DSHOT_SERVO_7_AF GPIO_AF1
#define DSHOT_SERVO_7_DRIVER DSHOTD1
#define DSHOT_SERVO_7_CHANNEL 2

#define DSHOT_SERVO_8 8
#define DSHOT_SERVO_8_GPIO GPIOE
#define DSHOT_SERVO_8_PIN GPIO14
#define DSHOT_SERVO_8_AF GPIO_AF1
#define DSHOT_SERVO_8_DRIVER DSHOTD1
#define DSHOT_SERVO_8_CHANNEL 3

#define DSHOT_CONF_TIM1 1
#define DSHOT_CONF1_DEF { \
  .dma_stream = STM32_PWM1_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM1_UP_DMA_CHANNEL, \
  .pwmp = &PWMD1,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV             \
}

#endif

/**
 * UART2 (Modem with optional flow control on AUXa disabled by default)
 */
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_AF 7
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

/**
 * UART7 (GPS) and UART3 (Companion)
 * are configured as UART from ChibiOS board file by default
 */

#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_AF 7

#define UART7_GPIO_PORT_TX GPIOA
#define UART7_GPIO_TX GPIO15
#define UART7_GPIO_PORT_RX GPIOB
#define UART7_GPIO_RX GPIO3
#define UART7_GPIO_AF 12

/**
 * UART4 on AUXa (not configured by default)
 */

#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_AF 8

/**
 * SBUS / Spektrum port
 *
 * Recommended config:
 *
 * primary SBUS port is UART8, a.k.a. RC1 on Tawaki board
 * secondary port (in dual driver) is UART6, a.k.a. RC2 on Tawaki board
 *
 * primary Spektrum port is UART6, a.k.a. RC2 on Tawaki board
 * secondary port is UART8, a.k.a. RC1 on Tawaki board
 */

// In case, do dynamic config of UARTs
#define USE_UART8_RX TRUE
#ifndef USE_UART8_TX // may be used in half duplex mode
#define USE_UART8_TX FALSE
#endif
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_AF 8

// FIXME when RC2 is used for FrSky telemetry
#define USE_UART6_RX TRUE
#define USE_UART6_TX FALSE
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO6
#define UART6_GPIO_AF 8

/* The line that is pulled low at power up to initiate the bind process
 * PB1: AUXb4
 */
#define SPEKTRUM_BIND_PIN GPIO1
#define SPEKTRUM_BIND_PIN_PORT GPIOB

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/**
 * PPM radio defines
 *
 * available on RC2
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD8

/*
 * PWM input
 */
// PWM_INPUT 1 on PA0 (AUXa1)
#define PWM_INPUT1_ICU            ICUD2
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO0
#define PWM_INPUT1_GPIO_AF        GPIO_AF1

// PWM_INPUT 2 on PA1 (AUXa2)
#define PWM_INPUT2_ICU            ICUD5
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_2
#define PWM_INPUT2_GPIO_PORT      GPIOA
#define PWM_INPUT2_GPIO_PIN       GPIO1
#define PWM_INPUT2_GPIO_AF        GPIO_AF2

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


// Internal I2C (baro, magneto)

#ifndef I2C4_CLOCK_SPEED
#define I2C4_CLOCK_SPEED 400000
#endif

#if I2C4_CLOCK_SPEED == 400000
#define I2C4_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#elif I2C4_CLOCK_SPEED == 100000
#define I2C4_CFG_DEF { \
  .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#else
#error "Unknown I2C4 clock speed"
#endif

// External I2C

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

// Internal SPI (IMU)
#define SPI4_GPIO_AF GPIO_AF5
#define SPI4_GPIO_PORT_MISO GPIOE
#define SPI4_GPIO_MISO GPIO5
#define SPI4_GPIO_PORT_MOSI GPIOE
#define SPI4_GPIO_MOSI GPIO6
#define SPI4_GPIO_PORT_SCK GPIOE
#define SPI4_GPIO_SCK GPIO2

// External SPI
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOD
#define SPI2_GPIO_SCK GPIO3

// SLAVE0 on SPI connector (NSS possible)
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO12
// SLAVE1 on AUXb1
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO3
// SLAVE2 on AUXb2
#define SPI_SELECT_SLAVE2_PORT GPIOA
#define SPI_SELECT_SLAVE2_PIN GPIO7
// SLAVE3 on AUXb3
#define SPI_SELECT_SLAVE3_PORT GPIOB
#define SPI_SELECT_SLAVE3_PIN GPIO0
// SLAVE4 on AUXb4
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN GPIO1
// SLAVE5 on PE4 (internal IMU)
#define SPI_SELECT_SLAVE5_PORT GPIOE
#define SPI_SELECT_SLAVE5_PIN GPIO4

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 0
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
#define SDLOG_BAT_CHAN ADC_CHANNEL_IN10
// usb led status
#define SDLOG_USB_LED 4
#define SDLOG_USB_VBUS_PORT GPIOA
#define SDLOG_USB_VBUS_PIN GPIO9


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_TAWAKI_1_00_H */

