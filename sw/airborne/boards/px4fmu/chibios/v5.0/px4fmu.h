#ifndef CONFIG_PX4FMU_5_00_H
#define CONFIG_PX4FMU_5_00_H

#define BOARD_PX4FMU

/**
 * ChibiOS board file
 */
#include "boards/px4fmu/chibios/v5.0/board.h"

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
#if defined(LINE_LED1)
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO        PAL_PORT(LINE_LED1)
#define LED_1_GPIO_PIN    PAL_PAD(LINE_LED1)
#define LED_1_GPIO_ON     gpio_clear
#define LED_1_GPIO_OFF    gpio_set
#define LED_1_AFIO_REMAP  ((void)0)
#endif

#if defined(LINE_LED2)
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO        PAL_PORT(LINE_LED2)
#define LED_2_GPIO_PIN    PAL_PAD(LINE_LED2)
#define LED_2_GPIO_ON     gpio_clear
#define LED_2_GPIO_OFF    gpio_set
#define LED_2_AFIO_REMAP  ((void)0)
#endif

#if defined(LINE_LED3)
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO        PAL_PORT(LINE_LED3)
#define LED_3_GPIO_PIN    PAL_PAD(LINE_LED3)
#define LED_3_GPIO_ON     gpio_clear
#define LED_3_GPIO_OFF    gpio_set
#define LED_3_AFIO_REMAP  ((void)0)
#endif

#if defined(LINE_LED4)
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO        PAL_PORT(LINE_LED4)
#define LED_4_GPIO_PIN    PAL_PAD(LINE_LED4)
#define LED_4_GPIO_ON     gpio_clear
#define LED_4_GPIO_OFF    gpio_set
#define LED_4_AFIO_REMAP  ((void)0)
#endif

/*
 * ADCs 
 */
#if defined(LINE_ADC1)
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL     ADC_CHANNEL_IN0
#define ADC_1             AD1_1
#define ADC_1_GPIO_PORT   PAL_PORT(LINE_ADC1)
#define ADC_1_GPIO_PIN    PAL_PAD(LINE_ADC1)
#endif
#endif

#if defined(LINE_ADC2)
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL     ADC_CHANNEL_IN1
#define ADC_2             AD1_2
#define ADC_2_GPIO_PORT   PAL_PORT(LINE_ADC2)
#define ADC_2_GPIO_PIN    PAL_PAD(LINE_ADC2)
#endif
#endif

#if defined(LINE_ADC3)
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL     ADC_CHANNEL_IN2
#define ADC_3             AD1_3
#define ADC_3_GPIO_PORT   PAL_PORT(LINE_ADC3)
#define ADC_3_GPIO_PIN    PAL_PAD(LINE_ADC3)
#endif
#endif

#if defined(LINE_ADC4)
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL     ADC_CHANNEL_IN3
#define ADC_4             AD1_4
#define ADC_4_GPIO_PORT   PAL_PORT(LINE_ADC4)
#define ADC_4_GPIO_PIN    PAL_PAD(LINE_ADC4)
#endif
#endif

#if defined(LINE_ADC5)
#if USE_ADC_5
#define AD1_5_CHANNEL     ADC_CHANNEL_IN4
#define ADC_5             AD1_5
#define ADC_5_GPIO_PORT   PAL_PORT(LINE_ADC5)
#define ADC_5_GPIO_PIN    PAL_PAD(LINE_ADC5)
#endif
#endif

#if defined(LINE_ADC6)
#if USE_ADC_6
#define AD1_6_CHANNEL     ADC_CHANNEL_IN14
#define ADC_6             AD1_6
#define ADC_6_GPIO_PORT   PAL_PORT(LINE_ADC6)
#define ADC_6_GPIO_PIN    PAL_PAD(LINE_ADC6)
#endif
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

/* allow to define ADC_CHANNEL_CURRENT in the airframe file*/
#if !defined(ADC_CHANNEL_CURRENT) && !ADC_CURRENT_DISABLE
#define ADC_CHANNEL_CURRENT ADC_2
#endif

/* Default powerbrick values */
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f) * 10.3208191126f * adc)
#define MilliAmpereOfAdc(adc) ((3.3f/4096.0f) * 24000.0f * adc)

/*
 * PWM defines (TODO DRIVER and CHANNEL)
 */
#if defined(LINE_SERVO1)
#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_GPIO    PAL_PORT(LINE_SERVO1)
#define PWM_SERVO_1_PIN     PAL_PAD(LINE_SERVO1)
#define PWM_SERVO_1_AF      AF_LINE_SERVO1
#define PWM_SERVO_1_DRIVER  PWMD1
#define PWM_SERVO_1_CHANNEL 4-1
#define PWM_SERVO_1_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO2)
#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_GPIO    PAL_PORT(LINE_SERVO2)
#define PWM_SERVO_2_PIN     PAL_PAD(LINE_SERVO2)
#define PWM_SERVO_2_AF      AF_LINE_SERVO2
#define PWM_SERVO_2_DRIVER  PWMD1
#define PWM_SERVO_2_CHANNEL 3-1
#define PWM_SERVO_2_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO3)
#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_GPIO    PAL_PORT(LINE_SERVO3)
#define PWM_SERVO_3_PIN     PAL_PAD(LINE_SERVO3)
#define PWM_SERVO_3_AF      AF_LINE_SERVO3
#define PWM_SERVO_3_DRIVER  PWMD1
#define PWM_SERVO_3_CHANNEL 2-1
#define PWM_SERVO_3_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO4)
#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_GPIO    PAL_PORT(LINE_SERVO4)
#define PWM_SERVO_4_PIN     PAL_PAD(LINE_SERVO4)
#define PWM_SERVO_4_AF      AF_LINE_SERVO4
#define PWM_SERVO_4_DRIVER  PWMD1
#define PWM_SERVO_4_CHANNEL 1-1
#define PWM_SERVO_4_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO5)
#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_GPIO    PAL_PORT(LINE_SERVO5)
#define PWM_SERVO_5_PIN     PAL_PAD(LINE_SERVO5)
#define PWM_SERVO_5_AF      AF_LINE_SERVO5
#define PWM_SERVO_5_DRIVER  PWMD4
#define PWM_SERVO_5_CHANNEL 2-1
#define PWM_SERVO_5_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO6)
#ifndef USE_PWM6
#define USE_PWM6 1
#endif
#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_GPIO    PAL_PORT(LINE_SERVO6)
#define PWM_SERVO_6_PIN     PAL_PAD(LINE_SERVO6)
#define PWM_SERVO_6_AF      AF_LINE_SERVO6
#define PWM_SERVO_6_DRIVER  PWMD4
#define PWM_SERVO_6_CHANNEL 3-1
#define PWM_SERVO_6_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_6_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO7)
#ifndef USE_PWM7
#define USE_PWM7 1
#endif
#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_GPIO    PAL_PORT(LINE_SERVO7)
#define PWM_SERVO_7_PIN     PAL_PAD(LINE_SERVO7)
#define PWM_SERVO_7_AF      AF_LINE_SERVO7
#define PWM_SERVO_7_DRIVER  PWMD12
#define PWM_SERVO_7_CHANNEL 1-1
#define PWM_SERVO_7_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_7_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif

#if defined(LINE_SERVO8)
#ifndef USE_PWM8
#define USE_PWM8 1
#endif
#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_GPIO    PAL_PORT(LINE_SERVO8)
#define PWM_SERVO_8_PIN     PAL_PAD(LINE_SERVO8)
#define PWM_SERVO_8_AF      AF_LINE_SERVO8
#define PWM_SERVO_8_DRIVER  PWMD12
#define PWM_SERVO_8_CHANNEL 2-1
#define PWM_SERVO_8_ACTIVE  PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_8_ACTIVE  PWM_OUTPUT_DISABLED
#endif
#endif


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
    { PWM_SERVO_4_ACTIVE, NULL }, \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
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
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_SERVO_5_ACTIVE, NULL }, \
    { PWM_SERVO_6_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM12
#define PWM_CONF_TIM12 STM32_PWM_USE_TIM12
#else
#define PWM_CONF_TIM12 1
#endif
#define PWM_CONF12_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM12_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_7_ACTIVE, NULL }, \
    { PWM_SERVO_8_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 2
#define PPM_TIMER_FREQUENCY 2000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD8

/**
 * UART defines
 */
#if defined(LINE_UART1_TX)
#define UART1_GPIO_PORT_TX    PAL_PORT(LINE_UART1_TX)
#define UART1_GPIO_TX         PAL_PAD(LINE_UART1_TX)
#endif
#if defined(LINE_UART1_RX)
#define UART1_GPIO_PORT_RX    PAL_PORT(LINE_UART1_RX)
#define UART1_GPIO_RX         PAL_PAD(LINE_UART1_RX)
#endif
#if defined(AF_LINE_UART1_TX)
#define UART1_GPIO_AF         AF_LINE_UART1_TX
#elif defined(AF_LINE_UART1_RX)
#define UART1_GPIO_AF         AF_LINE_UART1_RX
#else
#define UART1_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART2_TX)
#define UART2_GPIO_PORT_TX    PAL_PORT(LINE_UART2_TX)
#define UART2_GPIO_TX         PAL_PAD(LINE_UART2_TX)
#endif
#if defined(LINE_UART2_RX)
#define UART2_GPIO_PORT_RX    PAL_PORT(LINE_UART2_RX)
#define UART2_GPIO_RX         PAL_PAD(LINE_UART2_RX)
#endif
#if defined(AF_LINE_UART2_TX)
#define UART2_GPIO_AF         AF_LINE_UART2_TX
#elif defined(AF_LINE_UART2_RX)
#define UART2_GPIO_AF         AF_LINE_UART2_RX
#else
#define UART2_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART3_TX)
#define UART3_GPIO_PORT_TX    PAL_PORT(LINE_UART3_TX)
#define UART3_GPIO_TX         PAL_PAD(LINE_UART3_TX)
#endif
#if defined(LINE_UART3_RX)
#define UART3_GPIO_PORT_RX    PAL_PORT(LINE_UART3_RX)
#define UART3_GPIO_RX         PAL_PAD(LINE_UART3_RX)
#endif
#if defined(AF_LINE_UART3_TX)
#define UART3_GPIO_AF         AF_LINE_UART3_TX
#elif defined(AF_LINE_UART3_RX)
#define UART3_GPIO_AF         AF_LINE_UART3_RX
#else
#define UART3_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART4_TX)
#define UART4_GPIO_PORT_TX    PAL_PORT(LINE_UART4_TX)
#define UART4_GPIO_TX         PAL_PAD(LINE_UART4_TX)
#endif
#if defined(LINE_UART4_RX)
#define UART4_GPIO_PORT_RX    PAL_PORT(LINE_UART4_RX)
#define UART4_GPIO_RX         PAL_PAD(LINE_UART4_RX)
#endif
#if defined(AF_LINE_UART4_TX)
#define UART4_GPIO_AF         AF_LINE_UART4_TX
#elif defined(AF_LINE_UART4_RX)
#define UART4_GPIO_AF         AF_LINE_UART4_RX
#else
#define UART4_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART5_TX)
#define UART5_GPIO_PORT_TX    PAL_PORT(LINE_UART5_TX)
#define UART5_GPIO_TX         PAL_PAD(LINE_UART5_TX)
#endif
#if defined(LINE_UART5_RX)
#define UART5_GPIO_PORT_RX    PAL_PORT(LINE_UART5_RX)
#define UART5_GPIO_RX         PAL_PAD(LINE_UART5_RX)
#endif
#if defined(AF_LINE_UART5_TX)
#define UART5_GPIO_AF         AF_LINE_UART5_TX
#elif defined(AF_LINE_UART5_RX)
#define UART5_GPIO_AF         AF_LINE_UART5_RX
#else
#define UART5_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART6_TX)
#define UART6_GPIO_PORT_TX    PAL_PORT(LINE_UART6_TX)
#define UART6_GPIO_TX         PAL_PAD(LINE_UART6_TX)
#endif
#if defined(LINE_UART6_RX)
#define UART6_GPIO_PORT_RX    PAL_PORT(LINE_UART6_RX)
#define UART6_GPIO_RX         PAL_PAD(LINE_UART6_RX)
#endif
#if defined(AF_LINE_UART6_TX)
#define UART6_GPIO_AF         AF_LINE_UART6_TX
#elif defined(AF_LINE_UART6_RX)
#define UART6_GPIO_AF         AF_LINE_UART6_RX
#else
#define UART6_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART7_TX)
#define UART7_GPIO_PORT_TX    PAL_PORT(LINE_UART7_TX)
#define UART7_GPIO_TX         PAL_PAD(LINE_UART7_TX)
#endif
#if defined(LINE_UART7_RX)
#define UART7_GPIO_PORT_RX    PAL_PORT(LINE_UART7_RX)
#define UART7_GPIO_RX         PAL_PAD(LINE_UART7_RX)
#endif
#if defined(AF_LINE_UART7_TX)
#define UART7_GPIO_AF         AF_LINE_UART7_TX
#elif defined(AF_LINE_UART7_RX)
#define UART7_GPIO_AF         AF_LINE_UART7_RX
#else
#define UART7_GPIO_AF         ((void)0)
#endif

#if defined(LINE_UART8_TX)
#define UART8_GPIO_PORT_TX    PAL_PORT(LINE_UART8_TX)
#define UART8_GPIO_TX         PAL_PAD(LINE_UART8_TX)
#endif
#if defined(LINE_UART8_RX)
#define UART8_GPIO_PORT_RX    PAL_PORT(LINE_UART8_RX)
#define UART8_GPIO_RX         PAL_PAD(LINE_UART8_RX)
#endif
#if defined(AF_LINE_UART8_TX)
#define UART8_GPIO_AF         AF_LINE_UART8_TX
#elif defined(AF_LINE_UART8_RX)
#define UART8_GPIO_AF         AF_LINE_UART8_RX
#else
#define UART8_GPIO_AF         ((void)0)
#endif

/**
 * I2C defines
 */
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

#ifndef I2C3_CLOCK_SPEED
#define I2C3_CLOCK_SPEED 400000
#endif

#if I2C3_CLOCK_SPEED == 400000
#define I2C3_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#elif I2C3_CLOCK_SPEED == 100000
#define I2C3_CFG_DEF { \
  .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}
#else
#error "Unknown I2C3 clock speed"
#endif

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

/**
 * SPI Config
 */
#if defined(LINE_SPI1_MISO) && defined(LINE_SPI1_MOSI) && defined(LINE_SPI1_SCK)
#define SPI1_GPIO_PORT_MISO   PAL_PORT(LINE_SPI1_MISO)
#define SPI1_GPIO_MISO        PAL_PAD(LINE_SPI1_MISO)
#define SPI1_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI1_MOSI)
#define SPI1_GPIO_MOSI        PAL_PAD(LINE_SPI1_MOSI)
#define SPI1_GPIO_PORT_SCK    PAL_PORT(LINE_SPI1_SCK)
#define SPI1_GPIO_SCK         PAL_PAD(LINE_SPI1_SCK)

#if defined(AF_LINE_SPI1_SCK)
#define SPI1_GPIO_AF    AF_LINE_SPI1_SCK
#endif
#endif

#if defined(LINE_SPI2_MISO) && defined(LINE_SPI2_MOSI) && defined(LINE_SPI2_SCK)
#define SPI2_GPIO_PORT_MISO   PAL_PORT(LINE_SPI2_MISO)
#define SPI2_GPIO_MISO        PAL_PAD(LINE_SPI2_MISO)
#define SPI2_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI2_MOSI)
#define SPI2_GPIO_MOSI        PAL_PAD(LINE_SPI2_MOSI)
#define SPI2_GPIO_PORT_SCK    PAL_PORT(LINE_SPI2_SCK)
#define SPI2_GPIO_SCK         PAL_PAD(LINE_SPI2_SCK)

#if defined(AF_LINE_SPI2_SCK)
#define SPI2_GPIO_AF    AF_LINE_SPI2_SCK
#endif
#endif

#if defined(LINE_SPI3_MISO) && defined(LINE_SPI3_MOSI) && defined(LINE_SPI3_SCK)
#define SPI3_GPIO_PORT_MISO   PAL_PORT(LINE_SPI3_MISO)
#define SPI3_GPIO_MISO        PAL_PAD(LINE_SPI3_MISO)
#define SPI3_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI3_MOSI)
#define SPI3_GPIO_MOSI        PAL_PAD(LINE_SPI3_MOSI)
#define SPI3_GPIO_PORT_SCK    PAL_PORT(LINE_SPI3_SCK)
#define SPI3_GPIO_SCK         PAL_PAD(LINE_SPI3_SCK)

#if defined(AF_LINE_SPI3_SCK)
#define SPI3_GPIO_AF    AF_LINE_SPI3_SCK
#endif
#endif

#if defined(LINE_SPI4_MISO) && defined(LINE_SPI4_MOSI) && defined(LINE_SPI4_SCK)
#define SPI4_GPIO_PORT_MISO   PAL_PORT(LINE_SPI4_MISO)
#define SPI4_GPIO_MISO        PAL_PAD(LINE_SPI4_MISO)
#define SPI4_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI4_MOSI)
#define SPI4_GPIO_MOSI        PAL_PAD(LINE_SPI4_MOSI)
#define SPI4_GPIO_PORT_SCK    PAL_PORT(LINE_SPI4_SCK)
#define SPI4_GPIO_SCK         PAL_PAD(LINE_SPI4_SCK)

#if defined(AF_LINE_SPI4_SCK)
#define SPI4_GPIO_AF    AF_LINE_SPI4_SCK
#endif
#endif

#if defined(LINE_SPI5_MISO) && defined(LINE_SPI5_MOSI) && defined(LINE_SPI5_SCK)
#define SPI5_GPIO_PORT_MISO   PAL_PORT(LINE_SPI5_MISO)
#define SPI5_GPIO_MISO        PAL_PAD(LINE_SPI5_MISO)
#define SPI5_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI5_MOSI)
#define SPI5_GPIO_MOSI        PAL_PAD(LINE_SPI5_MOSI)
#define SPI5_GPIO_PORT_SCK    PAL_PORT(LINE_SPI5_SCK)
#define SPI5_GPIO_SCK         PAL_PAD(LINE_SPI5_SCK)

#if defined(AF_LINE_SPI5_SCK)
#define SPI5_GPIO_AF    AF_LINE_SPI5_SCK
#endif
#endif

#if defined(LINE_SPI6_MISO) && defined(LINE_SPI6_MOSI) && defined(LINE_SPI6_SCK)
#define SPI6_GPIO_PORT_MISO   PAL_PORT(LINE_SPI6_MISO)
#define SPI6_GPIO_MISO        PAL_PAD(LINE_SPI6_MISO)
#define SPI6_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI6_MOSI)
#define SPI6_GPIO_MOSI        PAL_PAD(LINE_SPI6_MOSI)
#define SPI6_GPIO_PORT_SCK    PAL_PORT(LINE_SPI6_SCK)
#define SPI6_GPIO_SCK         PAL_PAD(LINE_SPI6_SCK)

#if defined(AF_LINE_SPI6_SCK)
#define SPI6_GPIO_AF    AF_LINE_SPI6_SCK
#endif
#endif

/**
 * SPI Slaves
 */
#if defined(LINE_SPI_SLAVE0)
#define SPI_SELECT_SLAVE0_PORT  PAL_PORT(LINE_SPI_SLAVE0)
#define SPI_SELECT_SLAVE0_PIN   PAL_PAD(LINE_SPI_SLAVE0)
#endif

#if defined(LINE_SPI_SLAVE1)
#define SPI_SELECT_SLAVE1_PORT  PAL_PORT(LINE_SPI_SLAVE1)
#define SPI_SELECT_SLAVE1_PIN   PAL_PAD(LINE_SPI_SLAVE1)
#endif

#if defined(LINE_SPI_SLAVE2)
#define SPI_SELECT_SLAVE2_PORT  PAL_PORT(LINE_SPI_SLAVE2)
#define SPI_SELECT_SLAVE2_PIN   PAL_PAD(LINE_SPI_SLAVE2)
#endif

#if defined(LINE_SPI_SLAVE3)
#define SPI_SELECT_SLAVE3_PORT  PAL_PORT(LINE_SPI_SLAVE3)
#define SPI_SELECT_SLAVE3_PIN   PAL_PAD(LINE_SPI_SLAVE3)
#endif

#if defined(LINE_SPI_SLAVE4)
#define SPI_SELECT_SLAVE4_PORT  PAL_PORT(LINE_SPI_SLAVE4)
#define SPI_SELECT_SLAVE4_PIN   PAL_PAD(LINE_SPI_SLAVE4)
#endif

#if defined(LINE_SPI_SLAVE5)
#define SPI_SELECT_SLAVE5_PORT  PAL_PORT(LINE_SPI_SLAVE5)
#define SPI_SELECT_SLAVE5_PIN   PAL_PAD(LINE_SPI_SLAVE5)
#endif

#if defined(LINE_SPI_SLAVE6)
#define SPI_SELECT_SLAVE6_PORT  PAL_PORT(LINE_SPI_SLAVE6)
#define SPI_SELECT_SLAVE6_PIN   PAL_PAD(LINE_SPI_SLAVE6)
#endif

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
#if defined(LINE_SDIO_D0) && defined(LINE_SDIO_D1) && defined(LINE_SDIO_D2) && defined(LINE_SDIO_D3) && defined(LINE_SDIO_CK) && defined(LINE_SDIO_CMD)
#define SDIO_D0_PORT    PAL_PORT(LINE_SDIO_D0)
#define SDIO_D0_PIN     PAL_PAD(LINE_SDIO_D0)
#define SDIO_D1_PORT    PAL_PORT(LINE_SDIO_D1)
#define SDIO_D1_PIN     PAL_PAD(LINE_SDIO_D1)
#define SDIO_D2_PORT    PAL_PORT(LINE_SDIO_D2)
#define SDIO_D2_PIN     PAL_PAD(LINE_SDIO_D2)
#define SDIO_D3_PORT    PAL_PORT(LINE_SDIO_D3)
#define SDIO_D3_PIN     PAL_PAD(LINE_SDIO_D3)
#define SDIO_CK_PORT    PAL_PORT(LINE_SDIO_CK)
#define SDIO_CK_PIN     PAL_PAD(LINE_SDIO_CK)
#define SDIO_CMD_PORT   PAL_PORT(LINE_SDIO_CMD)
#define SDIO_CMD_PIN    PAL_PAD(LINE_SDIO_CMD)

#if defined(AF_LINE_SDIO_CMD)
#define SDIO_AF   AF_LINE_SDIO_CMD
#else
#define SDIO_AF   ((void)0)
#endif
#endif

#if defined(LINE_USB_VBUS)
#define SDLOG_USB_VBUS_PORT   PAL_PORT(LINE_USB_VBUS)
#define SDLOG_USB_VBUS_PIN    PAL_PAD(LINE_USB_VBUS)
#endif

// bat monitoring for file closing
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_1_CHANNEL
// usb led status
#define SDLOG_USB_LED 3


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_PX4FMU_4_00_H */

