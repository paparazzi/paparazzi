#ifndef CONFIG_HOLYBRO_KAKUTE_F7_H
#define CONFIG_HOLYBRO_KAKUTE_F7_H

#define BOARD_HOLYBRO_KAKUTE_F7

/**
 * ChibiOS board file
 */
#include "board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

/*
 * LEDs
 */
/* blue, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO PAL_PORT(LINE_LED1)
#define LED_1_GPIO_PIN PAL_PAD(LINE_LED1)
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// RSSI
#if USE_ADC_1
#define AD1_1_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, RSSI_ADC_IN)
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT PAL_PORT(LINE_RSSI)
#define ADC_1_GPIO_PIN PAL_PAD(LINE_RSSI)
#endif

// VBAT enabled by default
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT PAL_PORT(LINE_VBAT_MEAS)
#define ADC_2_GPIO_PIN PAL_PAD(LINE_VBAT_MEAS)
#endif

// CURRENT
#if USE_ADC_3
#define AD1_3_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, CURRENT_MEAS_ADC_IN)
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT PAL_PORT(LINE_CURRENT_MEAS)
#define ADC_3_GPIO_PIN PAL_PAD(LINE_CURRENT_MEAS)
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_2
#endif

/*
 * R1 = 1k
 * R2 = 10k
 * adc * (3.3 / 2^12) * ((R1 + R2) / R1)
 */
#define VBAT_R1 1000.0f
#define VBAT_R2 10000.0f
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*((VBAT_R1+VBAT_R2)/VBAT_R1)*adc)

/*
 * current sensor: 132A, 3.3V 12bits ADC -> 40 A/V -> 40000 * 3.3/2^12 mA/ADC
 */
#define DefaultMilliAmpereOfAdc(adc) ((40000.f*3.3f/4096.f)*adc)

/*
 * PWM defines
 */

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(LINE_S1)
#define PWM_SERVO_1_PIN PAL_PAD(LINE_S1)
#define PWM_SERVO_1_AF AF_S1
#define PWM_SERVO_1_DRIVER CONCAT_BOARD_PARAM(PWMD, S1_TIM)
#define PWM_SERVO_1_CHANNEL (S1_TIM_CH-1)
#define PWM_SERVO_1_CONF CONCAT_BOARD_PARAM(pwmcfg, S1_TIM)
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(LINE_S2)
#define PWM_SERVO_2_PIN PAL_PAD(LINE_S2)
#define PWM_SERVO_2_AF AF_S2
#define PWM_SERVO_2_DRIVER CONCAT_BOARD_PARAM(PWMD, S2_TIM)
#define PWM_SERVO_2_CHANNEL (S2_TIM_CH-1)
#define PWM_SERVO_2_CONF CONCAT_BOARD_PARAM(pwmcfg, S2_TIM)
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(LINE_S3)
#define PWM_SERVO_3_PIN PAL_PAD(LINE_S3)
#define PWM_SERVO_3_AF AF_S3
#define PWM_SERVO_3_DRIVER CONCAT_BOARD_PARAM(PWMD, S3_TIM)
#define PWM_SERVO_3_CHANNEL (S3_TIM_CH-1)
#define PWM_SERVO_3_CONF CONCAT_BOARD_PARAM(pwmcfg, S3_TIM)
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(LINE_S4)
#define PWM_SERVO_4_PIN PAL_PAD(LINE_S4)
#define PWM_SERVO_4_AF AF_S4
#define PWM_SERVO_4_DRIVER CONCAT_BOARD_PARAM(PWMD, S4_TIM)
#define PWM_SERVO_4_CHANNEL (S4_TIM_CH-1)
#define PWM_SERVO_4_CONF CONCAT_BOARD_PARAM(pwmcfg, S4_TIM)
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO PAL_PORT(LINE_S5)
#define PWM_SERVO_5_PIN PAL_PAD(LINE_S5)
#define PWM_SERVO_5_AF AF_S5
#define PWM_SERVO_5_DRIVER CONCAT_BOARD_PARAM(PWMD, S5_TIM)
#define PWM_SERVO_5_CHANNEL (S5_TIM_CH-1)
#define PWM_SERVO_5_CONF CONCAT_BOARD_PARAM(pwmcfg, S5_TIM)
#endif

#ifndef USE_PWM6
#define USE_PWM6 1
#endif
#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO PAL_PORT(LINE_S6)
#define PWM_SERVO_6_PIN PAL_PAD(LINE_S6)
#define PWM_SERVO_6_AF AF_S6
#define PWM_SERVO_6_DRIVER CONCAT_BOARD_PARAM(PWMD, S6_TIM)
#define PWM_SERVO_6_CHANNEL (S6_TIM_CH-1)
#define PWM_SERVO_6_CONF CONCAT_BOARD_PARAM(pwmcfg, S6_TIM)
#endif

// servo index starting at 1 + regular servos
// so NB = 1+6
#define ACTUATORS_PWM_NB 7

/**
 * DSHOT
 */
#ifndef DSHOT_TELEMETRY_DEV
#define DSHOT_TELEMETRY_DEV NULL
#endif

#ifndef USE_DSHOT_TIM
#define USE_DSHOT_TIM 0
#endif

#if USE_DSHOT_TIM

#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO PAL_PORT(LINE_S1)
#define DSHOT_SERVO_1_PIN PAL_PAD(LINE_S1)
#define DSHOT_SERVO_1_AF AF_S1
#define DSHOT_SERVO_1_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S1_TIM)
#define DSHOT_SERVO_1_CHANNEL S1_TIM_CH

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO PAL_PORT(LINE_S2)
#define DSHOT_SERVO_2_PIN PAL_PAD(LINE_S2)
#define DSHOT_SERVO_2_AF AF_S2
#define DSHOT_SERVO_2_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S2_TIM)
#define DSHOT_SERVO_2_CHANNEL S2_TIM_CH

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO PAL_PORT(LINE_S3)
#define DSHOT_SERVO_3_PIN PAL_PAD(LINE_S3)
#define DSHOT_SERVO_3_AF AF_S3
#define DSHOT_SERVO_3_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S3_TIM)
#define DSHOT_SERVO_3_CHANNEL S3_TIM_CH

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO PAL_PORT(LINE_S4)
#define DSHOT_SERVO_4_PIN PAL_PAD(LINE_S4)
#define DSHOT_SERVO_4_AF AF_S4
#define DSHOT_SERVO_4_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S4_TIM)
#define DSHOT_SERVO_4_CHANNEL S4_TIM_CH

#define DSHOT_CONF_TIM1 1
#define DSHOT_CONF4_DEF { \
  .dma_stream = STM32_PWM1_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM1_UP_DMA_CHANNEL, \
  .pwmp = &PWMD1,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV,            \
  .dma_buf = &dshot4DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#define DSHOT_CONF_TIM3 1
#define DSHOT_CONF3_DEF { \
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL, \
  .pwmp = &PWMD3,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV,            \
  .dma_buf = &dshot4DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif

/**
 * UART1 (Modem)
 */
#define UART1_GPIO_PORT_TX  PAL_PORT(LINE_UART1_TX)
#define UART1_GPIO_TX       PAL_PAD(LINE_UART1_TX)
#define UART1_GPIO_PORT_RX  PAL_PORT(LINE_UART1_RX)
#define UART1_GPIO_RX       PAL_PAD(LINE_UART1_RX)
#define UART1_GPIO_AF       AF_UART1_TX

/**
 * UART2 (GPS)
 */

#define UART2_GPIO_PORT_TX  PAL_PORT(LINE_UART2_TX)
#define UART2_GPIO_TX       PAL_PAD(LINE_UART2_TX)
#define UART2_GPIO_PORT_RX  PAL_PORT(LINE_UART2_RX)
#define UART2_GPIO_RX       PAL_PAD(LINE_UART2_RX)
#define UART2_GPIO_AF       AF_UART2_TX

/**
 * UART3 (Companion)
 */
#define UART3_GPIO_PORT_TX  PAL_PORT(LINE_UART3_TX)
#define UART3_GPIO_TX       PAL_PAD(LINE_UART3_TX)
#define UART3_GPIO_PORT_RX  PAL_PORT(LINE_UART3_RX)
#define UART3_GPIO_RX       PAL_PAD(LINE_UART3_RX)
#define UART3_GPIO_AF       AF_UART3_TX

/**
 * UART4
 */
#define UART4_GPIO_PORT_TX  PAL_PORT(LINE_UART4_TX)
#define UART4_GPIO_TX       PAL_PAD(LINE_UART4_TX)
#define UART4_GPIO_PORT_RX  PAL_PORT(LINE_UART4_RX)
#define UART4_GPIO_RX       PAL_PAD(LINE_UART4_RX)
#define UART4_GPIO_AF       AF_UART4_TX

/**
 * SBUS / Spektrum port
 */

#define USE_UART6_RX TRUE
#define USE_UART6_TX FALSE
#define UART6_GPIO_PORT_RX  PAL_PORT(LINE_RC1)
#define UART6_GPIO_RX       PAL_PAD(LINE_RC1)
#define UART6_GPIO_AF       RC1_USART_AF

/* The line that is pulled low at power up to initiate the bind process
 */
#define SPEKTRUM_BIND_PIN       PAL_PORT(LINE_XXX)
#define SPEKTRUM_BIND_PIN_PORT  PAL_PAD(LINE_XXX)

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/**
 * PPM radio defines
 *
 * available on RC2
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL CONCAT_BOARD_PARAM(ICU_CHANNEL_, RC2_TIM_CH)
#define PPM_TIMER CONCAT_BOARD_PARAM(ICUD, RC2_TIM)

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

/**
 * SPI Config
 */

// Internal SPI (SDCARD)
#define SPI1_GPIO_AF          AF_SPI1_CLK
#define SPI1_GPIO_PORT_MISO   PAL_PORT(LINE_SPI1_MISO)
#define SPI1_GPIO_MISO        PAL_PAD(LINE_SPI1_MISO)
#define SPI1_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI1_MOSI)
#define SPI1_GPIO_MOSI        PAL_PAD(LINE_SPI1_MOSI)
#define SPI1_GPIO_PORT_SCK    PAL_PORT(LINE_SPI1_CLK)
#define SPI1_GPIO_SCK         PAL_PAD(LINE_SPI1_CLK)

// Internal SPI (OSD)
#define SPI2_GPIO_AF          AF_SPI2_CLK
#define SPI2_GPIO_PORT_MISO   PAL_PORT(LINE_SPI2_MISO)
#define SPI2_GPIO_MISO        PAL_PAD(LINE_SPI2_MISO)
#define SPI2_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI2_MOSI)
#define SPI2_GPIO_MOSI        PAL_PAD(LINE_SPI2_MOSI)
#define SPI2_GPIO_PORT_SCK    PAL_PORT(LINE_SPI2_CLK)
#define SPI2_GPIO_SCK         PAL_PAD(LINE_SPI2_CLK)

// Internal SPI (IMU)
#define SPI4_GPIO_AF          AF_SPI4_INTERNAL_CLK
#define SPI4_GPIO_PORT_MISO   PAL_PORT(LINE_SPI4_INTERNAL_MISO)
#define SPI4_GPIO_MISO        PAL_PAD(LINE_SPI4_INTERNAL_MISO)
#define SPI4_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI4_INTERNAL_MOSI)
#define SPI4_GPIO_MOSI        PAL_PAD(LINE_SPI4_INTERNAL_MOSI)
#define SPI4_GPIO_PORT_SCK    PAL_PORT(LINE_SPI4_INTERNAL_CLK)
#define SPI4_GPIO_SCK         PAL_PAD(LINE_SPI4_INTERNAL_CLK)

// SLAVE0 on IMU1 (MPU6000)
#define SPI_SELECT_SLAVE0_PORT  PAL_PORT(LINE_IMU_CS)
#define SPI_SELECT_SLAVE0_PIN   PAL_PAD(LINE_IMU_CS)
// SLAVE1 on SDCARD
#define SPI_SELECT_SLAVE1_PORT  PAL_PORT(LINE_SDCARD_CS)
#define SPI_SELECT_SLAVE1_PIN   PAL_PAD(LINE_SDCARD_CS)
// SLAVE2 on OSD
#define SPI_SELECT_SLAVE2_PORT  PAL_PORT(LINE_OSD_CS)
#define SPI_SELECT_SLAVE2_PIN   PAL_PAD(LINE_OSD_CS)

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 0
#endif

/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_HOLYBRO_KAKUTE_F7_H */

