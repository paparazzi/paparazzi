#ifndef CONFIG_AIOF7_H
#define CONFIG_AIOF7_H

#define BOARD_AIOF7

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
/* color, on PC15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO PAL_PORT(LINE_LED)
#define LED_1_GPIO_PIN PAL_PAD(LINE_LED)
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear


/*
 * ADCs
 */

// Internal ADC for battery enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT PAL_PORT(LINE_VBAT_MEAS)
#define ADC_1_GPIO_PIN PAL_PAD(LINE_VBAT_MEAS)
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif


#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*10.91*adc)


/*
 * PWM defines
 */

/*
 * enable TIM5, TIM3, TIM4 by default
 */
#ifndef USE_PWM_TIM3
#define USE_PWM_TIM3 1
#endif

#ifndef USE_PWM_TIM4
#define USE_PWM_TIM4 1
#endif

#ifndef USE_PWM_TIM5
#define USE_PWM_TIM5 1
#endif

// motors,  PWM mode disabled by default (DShot is enabled by default)

#ifndef USE_PWM1
#define USE_PWM1 0
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(LINE_MOTOR_1)
#define PWM_SERVO_1_PIN PAL_PAD(LINE_MOTOR_1)
#define PWM_SERVO_1_AF AF_MOTOR_1
#define PWM_SERVO_1_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_1_TIM)
#define PWM_SERVO_1_CHANNEL (MOTOR_1_TIM_CH-1)
#define PWM_SERVO_1_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_1_TIM)
#endif

#ifndef USE_PWM2
#define USE_PWM2 0
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(LINE_MOTOR_2)
#define PWM_SERVO_2_PIN PAL_PAD(LINE_MOTOR_2)
#define PWM_SERVO_2_AF AF_MOTOR_2
#define PWM_SERVO_2_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_2_TIM)
#define PWM_SERVO_2_CHANNEL (MOTOR_2_TIM_CH-1)
#define PWM_SERVO_2_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_2_TIM)
#endif

#ifndef USE_PWM3
#define USE_PWM3 0
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(LINE_MOTOR_3)
#define PWM_SERVO_3_PIN PAL_PAD(LINE_MOTOR_3)
#define PWM_SERVO_3_AF AF_MOTOR_3
#define PWM_SERVO_3_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_3_TIM)
#define PWM_SERVO_3_CHANNEL (MOTOR_3_TIM_CH-1)
#define PWM_SERVO_3_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_3_TIM)
#endif

#ifndef USE_PWM4
#define USE_PWM4 0
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(LINE_MOTOR_4)
#define PWM_SERVO_4_PIN PAL_PAD(LINE_MOTOR_4)
#define PWM_SERVO_4_AF AF_MOTOR_4
#define PWM_SERVO_4_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_4_TIM)
#define PWM_SERVO_4_CHANNEL (MOTOR_4_TIM_CH-1)
#define PWM_SERVO_4_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_4_TIM)
#endif


#ifndef USE_PWM5
#define USE_PWM5 0
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO PAL_PORT(LINE_MOTOR_5)
#define PWM_SERVO_5_PIN PAL_PAD(LINE_MOTOR_5)
#define PWM_SERVO_5_AF AF_MOTOR_5
#define PWM_SERVO_5_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_5_TIM)
#define PWM_SERVO_5_CHANNEL (MOTOR_5_TIM_CH-1)
#define PWM_SERVO_5_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_5_TIM)
#endif

#ifndef USE_PWM6
#define USE_PWM6 0
#endif
#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO PAL_PORT(LINE_MOTOR_6)
#define PWM_SERVO_6_PIN PAL_PAD(LINE_MOTOR_6)
#define PWM_SERVO_6_AF AF_MOTOR_6
#define PWM_SERVO_6_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_6_TIM)
#define PWM_SERVO_6_CHANNEL (MOTOR_6_TIM_CH-1)
#define PWM_SERVO_6_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_6_TIM)
#endif



// servo index starting at 1 + regular servos + aux servos
// so NB = 1+6
#define ACTUATORS_PWM_NB 7


/**
 * DSHOT
 */


#define DSHOT_TIM3_TELEMETRY_DEV NULL
#define DSHOT_TIM4_TELEMETRY_DEV NULL
#define DSHOT_TIM5_TELEMETRY_DEV NULL



#ifndef USE_DSHOT_TIM3
#define USE_DSHOT_TIM3 1 // MOTOR_1 MOTOR_2
#endif

#ifndef USE_DSHOT_TIM4
#define USE_DSHOT_TIM4 1 // MOTOR_5 MOTOR_6
#endif

#ifndef USE_DSHOT_TIM5
#define USE_DSHOT_TIM5 1 // MOTOR_3 MOTOR_4
#endif

#if USE_DSHOT_TIM3 // MOTOR_1 MOTOR_2 on TIM3

#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO PAL_PORT(LINE_MOTOR_1)
#define DSHOT_SERVO_1_PIN PAL_PAD(LINE_MOTOR_1)
#define DSHOT_SERVO_1_AF AF_MOTOR_1
#define DSHOT_SERVO_1_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_1_TIM)
#define DSHOT_SERVO_1_CHANNEL MOTOR_1_TIM_CH

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO PAL_PORT(LINE_MOTOR_2)
#define DSHOT_SERVO_2_PIN PAL_PAD(LINE_MOTOR_2)
#define DSHOT_SERVO_2_AF AF_MOTOR_2
#define DSHOT_SERVO_2_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_2_TIM)
#define DSHOT_SERVO_2_CHANNEL MOTOR_2_TIM_CH


#define DSHOT_CONF_TIM3 1
#define DSHOT_CONF3_DEF { \
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL, \
  .pwmp = &PWMD3,                           \
  .tlm_sd = DSHOT_TIM3_TELEMETRY_DEV,       \
  .dma_buf = &dshot3DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif


#if USE_DSHOT_TIM4 // MOTOR_5 MOTOR_6 on TIM4

#define DSHOT_SERVO_5 5
#define DSHOT_SERVO_5_GPIO PAL_PORT(LINE_MOTOR_5)
#define DSHOT_SERVO_5_PIN PAL_PAD(LINE_MOTOR_5)
#define DSHOT_SERVO_5_AF AF_MOTOR_5
#define DSHOT_SERVO_5_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_5_TIM)
#define DSHOT_SERVO_5_CHANNEL MOTOR_5_TIM_CH

#define DSHOT_SERVO_6 6
#define DSHOT_SERVO_6_GPIO PAL_PORT(LINE_MOTOR_6)
#define DSHOT_SERVO_6_PIN PAL_PAD(LINE_MOTOR_6)
#define DSHOT_SERVO_6_AF AF_MOTOR_6
#define DSHOT_SERVO_6_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_6_TIM)
#define DSHOT_SERVO_6_CHANNEL MOTOR_6_TIM_CH


#define DSHOT_CONF_TIM4 1
#define DSHOT_CONF4_DEF { \
  .dma_stream = STM32_PWM4_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM4_UP_DMA_CHANNEL, \
  .pwmp = &PWMD4,                           \
  .tlm_sd = DSHOT_TIM4_TELEMETRY_DEV,       \
  .dma_buf = &dshot4DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif

#if USE_DSHOT_TIM5 // MOTOR_3 MOTOR_4 on TIM5

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO PAL_PORT(LINE_MOTOR_3)
#define DSHOT_SERVO_3_PIN PAL_PAD(LINE_MOTOR_3)
#define DSHOT_SERVO_3_AF AF_MOTOR_3
#define DSHOT_SERVO_3_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_3_TIM)
#define DSHOT_SERVO_3_CHANNEL MOTOR_3_TIM_CH

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO PAL_PORT(LINE_MOTOR_4)
#define DSHOT_SERVO_4_PIN PAL_PAD(LINE_MOTOR_4)
#define DSHOT_SERVO_4_AF AF_MOTOR_4
#define DSHOT_SERVO_4_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_4_TIM)
#define DSHOT_SERVO_4_CHANNEL MOTOR_4_TIM_CH


#define DSHOT_CONF_TIM5 1
#define DSHOT_CONF5_DEF { \
  .dma_stream = STM32_PWM5_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM5_UP_DMA_CHANNEL, \
  .pwmp = &PWMD5,                           \
  .tlm_sd = DSHOT_TIM5_TELEMETRY_DEV,       \
  .dma_buf = &dshot5DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif



/**
 * UART1, UART2, UART3, UART4, UART5, UART6
 * 
 */

#define UART1_GPIO_PORT_TX  PAL_PORT(LINE_UART1_TX)
#define UART1_GPIO_TX       PAL_PAD(LINE_UART1_TX)
#define UART1_GPIO_PORT_RX  PAL_PORT(LINE_UART1_RX)
#define UART1_GPIO_RX       PAL_PAD(LINE_UART1_RX)
#define UART1_GPIO_AF       AF_UART1_TX

#define UART2_GPIO_PORT_TX  PAL_PORT(LINE_UART2_TX)
#define UART2_GPIO_TX       PAL_PAD(LINE_UART2_TX)
#define UART2_GPIO_PORT_RX  PAL_PORT(LINE_UART2_RX)
#define UART2_GPIO_RX       PAL_PAD(LINE_UART2_RX)
#define UART2_GPIO_AF       AF_UART2_TX

// #define UART3_GPIO_PORT_TX  PAL_PORT(LINE_UART3_TX)
// #define UART3_GPIO_TX       PAL_PAD(LINE_UART3_TX)
// #define UART3_GPIO_PORT_RX  PAL_PORT(LINE_UART3_RX)
// #define UART3_GPIO_RX       PAL_PAD(LINE_UART3_RX)
// #define UART3_GPIO_AF       AF_UART3_TX

#define UART4_GPIO_PORT_TX  PAL_PORT(LINE_AUX_A1)
#define UART4_GPIO_TX       PAL_PAD(LINE_AUX_A1)
#define UART4_GPIO_PORT_RX  PAL_PORT(LINE_AUX_A2)
#define UART4_GPIO_RX       PAL_PAD(LINE_AUX_A2)
#define UART4_GPIO_AF       AUX_A1_UART_AF



/**
 * SBUS / Spektrum port UART3
 *
 */

// In case, do dynamic config of UARTs
#ifndef USE_UART3_RX
#define USE_UART3_RX TRUE
#endif
#ifndef USE_UART3_TX // may be used in half duplex mode
#define USE_UART3_TX FALSE
#endif
// Tx and Rx are configured on the same pin, only one of them should be used
#define UART3_GPIO_PORT_TX  PAL_PORT(LINE_RC1)
#define UART3_GPIO_TX       PAL_PAD(LINE_RC1)
#define UART3_GPIO_PORT_RX  PAL_PORT(LINE_RC1)
#define UART3_GPIO_RX       PAL_PAD(LINE_RC1)
#define UART3_GPIO_AF       RC1_USART_AF

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000




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

#ifndef USE_SPI1
#define USE_SPI1 TRUE
#endif


// SPI 1
#define SPI1_GPIO_AF          AF_SPI1_INTERNAL_CLK
#define SPI1_GPIO_PORT_MISO   PAL_PORT(LINE_SPI1_INTERNAL_MISO)
#define SPI1_GPIO_MISO        PAL_PAD( LINE_SPI1_INTERNAL_MISO)
#define SPI1_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI1_INTERNAL_MOSI)
#define SPI1_GPIO_MOSI        PAL_PAD( LINE_SPI1_INTERNAL_MOSI)
#define SPI1_GPIO_PORT_SCK    PAL_PORT(LINE_SPI1_INTERNAL_CLK)
#define SPI1_GPIO_SCK         PAL_PAD( LINE_SPI1_INTERNAL_CLK)

// SPI 2
#define SPI2_GPIO_AF          AF_SPI2_EXTERNAL_CLK
#define SPI2_GPIO_PORT_MISO   PAL_PORT(LINE_SPI2_EXTERNAL_MISO)
#define SPI2_GPIO_MISO        PAL_PAD(LINE_SPI2_EXTERNAL_MISO)
#define SPI2_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI2_EXTERNAL_MOSI)
#define SPI2_GPIO_MOSI        PAL_PAD(LINE_SPI2_EXTERNAL_MOSI)
#define SPI2_GPIO_PORT_SCK    PAL_PORT(LINE_SPI2_EXTERNAL_CLK)
#define SPI2_GPIO_SCK         PAL_PAD(LINE_SPI2_EXTERNAL_CLK)

// SPI 3
#define SPI3_GPIO_AF          AF_SPI3_INTERNAL_CLK
#define SPI3_GPIO_PORT_MISO   PAL_PORT(LINE_SPI3_INTERNAL_MISO)
#define SPI3_GPIO_MISO        PAL_PAD( LINE_SPI3_INTERNAL_MISO)
#define SPI3_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI3_INTERNAL_MOSI)
#define SPI3_GPIO_MOSI        PAL_PAD( LINE_SPI3_INTERNAL_MOSI)
#define SPI3_GPIO_PORT_SCK    PAL_PORT(LINE_SPI3_INTERNAL_CLK)
#define SPI3_GPIO_SCK         PAL_PAD( LINE_SPI3_INTERNAL_CLK)


// GYRO1 on PA04
#define SPI_SELECT_SLAVE0_PORT  PAL_PORT(LINE_GYRO_CS_1)
#define SPI_SELECT_SLAVE0_PIN   PAL_PAD(LINE_GYRO_CS_1)
// OSD_CS on PA15
#define SPI_SELECT_SLAVE1_PORT  PAL_PORT(LINE_OSD_CS)
#define SPI_SELECT_SLAVE1_PIN   PAL_PAD(LINE_OSD_CS)
// FLASH_CS on PB12
#define SPI_SELECT_SLAVE2_PORT  PAL_PORT(LINE_FLASH_CS)
#define SPI_SELECT_SLAVE2_PIN   PAL_PAD(LINE_FLASH_CS)
// GYRO_CS_2 on PC03
#define SPI_SELECT_SLAVE3_PORT  PAL_PORT(LINE_GYRO_CS_2)
#define SPI_SELECT_SLAVE3_PIN   PAL_PAD(LINE_GYRO_CS_2)
// BARO_CS on PC13
#define SPI_SELECT_SLAVE4_PORT  PAL_PORT(LINE_BARO_CS)
#define SPI_SELECT_SLAVE4_PIN   PAL_PAD(LINE_BARO_CS)


// bat monitoring for file closing
#define SDLOG_BAT_ADC CONCAT_BOARD_PARAM(ADCD, VBAT_MEAS_ADC)
#define SDLOG_BAT_CHAN CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)


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
#define WS2812D1_GPIO PAL_PORT(LINE_AUX)
#define WS2812D1_PIN  PAL_PAD(LINE_AUX)
#define WS2812D1_AF 2
#define WS2812D1_CFG_DEF { \
  .dma_stream = STM32_PWM5_UP_DMA_STREAM, \
  .dma_channel = STM32_PWM5_UP_DMA_CHANNEL, \
  .dma_priority = STM32_PWM5_UP_DMA_PRIORITY, \
  .pwm_channel = 0, \
  .pwmp = &PWMD5 \
}

#endif /* CONFIG_AIOF7_H */

