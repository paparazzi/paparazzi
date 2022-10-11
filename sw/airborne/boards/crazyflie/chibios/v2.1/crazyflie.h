#ifndef CONFIG_CRAZYFLIE_2_1_H
#define CONFIG_CRAZYFLIE_2_1_H

#define BOARD_CRAZYFLIE

/**
 * ChibiOS board file
 */
#include "boards/crazyflie/chibios/v2.1/board.h"

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
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO PAL_PORT(LED_RED_R)
#define LED_1_GPIO_PIN PAL_PAD(LED_RED_R)
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO PAL_PORT(LED_RED_L)
#define LED_2_GPIO_PIN PAL_PAD(LED_RED_L)
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO PAL_PORT(LED_GREEN_R)
#define LED_3_GPIO_PIN PAL_PAD(LED_GREEN_R)
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO PAL_PORT(LED_GREEN_L)
#define LED_4_GPIO_PIN PAL_PAD(LED_GREEN_L)
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO PAL_PORT(LED_BLUE_L)
#define LED_5_GPIO_PIN PAL_PAD(PAL_PAD(LED_BLUE_L))
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// TODO for AUX
// No VBAT monitoring ?

/*
 * PWM defines
 */

// SRVa connectors, activated in PWM mode by default

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(MOTOR1)
#define PWM_SERVO_1_PIN PAL_PAD(MOTOR1)
#define PWM_SERVO_1_AF AF_MOTOR1
#define PWM_SERVO_1_DRIVER PWMD2
#define PWM_SERVO_1_CHANNEL 1
#define PWM_SERVO_1_CONF pwmcfg2
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(MOTOR2)
#define PWM_SERVO_2_PIN PAL_PAD(MOTOR2)
#define PWM_SERVO_2_AF AF_MOTOR2
#define PWM_SERVO_2_DRIVER PWMD2
#define PWM_SERVO_2_CHANNEL 3
#define PWM_SERVO_2_CONF pwmcfg2
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(MOTOR3)
#define PWM_SERVO_3_PIN PAL_PAD(MOTOR3)
#define PWM_SERVO_3_AF AF_MOTOR3
#define PWM_SERVO_3_DRIVER PWMD2
#define PWM_SERVO_3_CHANNEL 0
#define PWM_SERVO_3_CONF pwmcfg2
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(MOTOR4)
#define PWM_SERVO_4_PIN PAL_PAD(MOTOR4)
#define PWM_SERVO_4_AF AF_MOTOR4
#define PWM_SERVO_4_DRIVER PWMD4
#define PWM_SERVO_4_CHANNEL 3
#define PWM_SERVO_4_CONF pwmcfg4
#endif

// servo index starting at 1 + regular servos + aux servos
// so NB = 1+4
#define ACTUATORS_PWM_NB 5


// PWM control of brushed motors
// Freq = 84 MHz (corresponding to prescaler of 0)
// Period = 256 (corresponding to 8bit resolution for command at ~328 kHz))
// as indicated in Crazyflie source code, 328 kHz offers better natural filtering
// than 128 kHz
// It is also needed to redefined PWM_CMD_TO_US to get the proper converstion
// from command to clock pulses number
#define PWM_CMD_TO_US(_t) (_t)
#define PWM_FREQUENCY 84000000
#define SERVO_HZ (PWM_FREQUENCY / 256) // 328125

/**
 * UART2 E_TX2
 */
#define UART2_GPIO_PORT_TX PAL_PORT(E_TX2)
#define UART2_GPIO_TX PAL_PAD(E_TX2)
#define UART2_GPIO_PORT_RX PAL_PORT(E_RX2)
#define UART2_GPIO_RX PAL_PAD(E_RX2)
#define UART2_GPIO_AF AF_E_RX2
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

/**
 * UART3 E_TX1
 */
#define UART3_GPIO_PORT_TX PAL_PORT(E_TX1)
#define UART3_GPIO_TX PAL_PAD(E_TX1)
#define UART3_GPIO_PORT_RX PAL_PORT(E_RX1)
#define UART3_GPIO_RX PAL_PAD(E_RX1)
#define UART3_GPIO_AF AF_E_RX1

/**
 * UART6 NRF
 */
#define UART6_GPIO_PORT_TX PAL_PORT(NRF_TX)
#define UART6_GPIO_TX PAL_PAD(NRF_TX)
#define UART6_GPIO_PORT_RX PAL_PORT(NRF_RX)
#define UART6_GPIO_RX PAL_PAD(NRF_RX)
#define UART6_GPIO_AF AF_NRF_RX
#define UART6_GPIO_PORT_CTS PAL_PORT(NRF_FLOW_CTRL)
#define UART6_GPIO_CTS NRF_FLOW_CTRL

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


// Internal I2C (IMU, baro)

#ifndef I2C3_CLOCK_SPEED
#define I2C3_CLOCK_SPEED 400000
#endif
#if I2C3_CLOCK_SPEED == 400000
#define I2C3_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C3_CLOCK_SPEED == 100000
#define I2C3_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error "Invalid I2C3 clock speed"
#endif
#define I2C3_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C3_CLOCK_SPEED,  \
           I2C3_DUTY_CYCLE,   \
           }

// External I2C

#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif
#if I2C1_CLOCK_SPEED == 400000
#define I2C1_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error "Invalid I2C1 clock speed"
#endif
#define I2C1_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           I2C1_DUTY_CYCLE,   \
           }

/*
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

// External I2C

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
#error "Unknown I2C2 clock speed"
#endif
*/

/**
 * SPI Config
 */

// External SPI
#define SPI1_GPIO_AF AF_E_SCK
#define SPI1_GPIO_PORT_MISO PAL_PORT(E_MISO)
#define SPI1_GPIO_MISO PAL_PAD(E_MISO)
#define SPI1_GPIO_PORT_MOSI PAL_PORT(E_MOSI)
#define SPI1_GPIO_MOSI PAL_PAD(E_MOSI)
#define SPI1_GPIO_PORT_SCK PAL_PORT(E_SCK)
#define SPI1_GPIO_SCK PAL_PAD(E_SCK)

#define SPI_SELECT_SLAVE0_PORT PAL_PORT(E_CS0)
#define SPI_SELECT_SLAVE0_PIN PAL_PAD(E_CS0)
#define SPI_SELECT_SLAVE1_PORT PAL_PORT(E_CS1)
#define SPI_SELECT_SLAVE1_PIN PAL_PAD(E_CS1)
#define SPI_SELECT_SLAVE2_PORT PAL_PORT(E_CS2)
#define SPI_SELECT_SLAVE2_PIN PAL_PAD(E_CS2)
#define SPI_SELECT_SLAVE3_PORT PAL_PORT(E_CS3)
#define SPI_SELECT_SLAVE3_PIN PAL_PAD(E_CS3

/**)
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

#endif /* CONFIG_TAWAKI_1_00_H */

