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
#define LED_1_GPIO LED_RED_R_PORT
#define LED_1_GPIO_PIN LED_RED_R
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO LED_RED_L_PORT
#define LED_2_GPIO_PIN LED_RED_L
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO LED_GREEN_R_PORT
#define LED_3_GPIO_PIN LED_GREEN_R
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO LED_GREEN_L_PORT
#define LED_4_GPIO_PIN LED_GREEN_L
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO LED_BLUE_L_PORT
#define LED_5_GPIO_PIN LED_BLUE_L
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
#define PWM_SERVO_1_GPIO MOTOR1_PORT
#define PWM_SERVO_1_PIN MOTOR1
#define PWM_SERVO_1_AF AF_MOTOR1
#define PWM_SERVO_1_DRIVER PWMD2
#define PWM_SERVO_1_CHANNEL 1
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO MOTOR2_PORT
#define PWM_SERVO_2_PIN MOTOR2
#define PWM_SERVO_2_AF AF_MOTOR2
#define PWM_SERVO_2_DRIVER PWMD2
#define PWM_SERVO_2_CHANNEL 3
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO MOTOR3_PORT
#define PWM_SERVO_3_PIN MOTOR3
#define PWM_SERVO_3_AF AF_MOTOR3
#define PWM_SERVO_3_DRIVER PWMD2
#define PWM_SERVO_3_CHANNEL 0
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO MOTOR4_PORT
#define PWM_SERVO_4_PIN MOTOR4
#define PWM_SERVO_4_AF AF_MOTOR4
#define PWM_SERVO_4_DRIVER PWMD4
#define PWM_SERVO_4_CHANNEL 3
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
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

#ifdef STM32_PWM_USE_TIM2
#define PWM_CONF_TIM2 STM32_PWM_USE_TIM2
#else
#define PWM_CONF_TIM2 1
#endif
#define PWM_CONF2_DEF { \
  84000000, \
  256, \
  NULL, \
  { \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
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
  84000000, \
  256, \
  NULL, \
  { \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_SERVO_4_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * UART2 E_TX2
 */
#define UART2_GPIO_PORT_TX E_TX2_PORT
#define UART2_GPIO_TX E_TX2
#define UART2_GPIO_PORT_RX E_RX2_PORT
#define UART2_GPIO_RX E_RX2
#define UART2_GPIO_AF AF_E_RX2
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

/**
 * UART3 E_TX1
 */
#define UART3_GPIO_PORT_TX E_TX1_PORT
#define UART3_GPIO_TX E_TX1
#define UART3_GPIO_PORT_RX E_RX1_PORT
#define UART3_GPIO_RX E_RX1
#define UART3_GPIO_AF AF_E_RX1

/**
 * UART6 NRF
 */
#define UART6_GPIO_PORT_TX NRF_TX_PORT
#define UART6_GPIO_TX NRF_TX
#define UART6_GPIO_PORT_RX NRF_RX_PORT
#define UART6_GPIO_RX NRF_RX
#define UART6_GPIO_AF AF_NRF_RX
#define UART6_GPIO_PORT_CTS NRF_FLOW_CTRL_PORT
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
#define SPI1_GPIO_PORT_MISO E_MISO_PORT
#define SPI1_GPIO_MISO E_MISO
#define SPI1_GPIO_PORT_MOSI E_MOSI_PORT
#define SPI1_GPIO_MOSI E_MOSI
#define SPI1_GPIO_PORT_SCK E_SCK_PORT
#define SPI1_GPIO_SCK E_SCK

#define SPI_SELECT_SLAVE0_PORT E_CS0_PORT
#define SPI_SELECT_SLAVE0_PIN E_CS0
#define SPI_SELECT_SLAVE1_PORT E_CS1_PORT
#define SPI_SELECT_SLAVE1_PIN E_CS1
#define SPI_SELECT_SLAVE2_PORT E_CS2_PORT
#define SPI_SELECT_SLAVE2_PIN E_CS2
#define SPI_SELECT_SLAVE3_PORT E_CS3_PORT
#define SPI_SELECT_SLAVE3_PIN E_CS3

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
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_TAWAKI_1_00_H */

