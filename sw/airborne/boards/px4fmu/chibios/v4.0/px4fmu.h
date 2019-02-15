#ifndef CONFIG_PX4FMU_4_00_H
#define CONFIG_PX4FMU_4_00_H

#define BOARD_PX4FMU

/**
 * ChibiOS board file
 */
#include "boards/px4fmu/chibios/v4.0/board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * Onboard LEDs
 */
/* red, on PE12 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO11
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, on PB1 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO1
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* blue, on PB3 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOB
#define LED_3_GPIO_PIN GPIO3
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * ADCs 
 */
// VOLT_SENS
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN2
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO2
#endif

// CUR_SENS
#ifndef USE_ADC_2
#define USE_ADC_2 2
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN3
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO3
#endif

// VDD_V5_SENS
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN4
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

/* Default powerbrick values */
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f) * 13.653333333f * adc)
#define MilliAmpereOfAdc(adc) ((3.3f/4096.0f) * 36.367515152f * adc)

/*
 * PWM defines TODO
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOE
#define PWM_SERVO_0_PIN GPIO14
#define PWM_SERVO_0_AF GPIO_AF1
#define PWM_SERVO_0_DRIVER PWMD1
#define PWM_SERVO_0_CHANNEL 3
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO13
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_DRIVER PWMD1
#define PWM_SERVO_1_CHANNEL 2
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
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_DRIVER PWMD1
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
#define PWM_SERVO_4_GPIO GPIOD
#define PWM_SERVO_4_PIN GPIO13
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_DRIVER PWMD4
#define PWM_SERVO_4_CHANNEL 1
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO GPIOD
#define PWM_SERVO_5_PIN GPIO14
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_DRIVER PWMD4
#define PWM_SERVO_5_CHANNEL 2
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
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
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_SERVO_0_ACTIVE, NULL }, \
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
    { PWM_SERVO_4_ACTIVE, NULL }, \
    { PWM_SERVO_5_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * UART defines
 */
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_AF 7

#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_AF 7

#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_AF 7

#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_AF 8

#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_AF 8

#define UART7_GPIO_PORT_TX GPIOE
#define UART7_GPIO_TX GPIO8
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_AF 8

#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO1
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_AF 8

/* Soft binding Spektrum */
#define RADIO_CONTROL_POWER_PORT GPIOE
#define RADIO_CONTROL_POWER_PIN GPIO4 //SPEKTRUM POWER
#define RADIO_CONTROL_POWER_ON gpio_clear // yes, inverted
#define RADIO_CONTROL_POWER_OFF gpio_set

//A receiver on powered on 3.3v
#define PERIPHERAL3V3_ENABLE_PORT GPIOC //VDD_3V3_PERIPHERAL_EN
#define PERIPHERAL3V3_ENABLE_PIN GPIO5
#define PERIPHERAL3V3_ENABLE_ON gpio_set
#define PERIPHERAL3V3_ENABLE_OFF gpio_clear

// /**
//  * PPM radio defines TODO
//  */
// #define RC_PPM_TICKS_PER_USEC 2
// #define PPM_TIMER_FREQUENCY 2000000
// #define PPM_CHANNEL ICU_CHANNEL_1
// #define PPM_TIMER ICUD1

// /*
//  * PWM input TODO
//  */
// // PWM_INPUT 1 on PA8 (also PPM IN)
// #define PWM_INPUT1_ICU            ICUD1
// #define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
// // PPM in (aka PA8) is used: not compatible with PPM RC receiver
// #define PWM_INPUT1_GPIO_PORT      GPIOA
// #define PWM_INPUT1_GPIO_PIN       GPIO8
// #define PWM_INPUT1_GPIO_AF        GPIO_AF1


/**
 * I2C defines
 */
#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif
#if I2C1_CLOCK_SPEED == 400000
#define I2C1_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C1 clock speed
#endif
#define I2C1_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           I2C1_DUTY_CYCLE,   \
           }

#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif
#if I2C2_CLOCK_SPEED == 400000
#define I2C2_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C2 clock speed
#endif
#define I2C2_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           I2C2_DUTY_CYCLE,   \
           }


/**
 * SPI Config
 * SPI1 si for MPU9250
 * SPI2 is for FRAM
 */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO10

/* SPI1_SLAVE1 -> slave select pin for the ICM 20609-G*/
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO15
/* SPI1_SLAVE1 -> slave select pin for the HMC5983 */
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO15
// SPI1_SLAVE2 -> slave select pin for the MPU9250
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO2
// SPI1_SLAVE3 -> slave select pin for the ms5611
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7
// SPI1_SLAVE4 -> slave select pin for the FRAM
#define SPI_SELECT_SLAVE4_PORT GPIOD
#define SPI_SELECT_SLAVE4_PIN GPIO10

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
#define SDLOG_BAT_CHAN AD1_1_CHANNEL
// usb led status
#define SDLOG_USB_LED 3
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

#endif /* CONFIG_PX4FMU_4_00_H */

