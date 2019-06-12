/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * Board identifier.
 */
#define BOARD_ST_APOGEE
#define BOARD_NAME  "AB/GRZ STM32F4 Apogee 1.0"

/*
 * Board oscillators-related settings.
 * NOTE: LSE fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                16000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F407xx

/*
 * PAPARAZZI CONFIGURATION
 */



/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
/* red, on PC0 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* orange, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/* green, on PC1 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO1
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* yellow, on PC3 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

/* AUX1, on PB1, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOB
#define LED_5_GPIO_PIN GPIO1
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* AUX2, on PC5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_PIN GPIO5
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* AUX3, on PC4, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_PIN GPIO4
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* AUX4, on PB15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOB
#define LED_8_GPIO_PIN GPIO15
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear


/* Pint to set Uart2 RX polarity, on PB13, output high inverts, low doesn't */
#define RC_POLARITY_GPIO_PORT GPIOB
#define RC_POLARITY_GPIO_PIN GPIO13

/*
 * ADCs
 */
// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN9
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO1
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN15
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO5
#endif

// AUX 3
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN14
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO4
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN4
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define DefaultVoltageOfAdc(adc) (0.006185*adc)

/*
 * PWM defines
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOB
#define PWM_SERVO_0_PIN GPIO0
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_DRIVER PWMD3
#define PWM_SERVO_0_CHANNEL 2
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO2
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_DRIVER PWMD2
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
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO5
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_DRIVER PWMD3
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
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO4
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD3
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
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO3
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_DRIVER PWMD2
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
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO15
#define PWM_SERVO_5_AF GPIO_AF1
#define PWM_SERVO_5_DRIVER PWMD2
#define PWM_SERVO_5_CHANNEL 0
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif

#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_DRIVER PWMD3
#define PWM_SERVO_6_CHANNEL 3
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_DISABLED
#endif


#ifdef STM32_PWM_USE_TIM2
#define PWM_CONF_TIM2 STM32_PWM_USE_TIM2
#else
#define PWM_CONF_TIM2 1
#endif
#define PWM_CONF2_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM2_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_5_ACTIVE, NULL }, \
    { PWM_SERVO_4_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
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
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_0_ACTIVE, NULL }, \
    { PWM_SERVO_6_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * DSHOT
 */
#define DSHOT_SERVO_0 0
#define DSHOT_SERVO_0_GPIO GPIOB
#define DSHOT_SERVO_0_PIN GPIO0
#define DSHOT_SERVO_0_AF GPIO_AF2
#define DSHOT_SERVO_0_DRIVER DSHOTD3
#define DSHOT_SERVO_0_CHANNEL 2

#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO GPIOA
#define DSHOT_SERVO_1_PIN GPIO2
#define DSHOT_SERVO_1_AF GPIO_AF1
#define DSHOT_SERVO_1_DRIVER DSHOTD2
#define DSHOT_SERVO_1_CHANNEL 2

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO GPIOB
#define DSHOT_SERVO_2_PIN GPIO5
#define DSHOT_SERVO_2_AF GPIO_AF2
#define DSHOT_SERVO_2_DRIVER DSHOTD3
#define DSHOT_SERVO_2_CHANNEL 1

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO GPIOB
#define DSHOT_SERVO_3_PIN GPIO4
#define DSHOT_SERVO_3_AF GPIO_AF2
#define DSHOT_SERVO_3_DRIVER DSHOTD3
#define DSHOT_SERVO_3_CHANNEL 0

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO GPIOB
#define DSHOT_SERVO_4_PIN GPIO3
#define DSHOT_SERVO_4_AF GPIO_AF1
#define DSHOT_SERVO_4_DRIVER DSHOTD2
#define DSHOT_SERVO_4_CHANNEL 1

#define DSHOT_SERVO_5 5
#define DSHOT_SERVO_5_GPIO GPIOA
#define DSHOT_SERVO_5_PIN GPIO15
#define DSHOT_SERVO_5_AF GPIO_AF1
#define DSHOT_SERVO_5_DRIVER DSHOTD2
#define DSHOT_SERVO_5_CHANNEL 0

#if USE_DSHOT6
// DSHOT6 on AUX1 pin, not activated by default
#define DSHOT_SERVO_6 6
#define DSHOT_SERVO_6_GPIO GPIOB
#define DSHOT_SERVO_6_PIN GPIO1
#define DSHOT_SERVO_6_AF GPIO_AF2
#define DSHOT_SERVO_6_DRIVER DSHOTD3
#define DSHOT_SERVO_6_CHANNEL 3
#endif

#ifndef DSHOT_TELEMETRY_DEV
#define DSHOT_TELEMETRY_DEV NULL
#endif

#define DSHOT_CONF_TIM2 1
#define DSHOT_CONF2_DEF { \
  .dma_stream = STM32_PWM2_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM2_UP_DMA_CHANNEL, \
  .pwmp = &PWMD2,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV             \
}

#define DSHOT_CONF_TIM3 1
#define DSHOT_CONF3_DEF { \
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL, \
  .pwmp = &PWMD3,                           \
  .tlm_sd = DSHOT_TELEMETRY_DEV             \
}

/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 2
#define PPM_TIMER_FREQUENCY 2000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD1

/*
 * Spektrum
 */

// shorter wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/* The line that is pulled low at power up to initiate the bind process
 * PB15: AUX4
 */
#define SPEKTRUM_BIND_PIN GPIO15
#define SPEKTRUM_BIND_PIN_PORT GPIOB

/* The line used to send the pulse train for the bind process
 * When using UART2 on Apogee, this as to be a different pin than the uart2 rx
 * Default pin for this is PA8: PPM_IN
 */
#ifndef SPEKTRUM_PRIMARY_BIND_CONF_PORT
#define SPEKTRUM_PRIMARY_BIND_CONF_PORT GPIOA
#define SPEKTRUM_PRIMARY_BIND_CONF_PIN GPIO8
#endif

/*
 * PWM input
 */
// PWM_INPUT 1 on PA8 (also PPM IN)
#define PWM_INPUT1_ICU            ICUD1
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
// PPM in (aka PA8) is used: not compatible with PPM RC receiver
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO8
#define PWM_INPUT1_GPIO_AF        GPIO_AF1

// PWM_INPUT 2 on PA3 (also SERVO 1)
#if (USE_PWM1 && USE_PWM_INPUT2)
#error "PW1 and PWM_INPUT2 are not compatible"
#endif
#define PWM_INPUT2_ICU            ICUD9
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT2_GPIO_PORT      GPIOA
#define PWM_INPUT2_GPIO_PIN       GPIO2
#define PWM_INPUT2_GPIO_AF        GPIO_AF3

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
 */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

// SLAVE0 on SPI connector
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO9
// SLAVE1 on AUX1
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO1
// SLAVE2 on AUX2
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO5
// SLAVE3 on AUX3
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO4
// SLAVE4 on AUX4
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN GPIO15

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/**
 * SDIO
 */
#define SDIO_D0_PORT GPIOC
#define SDIO_D0_PIN SDIO_D0
#define SDIO_D1_PORT GPIOC
#define SDIO_D1_PIN SDIO_D1
#define SDIO_D2_PORT GPIOC
#define SDIO_D2_PIN SDIO_D2
#define SDIO_D3_PORT GPIOC
#define SDIO_D3_PIN SDIO_D3
#define SDIO_CK_PORT GPIOC
#define SDIO_CK_PIN SDIO_CK
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN SDIO_CMD
#define SDIO_AF 12
// bat monitoring for file closing
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_4_CHANNEL
// usb led status
#define SDLOG_USB_LED 4
#define SDLOG_USB_VBUS_PORT GPIOA
#define SDLOG_USB_VBUS_PIN GPIO9




/*
 * IO pins assignments.
 */
/*
 * IO pins assignments.
 */
#define	UART4_TX                       0U
#define	UART4_RX                       1U
#define	PWM2_CH3                       2U
#define	UART2_RX                       3U
#define	ADC1_IN4                       4U
#define	SPI1_SCK                       5U
#define	SPI1_MISO                      6U
#define	SPI1_MOSI                      7U
#define	CU1_CH1                        8U
#define	OTG_FS_VBUS                    9U
#define	USART1_RX                      10U
#define	OTG_FS_DM                      11U
#define	OTG_FS_DP                      12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	PWM2_CH1                       15U

#define	PWM3_CH3                       0U
#define	AUX1                           1U
#define	BOOT1                          2U
#define	PWM2_CH2                       3U
#define	PWM3_CH1                       4U
#define	PWM3_CH2                       5U
#define	USART1_TX                      6U
#define	I2C1_SDA                       7U
#define	I2C1_SCL                       8U
#define	SPI1_CS                        9U
#define	I2C2_SCL                       10U
#define	I2C2_SDA                       11U
#define	POWER_SWITCH                   12U
#define	RX2_POL                        13U
#define	SDIO_DETECT                    14U
#define	AUX4                           15U

#define	LED1                           0U
#define	LED3                           1U
#define	PC02                           2U
#define	LED4                           3U
#define	AUX3                           4U
#define	AUX2                           5U
#define	USART6_TX                      6U
#define	USART6_RX                      7U
#define	SDIO_D0                        8U
#define	SDIO_D1                        9U
#define	SDIO_D2                        10U
#define	SDIO_D3                        11U
#define	SDIO_CK                        12U
#define	LED2                           13U
#define	OSC32_IN                       14U
#define	OSC32_OUT                      15U

#define	PD00                           0U
#define	PD01                           1U
#define	SDIO_CMD                       2U
#define	PD03                           3U
#define	PD04                           4U
#define	PD05                           5U
#define	PD06                           6U
#define	PD07                           7U
#define	PD08                           8U
#define	PD09                           9U
#define	PD10                           10U
#define	PD11                           11U
#define	PD12                           12U
#define	PD13                           13U
#define	PD14                           14U
#define	PD15                           15U

#define	PE00                           0U
#define	PE01                           1U
#define	PE02                           2U
#define	PE03                           3U
#define	PE04                           4U
#define	PE05                           5U
#define	PE06                           6U
#define	PE07                           7U
#define	PE08                           8U
#define	PE09                           9U
#define	PE10                           10U
#define	PE11                           11U
#define	PE12                           12U
#define	PE13                           13U
#define	PE14                           14U
#define	PE15                           15U

#define	PF00                           0U
#define	PF01                           1U
#define	PF02                           2U
#define	PF03                           3U
#define	PF04                           4U
#define	PF05                           5U
#define	PF06                           6U
#define	PF07                           7U
#define	PF08                           8U
#define	PF09                           9U
#define	PF10                           10U
#define	PF11                           11U
#define	PF12                           12U
#define	PF13                           13U
#define	PF14                           14U
#define	PF15                           15U

#define	PG00                           0U
#define	PG01                           1U
#define	PG02                           2U
#define	PG03                           3U
#define	PG04                           4U
#define	PG05                           5U
#define	PG06                           6U
#define	PG07                           7U
#define	PG08                           8U
#define	PG09                           9U
#define	PG10                           10U
#define	PG11                           11U
#define	PG12                           12U
#define	PG13                           13U
#define	PG14                           14U
#define	PG15                           15U

#define	OSC_IN                         0U
#define	OSC_OUT                        1U
#define	PH02                           2U
#define	PH03                           3U
#define	PH04                           4U
#define	PH05                           5U
#define	PH06                           6U
#define	PH07                           7U
#define	PH08                           8U
#define	PH09                           9U
#define	PH10                           10U
#define	PH11                           11U
#define	PH12                           12U
#define	PH13                           13U
#define	PH14                           14U
#define	PH15                           15U

#define	PI00                           0U
#define	PI01                           1U
#define	PI02                           2U
#define	PI03                           3U
#define	PI04                           4U
#define	PI05                           5U
#define	PI06                           6U
#define	PI07                           7U
#define	PI08                           8U
#define	PI09                           9U
#define	PI10                           10U
#define	PI11                           11U
#define	PI12                           12U
#define	PI13                           13U
#define	PI14                           14U
#define	PI15                           15U

#define	PJ00                           0U
#define	PJ01                           1U
#define	PJ02                           2U
#define	PJ03                           3U
#define	PJ04                           4U
#define	PJ05                           5U
#define	PJ06                           6U
#define	PJ07                           7U
#define	PJ08                           8U
#define	PJ09                           9U
#define	PJ10                           10U
#define	PJ11                           11U
#define	PJ12                           12U
#define	PJ13                           13U
#define	PJ14                           14U
#define	PJ15                           15U

#define	PK00                           0U
#define	PK01                           1U
#define	PK02                           2U
#define	PK03                           3U
#define	PK04                           4U
#define	PK05                           5U
#define	PK06                           6U
#define	PK07                           7U
#define	PK08                           8U
#define	PK09                           9U
#define	PK10                           10U
#define	PK11                           11U
#define	PK12                           12U
#define	PK13                           13U
#define	PK14                           14U
#define	PK15                           15U

/*
 * IO lines assignments.
 */
#define	LINE_UART4_TX                  PAL_LINE(GPIOA, 0U)
#define	LINE_UART4_RX                  PAL_LINE(GPIOA, 1U)
#define	LINE_PWM2_CH3                  PAL_LINE(GPIOA, 2U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOA, 3U)
#define	LINE_ADC1_IN4                  PAL_LINE(GPIOA, 4U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOA, 7U)
#define	LINE_CU1_CH1                   PAL_LINE(GPIOA, 8U)
#define	LINE_OTG_FS_VBUS               PAL_LINE(GPIOA, 9U)
#define	LINE_USART1_RX                 PAL_LINE(GPIOA, 10U)
#define	LINE_OTG_FS_DM                 PAL_LINE(GPIOA, 11U)
#define	LINE_OTG_FS_DP                 PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_PWM2_CH1                  PAL_LINE(GPIOA, 15U)

#define	LINE_PWM3_CH3                  PAL_LINE(GPIOB, 0U)
#define	LINE_AUX1                      PAL_LINE(GPIOB, 1U)
#define	LINE_BOOT1                     PAL_LINE(GPIOB, 2U)
#define	LINE_PWM2_CH2                  PAL_LINE(GPIOB, 3U)
#define	LINE_PWM3_CH1                  PAL_LINE(GPIOB, 4U)
#define	LINE_PWM3_CH2                  PAL_LINE(GPIOB, 5U)
#define	LINE_USART1_TX                 PAL_LINE(GPIOB, 6U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 7U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U)
#define	LINE_SPI1_CS                   PAL_LINE(GPIOB, 9U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define	LINE_POWER_SWITCH              PAL_LINE(GPIOB, 12U)
#define	LINE_RX2_POL                   PAL_LINE(GPIOB, 13U)
#define	LINE_SDIO_DETECT               PAL_LINE(GPIOB, 14U)
#define	LINE_AUX4                      PAL_LINE(GPIOB, 15U)

#define	LINE_LED1                      PAL_LINE(GPIOC, 0U)
#define	LINE_LED3                      PAL_LINE(GPIOC, 1U)
#define	LINE_LED4                      PAL_LINE(GPIOC, 3U)
#define	LINE_AUX3                      PAL_LINE(GPIOC, 4U)
#define	LINE_AUX2                      PAL_LINE(GPIOC, 5U)
#define	LINE_USART6_TX                 PAL_LINE(GPIOC, 6U)
#define	LINE_USART6_RX                 PAL_LINE(GPIOC, 7U)
#define	LINE_SDIO_D0                   PAL_LINE(GPIOC, 8U)
#define	LINE_SDIO_D1                   PAL_LINE(GPIOC, 9U)
#define	LINE_SDIO_D2                   PAL_LINE(GPIOC, 10U)
#define	LINE_SDIO_D3                   PAL_LINE(GPIOC, 11U)
#define	LINE_SDIO_CK                   PAL_LINE(GPIOC, 12U)
#define	LINE_LED2                      PAL_LINE(GPIOC, 13U)
#define	LINE_OSC32_IN                  PAL_LINE(GPIOC, 14U)
#define	LINE_OSC32_OUT                 PAL_LINE(GPIOC, 15U)

#define	LINE_SDIO_CMD                  PAL_LINE(GPIOD, 2U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(UART4_TX) | \
					 PIN_MODE_ALTERNATE(UART4_RX) | \
					 PIN_MODE_INPUT(PWM2_CH3) | \
					 PIN_MODE_ALTERNATE(UART2_RX) | \
					 PIN_MODE_ANALOG(ADC1_IN4) | \
					 PIN_MODE_ALTERNATE(SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(SPI1_MISO) | \
					 PIN_MODE_ALTERNATE(SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(CU1_CH1) | \
					 PIN_MODE_INPUT(OTG_FS_VBUS) | \
					 PIN_MODE_ALTERNATE(USART1_RX) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DM) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_INPUT(PWM2_CH1))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(UART4_RX) | \
					 PIN_OTYPE_OPENDRAIN(PWM2_CH3) | \
					 PIN_OTYPE_PUSHPULL(UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(ADC1_IN4) | \
					 PIN_OTYPE_PUSHPULL(SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(CU1_CH1) | \
					 PIN_OTYPE_OPENDRAIN(OTG_FS_VBUS) | \
					 PIN_OTYPE_PUSHPULL(USART1_RX) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DM) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_OPENDRAIN(PWM2_CH1))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART4_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PWM2_CH3) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC1_IN4) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(CU1_CH1) | \
					 PIN_OSPEED_SPEED_VERYLOW(OTG_FS_VBUS) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DM) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PWM2_CH1))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(UART4_TX) | \
					 PIN_PUPDR_FLOATING(UART4_RX) | \
					 PIN_PUPDR_FLOATING(PWM2_CH3) | \
					 PIN_PUPDR_FLOATING(UART2_RX) | \
					 PIN_PUPDR_FLOATING(ADC1_IN4) | \
					 PIN_PUPDR_FLOATING(SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(CU1_CH1) | \
					 PIN_PUPDR_PULLDOWN(OTG_FS_VBUS) | \
					 PIN_PUPDR_FLOATING(USART1_RX) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DM) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_FLOATING(PWM2_CH1))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART4_RX) | \
					 PIN_ODR_LEVEL_LOW(PWM2_CH3) | \
					 PIN_ODR_LEVEL_HIGH(UART2_RX) | \
					 PIN_ODR_LEVEL_LOW(ADC1_IN4) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(CU1_CH1) | \
					 PIN_ODR_LEVEL_LOW(OTG_FS_VBUS) | \
					 PIN_ODR_LEVEL_HIGH(USART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DM) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PWM2_CH1))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(UART4_TX, 8) | \
					 PIN_AFIO_AF(UART4_RX, 8) | \
					 PIN_AFIO_AF(PWM2_CH3, 0) | \
					 PIN_AFIO_AF(UART2_RX, 7) | \
					 PIN_AFIO_AF(ADC1_IN4, 0) | \
					 PIN_AFIO_AF(SPI1_SCK, 5) | \
					 PIN_AFIO_AF(SPI1_MISO, 5) | \
					 PIN_AFIO_AF(SPI1_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(CU1_CH1, 1) | \
					 PIN_AFIO_AF(OTG_FS_VBUS, 0) | \
					 PIN_AFIO_AF(USART1_RX, 7) | \
					 PIN_AFIO_AF(OTG_FS_DM, 10) | \
					 PIN_AFIO_AF(OTG_FS_DP, 10) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(PWM2_CH1, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(PWM3_CH3) | \
					 PIN_MODE_INPUT(AUX1) | \
					 PIN_MODE_INPUT(BOOT1) | \
					 PIN_MODE_INPUT(PWM2_CH2) | \
					 PIN_MODE_INPUT(PWM3_CH1) | \
					 PIN_MODE_INPUT(PWM3_CH2) | \
					 PIN_MODE_ALTERNATE(USART1_TX) | \
					 PIN_MODE_ALTERNATE(I2C1_SDA) | \
					 PIN_MODE_ALTERNATE(I2C1_SCL) | \
					 PIN_MODE_OUTPUT(SPI1_CS) | \
					 PIN_MODE_ALTERNATE(I2C2_SCL) | \
					 PIN_MODE_ALTERNATE(I2C2_SDA) | \
					 PIN_MODE_OUTPUT(POWER_SWITCH) | \
					 PIN_MODE_OUTPUT(RX2_POL) | \
					 PIN_MODE_INPUT(SDIO_DETECT) | \
					 PIN_MODE_INPUT(AUX4))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_OPENDRAIN(PWM3_CH3) | \
					 PIN_OTYPE_OPENDRAIN(AUX1) | \
					 PIN_OTYPE_OPENDRAIN(BOOT1) | \
					 PIN_OTYPE_OPENDRAIN(PWM2_CH2) | \
					 PIN_OTYPE_OPENDRAIN(PWM3_CH1) | \
					 PIN_OTYPE_OPENDRAIN(PWM3_CH2) | \
					 PIN_OTYPE_PUSHPULL(USART1_TX) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SDA) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SCL) | \
					 PIN_OTYPE_PUSHPULL(SPI1_CS) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SDA) | \
					 PIN_OTYPE_PUSHPULL(POWER_SWITCH) | \
					 PIN_OTYPE_PUSHPULL(RX2_POL) | \
					 PIN_OTYPE_OPENDRAIN(SDIO_DETECT) | \
					 PIN_OTYPE_OPENDRAIN(AUX4))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PWM3_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX1) | \
					 PIN_OSPEED_SPEED_VERYLOW(BOOT1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PWM2_CH2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PWM3_CH1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PWM3_CH2) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_CS) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(POWER_SWITCH) | \
					 PIN_OSPEED_SPEED_HIGH(RX2_POL) | \
					 PIN_OSPEED_SPEED_VERYLOW(SDIO_DETECT) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX4))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PWM3_CH3) | \
					 PIN_PUPDR_FLOATING(AUX1) | \
					 PIN_PUPDR_FLOATING(BOOT1) | \
					 PIN_PUPDR_FLOATING(PWM2_CH2) | \
					 PIN_PUPDR_FLOATING(PWM3_CH1) | \
					 PIN_PUPDR_FLOATING(PWM3_CH2) | \
					 PIN_PUPDR_FLOATING(USART1_TX) | \
					 PIN_PUPDR_PULLUP(I2C1_SDA) | \
					 PIN_PUPDR_PULLUP(I2C1_SCL) | \
					 PIN_PUPDR_FLOATING(SPI1_CS) | \
					 PIN_PUPDR_PULLUP(I2C2_SCL) | \
					 PIN_PUPDR_PULLUP(I2C2_SDA) | \
					 PIN_PUPDR_FLOATING(POWER_SWITCH) | \
					 PIN_PUPDR_FLOATING(RX2_POL) | \
					 PIN_PUPDR_PULLUP(SDIO_DETECT) | \
					 PIN_PUPDR_FLOATING(AUX4))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PWM3_CH3) | \
					 PIN_ODR_LEVEL_LOW(AUX1) | \
					 PIN_ODR_LEVEL_LOW(BOOT1) | \
					 PIN_ODR_LEVEL_LOW(PWM2_CH2) | \
					 PIN_ODR_LEVEL_LOW(PWM3_CH1) | \
					 PIN_ODR_LEVEL_LOW(PWM3_CH2) | \
					 PIN_ODR_LEVEL_HIGH(USART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_CS) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(POWER_SWITCH) | \
					 PIN_ODR_LEVEL_HIGH(RX2_POL) | \
					 PIN_ODR_LEVEL_LOW(SDIO_DETECT) | \
					 PIN_ODR_LEVEL_LOW(AUX4))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PWM3_CH3, 0) | \
					 PIN_AFIO_AF(AUX1, 0) | \
					 PIN_AFIO_AF(BOOT1, 0) | \
					 PIN_AFIO_AF(PWM2_CH2, 0) | \
					 PIN_AFIO_AF(PWM3_CH1, 0) | \
					 PIN_AFIO_AF(PWM3_CH2, 0) | \
					 PIN_AFIO_AF(USART1_TX, 7) | \
					 PIN_AFIO_AF(I2C1_SDA, 4))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(I2C1_SCL, 4) | \
					 PIN_AFIO_AF(SPI1_CS, 0) | \
					 PIN_AFIO_AF(I2C2_SCL, 4) | \
					 PIN_AFIO_AF(I2C2_SDA, 4) | \
					 PIN_AFIO_AF(POWER_SWITCH, 0) | \
					 PIN_AFIO_AF(RX2_POL, 0) | \
					 PIN_AFIO_AF(SDIO_DETECT, 0) | \
					 PIN_AFIO_AF(AUX4, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_OUTPUT(LED1) | \
					 PIN_MODE_OUTPUT(LED3) | \
					 PIN_MODE_INPUT(PC02) | \
					 PIN_MODE_OUTPUT(LED4) | \
					 PIN_MODE_INPUT(AUX3) | \
					 PIN_MODE_INPUT(AUX2) | \
					 PIN_MODE_ALTERNATE(USART6_TX) | \
					 PIN_MODE_ALTERNATE(USART6_RX) | \
					 PIN_MODE_ALTERNATE(SDIO_D0) | \
					 PIN_MODE_ALTERNATE(SDIO_D1) | \
					 PIN_MODE_ALTERNATE(SDIO_D2) | \
					 PIN_MODE_ALTERNATE(SDIO_D3) | \
					 PIN_MODE_ALTERNATE(SDIO_CK) | \
					 PIN_MODE_OUTPUT(LED2) | \
					 PIN_MODE_ALTERNATE(OSC32_IN) | \
					 PIN_MODE_ALTERNATE(OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(LED1) | \
					 PIN_OTYPE_PUSHPULL(LED3) | \
					 PIN_OTYPE_PUSHPULL(PC02) | \
					 PIN_OTYPE_PUSHPULL(LED4) | \
					 PIN_OTYPE_OPENDRAIN(AUX3) | \
					 PIN_OTYPE_OPENDRAIN(AUX2) | \
					 PIN_OTYPE_PUSHPULL(USART6_TX) | \
					 PIN_OTYPE_PUSHPULL(USART6_RX) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D0) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D1) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D2) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D3) | \
					 PIN_OTYPE_PUSHPULL(SDIO_CK) | \
					 PIN_OTYPE_PUSHPULL(LED2) | \
					 PIN_OTYPE_PUSHPULL(OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED4) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX3) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX2) | \
					 PIN_OSPEED_SPEED_HIGH(USART6_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART6_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED2) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(LED1) | \
					 PIN_PUPDR_FLOATING(LED3) | \
					 PIN_PUPDR_PULLDOWN(PC02) | \
					 PIN_PUPDR_FLOATING(LED4) | \
					 PIN_PUPDR_FLOATING(AUX3) | \
					 PIN_PUPDR_FLOATING(AUX2) | \
					 PIN_PUPDR_FLOATING(USART6_TX) | \
					 PIN_PUPDR_FLOATING(USART6_RX) | \
					 PIN_PUPDR_PULLUP(SDIO_D0) | \
					 PIN_PUPDR_PULLUP(SDIO_D1) | \
					 PIN_PUPDR_PULLUP(SDIO_D2) | \
					 PIN_PUPDR_PULLUP(SDIO_D3) | \
					 PIN_PUPDR_FLOATING(SDIO_CK) | \
					 PIN_PUPDR_FLOATING(LED2) | \
					 PIN_PUPDR_FLOATING(OSC32_IN) | \
					 PIN_PUPDR_FLOATING(OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(LED1) | \
					 PIN_ODR_LEVEL_LOW(LED3) | \
					 PIN_ODR_LEVEL_LOW(PC02) | \
					 PIN_ODR_LEVEL_LOW(LED4) | \
					 PIN_ODR_LEVEL_LOW(AUX3) | \
					 PIN_ODR_LEVEL_LOW(AUX2) | \
					 PIN_ODR_LEVEL_HIGH(USART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART6_RX) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_CK) | \
					 PIN_ODR_LEVEL_LOW(LED2) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(LED1, 0) | \
					 PIN_AFIO_AF(LED3, 0) | \
					 PIN_AFIO_AF(PC02, 0) | \
					 PIN_AFIO_AF(LED4, 0) | \
					 PIN_AFIO_AF(AUX3, 0) | \
					 PIN_AFIO_AF(AUX2, 0) | \
					 PIN_AFIO_AF(USART6_TX, 8) | \
					 PIN_AFIO_AF(USART6_RX, 8))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDIO_D0, 12) | \
					 PIN_AFIO_AF(SDIO_D1, 12) | \
					 PIN_AFIO_AF(SDIO_D2, 12) | \
					 PIN_AFIO_AF(SDIO_D3, 12) | \
					 PIN_AFIO_AF(SDIO_CK, 12) | \
					 PIN_AFIO_AF(LED2, 0) | \
					 PIN_AFIO_AF(OSC32_IN, 0) | \
					 PIN_AFIO_AF(OSC32_OUT, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(PD00) | \
					 PIN_MODE_INPUT(PD01) | \
					 PIN_MODE_ALTERNATE(SDIO_CMD) | \
					 PIN_MODE_INPUT(PD03) | \
					 PIN_MODE_INPUT(PD04) | \
					 PIN_MODE_INPUT(PD05) | \
					 PIN_MODE_INPUT(PD06) | \
					 PIN_MODE_INPUT(PD07) | \
					 PIN_MODE_INPUT(PD08) | \
					 PIN_MODE_INPUT(PD09) | \
					 PIN_MODE_INPUT(PD10) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_INPUT(PD12) | \
					 PIN_MODE_INPUT(PD13) | \
					 PIN_MODE_INPUT(PD14) | \
					 PIN_MODE_INPUT(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00) | \
					 PIN_OTYPE_PUSHPULL(PD01) | \
					 PIN_OTYPE_PUSHPULL(SDIO_CMD) | \
					 PIN_OTYPE_PUSHPULL(PD03) | \
					 PIN_OTYPE_PUSHPULL(PD04) | \
					 PIN_OTYPE_PUSHPULL(PD05) | \
					 PIN_OTYPE_PUSHPULL(PD06) | \
					 PIN_OTYPE_PUSHPULL(PD07) | \
					 PIN_OTYPE_PUSHPULL(PD08) | \
					 PIN_OTYPE_PUSHPULL(PD09) | \
					 PIN_OTYPE_PUSHPULL(PD10) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_PUSHPULL(PD12) | \
					 PIN_OTYPE_PUSHPULL(PD13) | \
					 PIN_OTYPE_PUSHPULL(PD14) | \
					 PIN_OTYPE_PUSHPULL(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PD00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD01) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_PULLDOWN(PD00) | \
					 PIN_PUPDR_PULLDOWN(PD01) | \
					 PIN_PUPDR_PULLUP(SDIO_CMD) | \
					 PIN_PUPDR_PULLDOWN(PD03) | \
					 PIN_PUPDR_PULLDOWN(PD04) | \
					 PIN_PUPDR_PULLDOWN(PD05) | \
					 PIN_PUPDR_PULLDOWN(PD06) | \
					 PIN_PUPDR_PULLDOWN(PD07) | \
					 PIN_PUPDR_PULLDOWN(PD08) | \
					 PIN_PUPDR_PULLDOWN(PD09) | \
					 PIN_PUPDR_PULLDOWN(PD10) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_PULLDOWN(PD12) | \
					 PIN_PUPDR_PULLDOWN(PD13) | \
					 PIN_PUPDR_PULLDOWN(PD14) | \
					 PIN_PUPDR_PULLDOWN(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_LOW(PD00) | \
					 PIN_ODR_LEVEL_LOW(PD01) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_CMD) | \
					 PIN_ODR_LEVEL_LOW(PD03) | \
					 PIN_ODR_LEVEL_LOW(PD04) | \
					 PIN_ODR_LEVEL_LOW(PD05) | \
					 PIN_ODR_LEVEL_LOW(PD06) | \
					 PIN_ODR_LEVEL_LOW(PD07) | \
					 PIN_ODR_LEVEL_LOW(PD08) | \
					 PIN_ODR_LEVEL_LOW(PD09) | \
					 PIN_ODR_LEVEL_LOW(PD10) | \
					 PIN_ODR_LEVEL_LOW(PD11) | \
					 PIN_ODR_LEVEL_LOW(PD12) | \
					 PIN_ODR_LEVEL_LOW(PD13) | \
					 PIN_ODR_LEVEL_LOW(PD14) | \
					 PIN_ODR_LEVEL_LOW(PD15))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00, 0) | \
					 PIN_AFIO_AF(PD01, 0) | \
					 PIN_AFIO_AF(SDIO_CMD, 12) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(PD05, 0) | \
					 PIN_AFIO_AF(PD06, 0) | \
					 PIN_AFIO_AF(PD07, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08, 0) | \
					 PIN_AFIO_AF(PD09, 0) | \
					 PIN_AFIO_AF(PD10, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12, 0) | \
					 PIN_AFIO_AF(PD13, 0) | \
					 PIN_AFIO_AF(PD14, 0) | \
					 PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_INPUT(PE00) | \
					 PIN_MODE_INPUT(PE01) | \
					 PIN_MODE_INPUT(PE02) | \
					 PIN_MODE_INPUT(PE03) | \
					 PIN_MODE_INPUT(PE04) | \
					 PIN_MODE_INPUT(PE05) | \
					 PIN_MODE_INPUT(PE06) | \
					 PIN_MODE_INPUT(PE07) | \
					 PIN_MODE_INPUT(PE08) | \
					 PIN_MODE_INPUT(PE09) | \
					 PIN_MODE_INPUT(PE10) | \
					 PIN_MODE_INPUT(PE11) | \
					 PIN_MODE_INPUT(PE12) | \
					 PIN_MODE_INPUT(PE13) | \
					 PIN_MODE_INPUT(PE14) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00) | \
					 PIN_OTYPE_PUSHPULL(PE01) | \
					 PIN_OTYPE_PUSHPULL(PE02) | \
					 PIN_OTYPE_PUSHPULL(PE03) | \
					 PIN_OTYPE_PUSHPULL(PE04) | \
					 PIN_OTYPE_PUSHPULL(PE05) | \
					 PIN_OTYPE_PUSHPULL(PE06) | \
					 PIN_OTYPE_PUSHPULL(PE07) | \
					 PIN_OTYPE_PUSHPULL(PE08) | \
					 PIN_OTYPE_PUSHPULL(PE09) | \
					 PIN_OTYPE_PUSHPULL(PE10) | \
					 PIN_OTYPE_PUSHPULL(PE11) | \
					 PIN_OTYPE_PUSHPULL(PE12) | \
					 PIN_OTYPE_PUSHPULL(PE13) | \
					 PIN_OTYPE_PUSHPULL(PE14) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PE00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_PULLDOWN(PE00) | \
					 PIN_PUPDR_PULLDOWN(PE01) | \
					 PIN_PUPDR_PULLDOWN(PE02) | \
					 PIN_PUPDR_PULLDOWN(PE03) | \
					 PIN_PUPDR_PULLDOWN(PE04) | \
					 PIN_PUPDR_PULLDOWN(PE05) | \
					 PIN_PUPDR_PULLDOWN(PE06) | \
					 PIN_PUPDR_PULLDOWN(PE07) | \
					 PIN_PUPDR_PULLDOWN(PE08) | \
					 PIN_PUPDR_PULLDOWN(PE09) | \
					 PIN_PUPDR_PULLDOWN(PE10) | \
					 PIN_PUPDR_PULLDOWN(PE11) | \
					 PIN_PUPDR_PULLDOWN(PE12) | \
					 PIN_PUPDR_PULLDOWN(PE13) | \
					 PIN_PUPDR_PULLDOWN(PE14) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_LOW(PE00) | \
					 PIN_ODR_LEVEL_LOW(PE01) | \
					 PIN_ODR_LEVEL_LOW(PE02) | \
					 PIN_ODR_LEVEL_LOW(PE03) | \
					 PIN_ODR_LEVEL_LOW(PE04) | \
					 PIN_ODR_LEVEL_LOW(PE05) | \
					 PIN_ODR_LEVEL_LOW(PE06) | \
					 PIN_ODR_LEVEL_LOW(PE07) | \
					 PIN_ODR_LEVEL_LOW(PE08) | \
					 PIN_ODR_LEVEL_LOW(PE09) | \
					 PIN_ODR_LEVEL_LOW(PE10) | \
					 PIN_ODR_LEVEL_LOW(PE11) | \
					 PIN_ODR_LEVEL_LOW(PE12) | \
					 PIN_ODR_LEVEL_LOW(PE13) | \
					 PIN_ODR_LEVEL_LOW(PE14) | \
					 PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00, 0) | \
					 PIN_AFIO_AF(PE01, 0) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(PE04, 0) | \
					 PIN_AFIO_AF(PE05, 0) | \
					 PIN_AFIO_AF(PE06, 0) | \
					 PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08, 0) | \
					 PIN_AFIO_AF(PE09, 0) | \
					 PIN_AFIO_AF(PE10, 0) | \
					 PIN_AFIO_AF(PE11, 0) | \
					 PIN_AFIO_AF(PE12, 0) | \
					 PIN_AFIO_AF(PE13, 0) | \
					 PIN_AFIO_AF(PE14, 0) | \
					 PIN_AFIO_AF(PE15, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_INPUT(PF00) | \
					 PIN_MODE_INPUT(PF01) | \
					 PIN_MODE_INPUT(PF02) | \
					 PIN_MODE_INPUT(PF03) | \
					 PIN_MODE_INPUT(PF04) | \
					 PIN_MODE_INPUT(PF05) | \
					 PIN_MODE_INPUT(PF06) | \
					 PIN_MODE_INPUT(PF07) | \
					 PIN_MODE_INPUT(PF08) | \
					 PIN_MODE_INPUT(PF09) | \
					 PIN_MODE_INPUT(PF10) | \
					 PIN_MODE_INPUT(PF11) | \
					 PIN_MODE_INPUT(PF12) | \
					 PIN_MODE_INPUT(PF13) | \
					 PIN_MODE_INPUT(PF14) | \
					 PIN_MODE_INPUT(PF15))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_PUSHPULL(PF00) | \
					 PIN_OTYPE_PUSHPULL(PF01) | \
					 PIN_OTYPE_PUSHPULL(PF02) | \
					 PIN_OTYPE_PUSHPULL(PF03) | \
					 PIN_OTYPE_PUSHPULL(PF04) | \
					 PIN_OTYPE_PUSHPULL(PF05) | \
					 PIN_OTYPE_PUSHPULL(PF06) | \
					 PIN_OTYPE_PUSHPULL(PF07) | \
					 PIN_OTYPE_PUSHPULL(PF08) | \
					 PIN_OTYPE_PUSHPULL(PF09) | \
					 PIN_OTYPE_PUSHPULL(PF10) | \
					 PIN_OTYPE_PUSHPULL(PF11) | \
					 PIN_OTYPE_PUSHPULL(PF12) | \
					 PIN_OTYPE_PUSHPULL(PF13) | \
					 PIN_OTYPE_PUSHPULL(PF14) | \
					 PIN_OTYPE_PUSHPULL(PF15))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PF00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF15))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLDOWN(PF00) | \
					 PIN_PUPDR_PULLDOWN(PF01) | \
					 PIN_PUPDR_PULLDOWN(PF02) | \
					 PIN_PUPDR_PULLDOWN(PF03) | \
					 PIN_PUPDR_PULLDOWN(PF04) | \
					 PIN_PUPDR_PULLDOWN(PF05) | \
					 PIN_PUPDR_PULLDOWN(PF06) | \
					 PIN_PUPDR_PULLDOWN(PF07) | \
					 PIN_PUPDR_PULLDOWN(PF08) | \
					 PIN_PUPDR_PULLDOWN(PF09) | \
					 PIN_PUPDR_PULLDOWN(PF10) | \
					 PIN_PUPDR_PULLDOWN(PF11) | \
					 PIN_PUPDR_PULLDOWN(PF12) | \
					 PIN_PUPDR_PULLDOWN(PF13) | \
					 PIN_PUPDR_PULLDOWN(PF14) | \
					 PIN_PUPDR_PULLDOWN(PF15))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_LOW(PF00) | \
					 PIN_ODR_LEVEL_LOW(PF01) | \
					 PIN_ODR_LEVEL_LOW(PF02) | \
					 PIN_ODR_LEVEL_LOW(PF03) | \
					 PIN_ODR_LEVEL_LOW(PF04) | \
					 PIN_ODR_LEVEL_LOW(PF05) | \
					 PIN_ODR_LEVEL_LOW(PF06) | \
					 PIN_ODR_LEVEL_LOW(PF07) | \
					 PIN_ODR_LEVEL_LOW(PF08) | \
					 PIN_ODR_LEVEL_LOW(PF09) | \
					 PIN_ODR_LEVEL_LOW(PF10) | \
					 PIN_ODR_LEVEL_LOW(PF11) | \
					 PIN_ODR_LEVEL_LOW(PF12) | \
					 PIN_ODR_LEVEL_LOW(PF13) | \
					 PIN_ODR_LEVEL_LOW(PF14) | \
					 PIN_ODR_LEVEL_LOW(PF15))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00, 0) | \
					 PIN_AFIO_AF(PF01, 0) | \
					 PIN_AFIO_AF(PF02, 0) | \
					 PIN_AFIO_AF(PF03, 0) | \
					 PIN_AFIO_AF(PF04, 0) | \
					 PIN_AFIO_AF(PF05, 0) | \
					 PIN_AFIO_AF(PF06, 0) | \
					 PIN_AFIO_AF(PF07, 0))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08, 0) | \
					 PIN_AFIO_AF(PF09, 0) | \
					 PIN_AFIO_AF(PF10, 0) | \
					 PIN_AFIO_AF(PF11, 0) | \
					 PIN_AFIO_AF(PF12, 0) | \
					 PIN_AFIO_AF(PF13, 0) | \
					 PIN_AFIO_AF(PF14, 0) | \
					 PIN_AFIO_AF(PF15, 0))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00) | \
					 PIN_MODE_INPUT(PG01) | \
					 PIN_MODE_INPUT(PG02) | \
					 PIN_MODE_INPUT(PG03) | \
					 PIN_MODE_INPUT(PG04) | \
					 PIN_MODE_INPUT(PG05) | \
					 PIN_MODE_INPUT(PG06) | \
					 PIN_MODE_INPUT(PG07) | \
					 PIN_MODE_INPUT(PG08) | \
					 PIN_MODE_INPUT(PG09) | \
					 PIN_MODE_INPUT(PG10) | \
					 PIN_MODE_INPUT(PG11) | \
					 PIN_MODE_INPUT(PG12) | \
					 PIN_MODE_INPUT(PG13) | \
					 PIN_MODE_INPUT(PG14) | \
					 PIN_MODE_INPUT(PG15))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(PG00) | \
					 PIN_OTYPE_PUSHPULL(PG01) | \
					 PIN_OTYPE_PUSHPULL(PG02) | \
					 PIN_OTYPE_PUSHPULL(PG03) | \
					 PIN_OTYPE_PUSHPULL(PG04) | \
					 PIN_OTYPE_PUSHPULL(PG05) | \
					 PIN_OTYPE_PUSHPULL(PG06) | \
					 PIN_OTYPE_PUSHPULL(PG07) | \
					 PIN_OTYPE_PUSHPULL(PG08) | \
					 PIN_OTYPE_PUSHPULL(PG09) | \
					 PIN_OTYPE_PUSHPULL(PG10) | \
					 PIN_OTYPE_PUSHPULL(PG11) | \
					 PIN_OTYPE_PUSHPULL(PG12) | \
					 PIN_OTYPE_PUSHPULL(PG13) | \
					 PIN_OTYPE_PUSHPULL(PG14) | \
					 PIN_OTYPE_PUSHPULL(PG15))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00) | \
					 PIN_PUPDR_PULLDOWN(PG01) | \
					 PIN_PUPDR_PULLDOWN(PG02) | \
					 PIN_PUPDR_PULLDOWN(PG03) | \
					 PIN_PUPDR_PULLDOWN(PG04) | \
					 PIN_PUPDR_PULLDOWN(PG05) | \
					 PIN_PUPDR_PULLDOWN(PG06) | \
					 PIN_PUPDR_PULLDOWN(PG07) | \
					 PIN_PUPDR_PULLDOWN(PG08) | \
					 PIN_PUPDR_PULLDOWN(PG09) | \
					 PIN_PUPDR_PULLDOWN(PG10) | \
					 PIN_PUPDR_PULLDOWN(PG11) | \
					 PIN_PUPDR_PULLDOWN(PG12) | \
					 PIN_PUPDR_PULLDOWN(PG13) | \
					 PIN_PUPDR_PULLDOWN(PG14) | \
					 PIN_PUPDR_PULLDOWN(PG15))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(PG00) | \
					 PIN_ODR_LEVEL_LOW(PG01) | \
					 PIN_ODR_LEVEL_LOW(PG02) | \
					 PIN_ODR_LEVEL_LOW(PG03) | \
					 PIN_ODR_LEVEL_LOW(PG04) | \
					 PIN_ODR_LEVEL_LOW(PG05) | \
					 PIN_ODR_LEVEL_LOW(PG06) | \
					 PIN_ODR_LEVEL_LOW(PG07) | \
					 PIN_ODR_LEVEL_LOW(PG08) | \
					 PIN_ODR_LEVEL_LOW(PG09) | \
					 PIN_ODR_LEVEL_LOW(PG10) | \
					 PIN_ODR_LEVEL_LOW(PG11) | \
					 PIN_ODR_LEVEL_LOW(PG12) | \
					 PIN_ODR_LEVEL_LOW(PG13) | \
					 PIN_ODR_LEVEL_LOW(PG14) | \
					 PIN_ODR_LEVEL_LOW(PG15))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00, 0) | \
					 PIN_AFIO_AF(PG01, 0) | \
					 PIN_AFIO_AF(PG02, 0) | \
					 PIN_AFIO_AF(PG03, 0) | \
					 PIN_AFIO_AF(PG04, 0) | \
					 PIN_AFIO_AF(PG05, 0) | \
					 PIN_AFIO_AF(PG06, 0) | \
					 PIN_AFIO_AF(PG07, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08, 0) | \
					 PIN_AFIO_AF(PG09, 0) | \
					 PIN_AFIO_AF(PG10, 0) | \
					 PIN_AFIO_AF(PG11, 0) | \
					 PIN_AFIO_AF(PG12, 0) | \
					 PIN_AFIO_AF(PG13, 0) | \
					 PIN_AFIO_AF(PG14, 0) | \
					 PIN_AFIO_AF(PG15, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(OSC_IN) | \
					 PIN_MODE_ALTERNATE(OSC_OUT) | \
					 PIN_MODE_INPUT(PH02) | \
					 PIN_MODE_INPUT(PH03) | \
					 PIN_MODE_INPUT(PH04) | \
					 PIN_MODE_INPUT(PH05) | \
					 PIN_MODE_INPUT(PH06) | \
					 PIN_MODE_INPUT(PH07) | \
					 PIN_MODE_INPUT(PH08) | \
					 PIN_MODE_INPUT(PH09) | \
					 PIN_MODE_INPUT(PH10) | \
					 PIN_MODE_INPUT(PH11) | \
					 PIN_MODE_INPUT(PH12) | \
					 PIN_MODE_INPUT(PH13) | \
					 PIN_MODE_INPUT(PH14) | \
					 PIN_MODE_INPUT(PH15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(PH02) | \
					 PIN_OTYPE_PUSHPULL(PH03) | \
					 PIN_OTYPE_PUSHPULL(PH04) | \
					 PIN_OTYPE_PUSHPULL(PH05) | \
					 PIN_OTYPE_PUSHPULL(PH06) | \
					 PIN_OTYPE_PUSHPULL(PH07) | \
					 PIN_OTYPE_PUSHPULL(PH08) | \
					 PIN_OTYPE_PUSHPULL(PH09) | \
					 PIN_OTYPE_PUSHPULL(PH10) | \
					 PIN_OTYPE_PUSHPULL(PH11) | \
					 PIN_OTYPE_PUSHPULL(PH12) | \
					 PIN_OTYPE_PUSHPULL(PH13) | \
					 PIN_OTYPE_PUSHPULL(PH14) | \
					 PIN_OTYPE_PUSHPULL(PH15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(OSC_IN) | \
					 PIN_PUPDR_FLOATING(OSC_OUT) | \
					 PIN_PUPDR_PULLDOWN(PH02) | \
					 PIN_PUPDR_PULLDOWN(PH03) | \
					 PIN_PUPDR_PULLDOWN(PH04) | \
					 PIN_PUPDR_PULLDOWN(PH05) | \
					 PIN_PUPDR_PULLDOWN(PH06) | \
					 PIN_PUPDR_PULLDOWN(PH07) | \
					 PIN_PUPDR_PULLDOWN(PH08) | \
					 PIN_PUPDR_PULLDOWN(PH09) | \
					 PIN_PUPDR_PULLDOWN(PH10) | \
					 PIN_PUPDR_PULLDOWN(PH11) | \
					 PIN_PUPDR_PULLDOWN(PH12) | \
					 PIN_PUPDR_PULLDOWN(PH13) | \
					 PIN_PUPDR_PULLDOWN(PH14) | \
					 PIN_PUPDR_PULLDOWN(PH15))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC_OUT) | \
					 PIN_ODR_LEVEL_LOW(PH02) | \
					 PIN_ODR_LEVEL_LOW(PH03) | \
					 PIN_ODR_LEVEL_LOW(PH04) | \
					 PIN_ODR_LEVEL_LOW(PH05) | \
					 PIN_ODR_LEVEL_LOW(PH06) | \
					 PIN_ODR_LEVEL_LOW(PH07) | \
					 PIN_ODR_LEVEL_LOW(PH08) | \
					 PIN_ODR_LEVEL_LOW(PH09) | \
					 PIN_ODR_LEVEL_LOW(PH10) | \
					 PIN_ODR_LEVEL_LOW(PH11) | \
					 PIN_ODR_LEVEL_LOW(PH12) | \
					 PIN_ODR_LEVEL_LOW(PH13) | \
					 PIN_ODR_LEVEL_LOW(PH14) | \
					 PIN_ODR_LEVEL_LOW(PH15))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(OSC_IN, 0) | \
					 PIN_AFIO_AF(OSC_OUT, 0) | \
					 PIN_AFIO_AF(PH02, 0) | \
					 PIN_AFIO_AF(PH03, 0) | \
					 PIN_AFIO_AF(PH04, 0) | \
					 PIN_AFIO_AF(PH05, 0) | \
					 PIN_AFIO_AF(PH06, 0) | \
					 PIN_AFIO_AF(PH07, 0))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08, 0) | \
					 PIN_AFIO_AF(PH09, 0) | \
					 PIN_AFIO_AF(PH10, 0) | \
					 PIN_AFIO_AF(PH11, 0) | \
					 PIN_AFIO_AF(PH12, 0) | \
					 PIN_AFIO_AF(PH13, 0) | \
					 PIN_AFIO_AF(PH14, 0) | \
					 PIN_AFIO_AF(PH15, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_INPUT(PI00) | \
					 PIN_MODE_INPUT(PI01) | \
					 PIN_MODE_INPUT(PI02) | \
					 PIN_MODE_INPUT(PI03) | \
					 PIN_MODE_INPUT(PI04) | \
					 PIN_MODE_INPUT(PI05) | \
					 PIN_MODE_INPUT(PI06) | \
					 PIN_MODE_INPUT(PI07) | \
					 PIN_MODE_INPUT(PI08) | \
					 PIN_MODE_INPUT(PI09) | \
					 PIN_MODE_INPUT(PI10) | \
					 PIN_MODE_INPUT(PI11) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00) | \
					 PIN_OTYPE_PUSHPULL(PI01) | \
					 PIN_OTYPE_PUSHPULL(PI02) | \
					 PIN_OTYPE_PUSHPULL(PI03) | \
					 PIN_OTYPE_PUSHPULL(PI04) | \
					 PIN_OTYPE_PUSHPULL(PI05) | \
					 PIN_OTYPE_PUSHPULL(PI06) | \
					 PIN_OTYPE_PUSHPULL(PI07) | \
					 PIN_OTYPE_PUSHPULL(PI08) | \
					 PIN_OTYPE_PUSHPULL(PI09) | \
					 PIN_OTYPE_PUSHPULL(PI10) | \
					 PIN_OTYPE_PUSHPULL(PI11) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PI00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_PULLDOWN(PI00) | \
					 PIN_PUPDR_PULLDOWN(PI01) | \
					 PIN_PUPDR_PULLDOWN(PI02) | \
					 PIN_PUPDR_PULLDOWN(PI03) | \
					 PIN_PUPDR_PULLDOWN(PI04) | \
					 PIN_PUPDR_PULLDOWN(PI05) | \
					 PIN_PUPDR_PULLDOWN(PI06) | \
					 PIN_PUPDR_PULLDOWN(PI07) | \
					 PIN_PUPDR_PULLDOWN(PI08) | \
					 PIN_PUPDR_PULLDOWN(PI09) | \
					 PIN_PUPDR_PULLDOWN(PI10) | \
					 PIN_PUPDR_PULLDOWN(PI11) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00) | \
					 PIN_ODR_LEVEL_LOW(PI01) | \
					 PIN_ODR_LEVEL_LOW(PI02) | \
					 PIN_ODR_LEVEL_LOW(PI03) | \
					 PIN_ODR_LEVEL_LOW(PI04) | \
					 PIN_ODR_LEVEL_LOW(PI05) | \
					 PIN_ODR_LEVEL_LOW(PI06) | \
					 PIN_ODR_LEVEL_LOW(PI07) | \
					 PIN_ODR_LEVEL_LOW(PI08) | \
					 PIN_ODR_LEVEL_LOW(PI09) | \
					 PIN_ODR_LEVEL_LOW(PI10) | \
					 PIN_ODR_LEVEL_LOW(PI11) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(PI00, 0) | \
					 PIN_AFIO_AF(PI01, 0) | \
					 PIN_AFIO_AF(PI02, 0) | \
					 PIN_AFIO_AF(PI03, 0) | \
					 PIN_AFIO_AF(PI04, 0) | \
					 PIN_AFIO_AF(PI05, 0) | \
					 PIN_AFIO_AF(PI06, 0) | \
					 PIN_AFIO_AF(PI07, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(PI08, 0) | \
					 PIN_AFIO_AF(PI09, 0) | \
					 PIN_AFIO_AF(PI10, 0) | \
					 PIN_AFIO_AF(PI11, 0) | \
					 PIN_AFIO_AF(PI12, 0) | \
					 PIN_AFIO_AF(PI13, 0) | \
					 PIN_AFIO_AF(PI14, 0) | \
					 PIN_AFIO_AF(PI15, 0))

#define VAL_GPIOJ_MODER                 (PIN_MODE_INPUT(PJ00) | \
					 PIN_MODE_INPUT(PJ01) | \
					 PIN_MODE_INPUT(PJ02) | \
					 PIN_MODE_INPUT(PJ03) | \
					 PIN_MODE_INPUT(PJ04) | \
					 PIN_MODE_INPUT(PJ05) | \
					 PIN_MODE_INPUT(PJ06) | \
					 PIN_MODE_INPUT(PJ07) | \
					 PIN_MODE_INPUT(PJ08) | \
					 PIN_MODE_INPUT(PJ09) | \
					 PIN_MODE_INPUT(PJ10) | \
					 PIN_MODE_INPUT(PJ11) | \
					 PIN_MODE_INPUT(PJ12) | \
					 PIN_MODE_INPUT(PJ13) | \
					 PIN_MODE_INPUT(PJ14) | \
					 PIN_MODE_INPUT(PJ15))

#define VAL_GPIOJ_OTYPER                (PIN_OTYPE_PUSHPULL(PJ00) | \
					 PIN_OTYPE_PUSHPULL(PJ01) | \
					 PIN_OTYPE_PUSHPULL(PJ02) | \
					 PIN_OTYPE_PUSHPULL(PJ03) | \
					 PIN_OTYPE_PUSHPULL(PJ04) | \
					 PIN_OTYPE_PUSHPULL(PJ05) | \
					 PIN_OTYPE_PUSHPULL(PJ06) | \
					 PIN_OTYPE_PUSHPULL(PJ07) | \
					 PIN_OTYPE_PUSHPULL(PJ08) | \
					 PIN_OTYPE_PUSHPULL(PJ09) | \
					 PIN_OTYPE_PUSHPULL(PJ10) | \
					 PIN_OTYPE_PUSHPULL(PJ11) | \
					 PIN_OTYPE_PUSHPULL(PJ12) | \
					 PIN_OTYPE_PUSHPULL(PJ13) | \
					 PIN_OTYPE_PUSHPULL(PJ14) | \
					 PIN_OTYPE_PUSHPULL(PJ15))

#define VAL_GPIOJ_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PJ00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ15))

#define VAL_GPIOJ_PUPDR                 (PIN_PUPDR_PULLDOWN(PJ00) | \
					 PIN_PUPDR_PULLDOWN(PJ01) | \
					 PIN_PUPDR_PULLDOWN(PJ02) | \
					 PIN_PUPDR_PULLDOWN(PJ03) | \
					 PIN_PUPDR_PULLDOWN(PJ04) | \
					 PIN_PUPDR_PULLDOWN(PJ05) | \
					 PIN_PUPDR_PULLDOWN(PJ06) | \
					 PIN_PUPDR_PULLDOWN(PJ07) | \
					 PIN_PUPDR_PULLDOWN(PJ08) | \
					 PIN_PUPDR_PULLDOWN(PJ09) | \
					 PIN_PUPDR_PULLDOWN(PJ10) | \
					 PIN_PUPDR_PULLDOWN(PJ11) | \
					 PIN_PUPDR_PULLDOWN(PJ12) | \
					 PIN_PUPDR_PULLDOWN(PJ13) | \
					 PIN_PUPDR_PULLDOWN(PJ14) | \
					 PIN_PUPDR_PULLDOWN(PJ15))

#define VAL_GPIOJ_ODR                   (PIN_ODR_LEVEL_LOW(PJ00) | \
					 PIN_ODR_LEVEL_LOW(PJ01) | \
					 PIN_ODR_LEVEL_LOW(PJ02) | \
					 PIN_ODR_LEVEL_LOW(PJ03) | \
					 PIN_ODR_LEVEL_LOW(PJ04) | \
					 PIN_ODR_LEVEL_LOW(PJ05) | \
					 PIN_ODR_LEVEL_LOW(PJ06) | \
					 PIN_ODR_LEVEL_LOW(PJ07) | \
					 PIN_ODR_LEVEL_LOW(PJ08) | \
					 PIN_ODR_LEVEL_LOW(PJ09) | \
					 PIN_ODR_LEVEL_LOW(PJ10) | \
					 PIN_ODR_LEVEL_LOW(PJ11) | \
					 PIN_ODR_LEVEL_LOW(PJ12) | \
					 PIN_ODR_LEVEL_LOW(PJ13) | \
					 PIN_ODR_LEVEL_LOW(PJ14) | \
					 PIN_ODR_LEVEL_LOW(PJ15))

#define VAL_GPIOJ_AFRL			(PIN_AFIO_AF(PJ00, 0) | \
					 PIN_AFIO_AF(PJ01, 0) | \
					 PIN_AFIO_AF(PJ02, 0) | \
					 PIN_AFIO_AF(PJ03, 0) | \
					 PIN_AFIO_AF(PJ04, 0) | \
					 PIN_AFIO_AF(PJ05, 0) | \
					 PIN_AFIO_AF(PJ06, 0) | \
					 PIN_AFIO_AF(PJ07, 0))

#define VAL_GPIOJ_AFRH			(PIN_AFIO_AF(PJ08, 0) | \
					 PIN_AFIO_AF(PJ09, 0) | \
					 PIN_AFIO_AF(PJ10, 0) | \
					 PIN_AFIO_AF(PJ11, 0) | \
					 PIN_AFIO_AF(PJ12, 0) | \
					 PIN_AFIO_AF(PJ13, 0) | \
					 PIN_AFIO_AF(PJ14, 0) | \
					 PIN_AFIO_AF(PJ15, 0))

#define VAL_GPIOK_MODER                 (PIN_MODE_INPUT(PK00) | \
					 PIN_MODE_INPUT(PK01) | \
					 PIN_MODE_INPUT(PK02) | \
					 PIN_MODE_INPUT(PK03) | \
					 PIN_MODE_INPUT(PK04) | \
					 PIN_MODE_INPUT(PK05) | \
					 PIN_MODE_INPUT(PK06) | \
					 PIN_MODE_INPUT(PK07) | \
					 PIN_MODE_INPUT(PK08) | \
					 PIN_MODE_INPUT(PK09) | \
					 PIN_MODE_INPUT(PK10) | \
					 PIN_MODE_INPUT(PK11) | \
					 PIN_MODE_INPUT(PK12) | \
					 PIN_MODE_INPUT(PK13) | \
					 PIN_MODE_INPUT(PK14) | \
					 PIN_MODE_INPUT(PK15))

#define VAL_GPIOK_OTYPER                (PIN_OTYPE_PUSHPULL(PK00) | \
					 PIN_OTYPE_PUSHPULL(PK01) | \
					 PIN_OTYPE_PUSHPULL(PK02) | \
					 PIN_OTYPE_PUSHPULL(PK03) | \
					 PIN_OTYPE_PUSHPULL(PK04) | \
					 PIN_OTYPE_PUSHPULL(PK05) | \
					 PIN_OTYPE_PUSHPULL(PK06) | \
					 PIN_OTYPE_PUSHPULL(PK07) | \
					 PIN_OTYPE_PUSHPULL(PK08) | \
					 PIN_OTYPE_PUSHPULL(PK09) | \
					 PIN_OTYPE_PUSHPULL(PK10) | \
					 PIN_OTYPE_PUSHPULL(PK11) | \
					 PIN_OTYPE_PUSHPULL(PK12) | \
					 PIN_OTYPE_PUSHPULL(PK13) | \
					 PIN_OTYPE_PUSHPULL(PK14) | \
					 PIN_OTYPE_PUSHPULL(PK15))

#define VAL_GPIOK_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PK00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK15))

#define VAL_GPIOK_PUPDR                 (PIN_PUPDR_PULLDOWN(PK00) | \
					 PIN_PUPDR_PULLDOWN(PK01) | \
					 PIN_PUPDR_PULLDOWN(PK02) | \
					 PIN_PUPDR_PULLDOWN(PK03) | \
					 PIN_PUPDR_PULLDOWN(PK04) | \
					 PIN_PUPDR_PULLDOWN(PK05) | \
					 PIN_PUPDR_PULLDOWN(PK06) | \
					 PIN_PUPDR_PULLDOWN(PK07) | \
					 PIN_PUPDR_PULLDOWN(PK08) | \
					 PIN_PUPDR_PULLDOWN(PK09) | \
					 PIN_PUPDR_PULLDOWN(PK10) | \
					 PIN_PUPDR_PULLDOWN(PK11) | \
					 PIN_PUPDR_PULLDOWN(PK12) | \
					 PIN_PUPDR_PULLDOWN(PK13) | \
					 PIN_PUPDR_PULLDOWN(PK14) | \
					 PIN_PUPDR_PULLDOWN(PK15))

#define VAL_GPIOK_ODR                   (PIN_ODR_LEVEL_LOW(PK00) | \
					 PIN_ODR_LEVEL_LOW(PK01) | \
					 PIN_ODR_LEVEL_LOW(PK02) | \
					 PIN_ODR_LEVEL_LOW(PK03) | \
					 PIN_ODR_LEVEL_LOW(PK04) | \
					 PIN_ODR_LEVEL_LOW(PK05) | \
					 PIN_ODR_LEVEL_LOW(PK06) | \
					 PIN_ODR_LEVEL_LOW(PK07) | \
					 PIN_ODR_LEVEL_LOW(PK08) | \
					 PIN_ODR_LEVEL_LOW(PK09) | \
					 PIN_ODR_LEVEL_LOW(PK10) | \
					 PIN_ODR_LEVEL_LOW(PK11) | \
					 PIN_ODR_LEVEL_LOW(PK12) | \
					 PIN_ODR_LEVEL_LOW(PK13) | \
					 PIN_ODR_LEVEL_LOW(PK14) | \
					 PIN_ODR_LEVEL_LOW(PK15))

#define VAL_GPIOK_AFRL			(PIN_AFIO_AF(PK00, 0) | \
					 PIN_AFIO_AF(PK01, 0) | \
					 PIN_AFIO_AF(PK02, 0) | \
					 PIN_AFIO_AF(PK03, 0) | \
					 PIN_AFIO_AF(PK04, 0) | \
					 PIN_AFIO_AF(PK05, 0) | \
					 PIN_AFIO_AF(PK06, 0) | \
					 PIN_AFIO_AF(PK07, 0))

#define VAL_GPIOK_AFRH			(PIN_AFIO_AF(PK08, 0) | \
					 PIN_AFIO_AF(PK09, 0) | \
					 PIN_AFIO_AF(PK10, 0) | \
					 PIN_AFIO_AF(PK11, 0) | \
					 PIN_AFIO_AF(PK12, 0) | \
					 PIN_AFIO_AF(PK13, 0) | \
					 PIN_AFIO_AF(PK14, 0) | \
					 PIN_AFIO_AF(PK15, 0))

#define AF_UART4_TX                      8U
#define AF_LINE_UART4_TX                 8U
#define AF_UART4_RX                      8U
#define AF_LINE_UART4_RX                 8U
#define AF_UART2_RX                      7U
#define AF_LINE_UART2_RX                 7U
#define AF_SPI1_SCK                      5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_SPI1_MISO                     5U
#define AF_LINE_SPI1_MISO                5U
#define AF_SPI1_MOSI                     5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_CU1_CH1                       1U
#define AF_LINE_CU1_CH1                  1U
#define AF_USART1_RX                     7U
#define AF_LINE_USART1_RX                7U
#define AF_OTG_FS_DM                     10U
#define AF_LINE_OTG_FS_DM                10U
#define AF_OTG_FS_DP                     10U
#define AF_LINE_OTG_FS_DP                10U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_USART1_TX                     7U
#define AF_LINE_USART1_TX                7U
#define AF_I2C1_SDA                      4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_I2C1_SCL                      4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_I2C2_SCL                      4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_I2C2_SDA                      4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_USART6_TX                     8U
#define AF_LINE_USART6_TX                8U
#define AF_USART6_RX                     8U
#define AF_LINE_USART6_RX                8U
#define AF_SDIO_D0                       12U
#define AF_LINE_SDIO_D0                  12U
#define AF_SDIO_D1                       12U
#define AF_LINE_SDIO_D1                  12U
#define AF_SDIO_D2                       12U
#define AF_LINE_SDIO_D2                  12U
#define AF_SDIO_D3                       12U
#define AF_LINE_SDIO_D3                  12U
#define AF_SDIO_CK                       12U
#define AF_LINE_SDIO_CK                  12U
#define AF_OSC32_IN                      0U
#define AF_LINE_OSC32_IN                 0U
#define AF_OSC32_OUT                     0U
#define AF_LINE_OSC32_OUT                0U
#define AF_SDIO_CMD                      12U
#define AF_LINE_SDIO_CMD                 12U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

