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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F1-Lia.
 */

/*
 * Board identifier.
 */
#define BOARD_LIA_STM32F105RC
#define BOARD_NAME              "Lia 1.1 STM32F105RC"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                12000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F105xC


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * PA0  - 4 - Digital input                    (PPM_IN)
 *      - B - Alternate Push Pull output 50MHz (SERVO5)
 * PA1  - B - Alternate Push Pull output 50MHz (SERVO6)
 * PA2  - B - Alternate Push Pull output 50MHz (UART2_TX)
 * PA3  - 4 - Digital input                    (UART2_RX)
 * PA4  - B - Alternate Push Pull output 50MHz (EXTSPI_SS)
 * PA5  - B - Alternate Push Pull output 50MHz (EXTSPI_SCK)
 * PA6  - 4 - Digital input.                   (EXTSPI_MISO)
 * PA7  - B - Alternate Push Pull output 50MHz (EXTSPI_MOSI)
 * PA8  - 7 - Open Drain output 50MHz          (LED1)
 * PA9  - 4 - Digital input.                   (USB_VBUS)
 * PA10 - 4 - Digital input.                   (UART1_RX)/(PPM_IN TIM2_CH2)
 * PA11 - 4 - Digital input                    (USB_DM)
 * PA12 - 4 - Digital input                    (USB_DP)
 * PA13 - 4 - Digital input                    (JTAG_TMS)
 * PA14 - 4 - Digital input                    (JTAG_TCK)
 * PA15 - 4 - Digital input                    (JTAG_TDI)
 */
#define VAL_GPIOACRL            0xB4BB4BBB      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x44444447      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup:
 * PB0  - 4 - Digital input                    (BARO_DRDY)
 * PB1  - 4 - Digital input                    (EXTSPI_DRDY)
 * PB2  - 4 - Digital input                    (IMU_ACC_DRDY)
 * PB3  - 4 - Digital input                    (JTAG_TDO)
 * PB4  - 7 - Open Drain output 50MHz          (LED2)
 * PB5  - 4 - Digital input                    (IMU_MAG_DRDY)
 * PB6  - B - Alternate Push Pull output 50MHz (SERVO7)
 *      - 7 - Open Drain output 50MHz.         (I2C1_SCL)
 * PB7  - B - Alternate Push Pull output 50MHz (SERVO8)
 *      - 7 - Open Drain output 50MHz.         (I2C1_SDA)
 * PB8  - 4 - Digital input.                   (CAN_RX)
 * PB9  - 7 - Open Drain output 50MHz.         (CAN_TX)
 * PB10 - E - Alternate Open Drain output 2MHz.(I2C2_SCL)
 * PB11 - E - Alternate Open Drain output 2MHz.(I2C2_SDA)
 * PB12 - 3 - Push Pull output 50MHz.          (IMU_ACC_CS)
 * PB13 - B - Alternate Push Pull output 50MHz (IMU_SPI_SCK)
 * PB14 - 4 - Digital input                    (IMU_SPI_MISO)
 * PB15 - B - Alternate Push Pull output 50MHz (IMU_SPI_MOSI)
 */
#define VAL_GPIOBCRL            0xBB474444      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0xB4B3EE74      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup:
 * PC0  - 0 - Analog input                     (ADC2)
 * PC1  - 0 - Analog input                     (ADC3)
 * PC2  - 7 - Open Drain output 50MHz           (LED3)
 * PC3  - 0 - Analog input                     (ADC1)
 * PC4  - 0 - Analog input                     (VBAT_MEAS)
 * PC5  - 7 - Open Drain output 50MHz           (LED4)
 * PC6  - B - Alternate Push Pull output 50MHz (SERVO1)
 * PC7  - B - Alternate Push Pull output 50MHz (SERVO2)
 * PC8  - B - Alternate Push Pull output 50MHz (SERVO3)
 * PC9  - B - Alternate Push Pull output 50MHz (SERVO4)
 * PC10 - B - Alternate Push Pull output 50MHz (UART3_TX)
 * PC11 - 4 - Digital input                    (UART3_RX)
 * PC12 - B - Alternate Push Pull output 50MHz (PC12-UART5_TX)
 * PC13 - 3 - Push Pull output 50MHz.          (IMU_GYRO_SS)
 * PC14 - 4 - Digital input                    (IMU_GYRO_DRDY)
 * PC15 - 7 - Open Drain output 50MHz          (LED5)
 */
#define VAL_GPIOCCRL            0xBB700700      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x743B4BBB      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup:
 * PD0  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_IN).
 * PD1  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_OUT).
 * PD2  - 4 - Digital input (UART5_RX).
 * PD3  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD4  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD5  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD6  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD7  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD8  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD9  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD10 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD11 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD12 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD13 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD14 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD15 - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 */
#define VAL_GPIODCRL            0x88888488      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * ALL  - 8 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).)
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF


/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
/* 1 red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN 8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* 2 green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN 4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/* 3 green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN 2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* 4 red, shared with ADC15 (ADC_4 on connector ANALOG2) */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN 5
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

/* 5 green, on PC15 */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOC
#define LED_5_GPIO_PIN 15
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN13
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO3
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN10
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO0
#endif

// AUX 3
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN11
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO1
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN14
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO4
#endif

// Internal Temperature sensor enabled by default
#ifndef USE_ADC_5
#define USE_ADC_5 1
#define USE_ADC_SENSOR 1
#endif
#if USE_ADC_5
#define AD1_5_CHANNEL ADC_CHANNEL_SENSOR
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO4
#endif



/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define DefaultVoltageOfAdc(adc) (0.004489*adc)

/*
 * PWM defines
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOC
#define PWM_SERVO_0_PIN GPIO6
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_DRIVER PWMD3
#define PWM_SERVO_0_CHANNEL 0
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO7
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_DRIVER PWMD3
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
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_DRIVER PWMD3
#define PWM_SERVO_2_CHANNEL 2
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD3
#define PWM_SERVO_3_CHANNEL 3
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO0
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_DRIVER PWMD5
#define PWM_SERVO_4_CHANNEL 0
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
#define PWM_SERVO_5_PIN GPIO1
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_DRIVER PWMD5
#define PWM_SERVO_5_CHANNEL 1
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif


#if USE_SERVOS_7AND8
  #if USE_I2C1
    #error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
  #else /* !USE_I2C1 */
    #if USE_PWM6
    #define PWM_SERVO_6 6
    #define PWM_SERVO_6_GPIO GPIOB
    #define PWM_SERVO_6_PIN GPIO6
    #define PWM_SERVO_6_AF GPIO_AF2
    #define PWM_SERVO_6_DRIVER PWMD4
    #define PWM_SERVO_6_CHANNEL 0
    #define PWM_SERVO_6_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
    #else
    #define PWM_SERVO_6_ACTIVE PWM_OUTPUT_DISABLED
    #endif

    #if USE_PWM7
    #define PWM_SERVO_7 7
    #define PWM_SERVO_7_GPIO GPIOB
    #define PWM_SERVO_7_PIN GPIO7
    #define PWM_SERVO_7_AF GPIO_AF2
    #define PWM_SERVO_7_DRIVER PWMD4
    #define PWM_SERVO_7_CHANNEL 1
    #define PWM_SERVO_7_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
    #else
    #define PWM_SERVO_7_ACTIVE PWM_OUTPUT_DISABLED
    #endif

    #define PWM_CONF_TIM3 1
    #define PWM_CONF_TIM4 1
    #define PWM_CONF_TIM5 1
    #define PWM_CONF3_DEF {  \
               PWM_FREQUENCY, \
               PWM_FREQUENCY/TIM3_SERVO_HZ, \
               NULL,  \
               {        \
                   {PWM_SERVO_0_ACTIVE, NULL},  \
                   {PWM_SERVO_1_ACTIVE, NULL},  \
                   {PWM_SERVO_2_ACTIVE, NULL},  \
                   {PWM_SERVO_3_ACTIVE, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF4_DEF {  \
               PWM_FREQUENCY, \
               PWM_FREQUENCY/TIM4_SERVO_HZ, \
               NULL,  \
               {        \
                   {PWM_SERVO_6_ACTIVE, NULL},  \
                   {PWM_SERVO_7_ACTIVE, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF5_DEF {  \
               PWM_FREQUENCY, \
               PWM_FREQUENCY/TIM5_SERVO_HZ, \
               NULL,  \
               {        \
                   {PWM_SERVO_4_ACTIVE, NULL},  \
                   {PWM_SERVO_5_ACTIVE, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
  #endif /* USE_I2C1 */
#else /* !USE_SERVOS_7AND8 */
  #define PWM_CONF_TIM3 1
  #define PWM_CONF_TIM5 1
    #define PWM_CONF3_DEF {  \
               PWM_FREQUENCY, \
               PWM_FREQUENCY/TIM3_SERVO_HZ, \
               NULL,  \
               {        \
                   {PWM_SERVO_0_ACTIVE, NULL},  \
                   {PWM_SERVO_1_ACTIVE, NULL},  \
                   {PWM_SERVO_2_ACTIVE, NULL},  \
                   {PWM_SERVO_3_ACTIVE, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF5_DEF {  \
               PWM_FREQUENCY, \
               PWM_FREQUENCY/TIM5_SERVO_HZ, \
               NULL,  \
               {        \
                   {PWM_SERVO_4_ACTIVE, NULL},  \
                   {PWM_SERVO_5_ACTIVE, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
#endif /* USE_SERVOS_7AND8 */


/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD1

/**
 * I2C defines
 */
#define I2C1_CLOCK_SPEED 400000
#define I2C1_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

#define I2C2_CLOCK_SPEED 400000
#define I2C2_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

/**
 * SPI Config
 */
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12

#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO4

#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12

#define SPI3_GPIO_PORT_NSS GPIO

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
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
