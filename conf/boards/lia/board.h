/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    modified by: AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
    Utah State University, http://aggieair.usu.edu/

    Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
    Calvin Coopmans (c.r.coopmans@ieee.org)

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
 * Setup for the Lia board
 */

/*
 * Board identifier.
 */
#define BOARD_LIA_STM32F105RC
#define BOARD_NAME              "Lia 1.1 STM32F105RC"

/*
 * Board frequencies.
 */
#define STM32_HSECLK            12000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_CL

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
 * LEDs
 */
/* 1 red, on PA8 */
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN 8

/* 2 green, shared with JTAG_TRST */
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN 4

/* 3 green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN 2

/* 4 red, shared with ADC15 (ADC_4 on connector ANALOG2) */
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN 5

/* 5 green, on PC15 */
#define LED_5_GPIO GPIOC
#define LED_5_GPIO_PIN 15

/*
 * ADCs
 */
#define BOARD_ADC_CHANNEL_1 13
#define BOARD_ADC_CHANNEL_2 10
#define BOARD_ADC_CHANNEL_3 11
// we can only use ADC1,2,3; the last channel is for bat monitoring
#define BOARD_ADC_CHANNEL_4 14

/*
 * provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 0
#define ADC_2 1
#define ADC_3 2

// allow to define ADC_CHANNEL_VSUPPLY in the airframe file
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
#endif

#ifndef ADC_CHANNEL_TEMP_SENSOR
#define ADC_CHANNEL_TEMP_SENSOR 4
#endif

// active ADC
#define USE_AD1 1
#define USE_AD1_1 1
#define USE_AD1_2 1
#define USE_AD1_3 1
#define USE_AD1_4 1

#define DefaultVoltageOfAdc(adc) (0.0047*adc)

// Read the electrical characteristics for STM32F105 chip
#define CpuTempOfAdc(adc) ((1430 - adc)/4.3+25)

/*
 * PWM defines
 */
#define PWM_FREQUENCY_1MHZ 1000000
#define PWM_CMD_TO_US(_t) _t

#define PWM_SERVO_1 0
#define PWM_SERVO_1_DRIVER PWMD3
#define PWM_SERVO_1_CHANNEL 0

#define PWM_SERVO_2 1
#define PWM_SERVO_2_DRIVER PWMD3
#define PWM_SERVO_2_CHANNEL 1

#define PWM_SERVO_3 2
#define PWM_SERVO_3_DRIVER PWMD3
#define PWM_SERVO_3_CHANNEL 2

#define PWM_SERVO_4 3
#define PWM_SERVO_4_DRIVER PWMD3
#define PWM_SERVO_4_CHANNEL 3

#define PWM_SERVO_5 4
#define PWM_SERVO_5_DRIVER PWMD5
#define PWM_SERVO_5_CHANNEL 0

#define PWM_SERVO_6 5
#define PWM_SERVO_6_DRIVER PWMD5
#define PWM_SERVO_6_CHANNEL 1



/*
#define ACTUATORS_PWM_NB 4
#define PWM_CONF_TIM3 1
#define PWM_CONF3_DEF {  \
          PWM_FREQUENCY_1MHZ, \
          PWM_FREQUENCY_1MHZ/SERVO_HZ, \
          pwmpcb,  \
          {        \
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
            {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          },       \
          0,       \
          0        \
        }
*/


#if USE_SERVOS_7AND8
  #if USE_I2C1
    #error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
  #else
    #define PWM_SERVO_7 6
    #define PWM_SERVO_7_DRIVER PWMD4
    #define PWM_SERVO_7_CHANNEL 0

    #define PWM_SERVO_8 7
    #define PWM_SERVO_8_DRIVER PWMD4
    #define PWM_SERVO_8_CHANNEL 1

    #define ACTUATORS_PWM_NB 8
    #define PWM_CONF_TIM3 1
    #define PWM_CONF_TIM4 1
    #define PWM_CONF_TIM5 1
    #define PWM_CONF3_DEF {  \
               PWM_FREQUENCY_1MHZ, \
               PWM_FREQUENCY_1MHZ/SERVO_HZ, \
               pwmpcb,  \
               {        \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF4_DEF {  \
               PWM_FREQUENCY_1MHZ, \
               PWM_FREQUENCY_1MHZ/SERVO_HZ, \
               pwmpcb,  \
               {        \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF5_DEF {  \
               PWM_FREQUENCY_1MHZ, \
               PWM_FREQUENCY_1MHZ/SERVO_HZ, \
               pwmpcb,  \
               {        \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
  #endif
#else
  #define ACTUATORS_PWM_NB 6
  #define PWM_CONF_TIM3 1
  #define PWM_CONF_TIM5 1
    #define PWM_CONF3_DEF {  \
               PWM_FREQUENCY_1MHZ, \
               PWM_FREQUENCY_1MHZ/SERVO_HZ, \
               pwmpcb,  \
               {        \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
               },       \
               0,       \
               0        \
               }
    #define PWM_CONF5_DEF {  \
               PWM_FREQUENCY_1MHZ, \
               PWM_FREQUENCY_1MHZ/SERVO_HZ, \
               pwmpcb,  \
               {        \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL},  \
                   {PWM_OUTPUT_DISABLED, NULL}  \
               },       \
               0,       \
               0        \
               }
#endif


/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_3
#define PPM_TIMER ICUD1

/**
 * I2C2 defines
 */
#define I2C1_CLOCK_SPEED 300000
#define I2C1_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

#define I2C2_CLOCK_SPEED 300000
#define I2C2_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

/**
 * SPI Config
 *
 * Just defines which make sense for Lia board
 */
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN      4

#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN     12

#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN     13

#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN     12

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
