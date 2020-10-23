/*
 * Chris Efstathiou hendrixgr@gmail.com
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef CONFIG_MATEK_F405_WING_1_0_H
#define CONFIG_MATEK_F405_WING_1_0_H

#define BOARD_MATEK_F405_WING

/* The Matek F405 Wing autopilot has a 8MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

// Onboard LEDs
/* STAT blue, on PB5 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* WARN red, on PB4 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

// LED STRIP 2812
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOA
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

// BEEPER
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO15
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#define VBUS_GPIO   GPIOC
#define VBUS_GPIO_PIN   GPIO13

/* UART */
//CAN BE USED AS GPS SERIAL PORT
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10

// UART 2 RX INPUT IS USED AS THE PPM INPUT THUS I WILL USE THE TX OUTPUT AS ADC INPUT
#define UART2_GPIO_AF GPIO_AF8
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3

#define UART3_GPIO_AF GPIO_AF8
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2

// CAN BE USED AS A MODEM SERIAL PORT
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7

// SPI1,  MPU6000 ON SPI1
#define SPI1_GPIO_AF GPIO_AF5

#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4

// SPI2 IS USED FOR THE MAX7456 OSD
#define SPI2_GPIO_AF GPIO_AF5

#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOC
#define SPI2_GPIO_MISO GPIO2
#define SPI2_GPIO_PORT_MOSI GPIOC
#define SPI2_GPIO_MOSI GPIO3
#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12

#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12

// SDCARD ON SPI3
#define SPI3_GPIO_AF GPIO_AF5

#define SPI3_GPIO_PORT_SCK GPIOB
#define SPI3_GPIO_SCK GPIO3
#define SPI3_GPIO_PORT_MISO GPIOB
#define SPI3_GPIO_MISO GPIO4
#define SPI3_GPIO_PORT_MOSI GPIOB
#define SPI3_GPIO_MOSI GPIO5
#define SPI3_GPIO_PORT_NSS GPIOC
#define SPI3_GPIO_NSS GPIO14

#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO14


/* I2C mapping */
/* HMC5883L mag on I2C1 with DRDY on PB7 */
/* MS5611 baro on I2C1 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

// ADC

/* Onboard ADCs */
/*
   ADC1 PC2/ADC1,2,3 channel 12 (Voltage input 3.3v max)
   ADC2 PC1/ADC1,2,3 channel 11 (Current input 3.3v max)
   ADC3 PA3/ADC1,2,3 channel 3
   ADC4 PA2/ADC1,2,3 channel 2
*/

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */

#define USE_AD_TIM2 1

#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif

#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif

#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif

// POWER SUPPLY VOLTAGE MEASUREMENT INPUT
#if USE_ADC_1
#define AD1_1_CHANNEL 10
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO0
#endif

// CURRENT MEASUREMENT INPUT
#if USE_ADC_2
#define AD1_2_CHANNEL 11
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO1
#ifndef CURRENT_ADC_IN
#define CURRENT_ADC_IN ADC_2
#endif
#endif

// RSSI MEASUREMENT INPUT
#if USE_ADC_3
#define AD1_3_CHANNEL 15
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO5
#endif

// FREE, LABELED AS UART2 TX PIN
#if USE_ADC_4
#define AD1_4_CHANNEL 2
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO2
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

#ifndef CURRENT_ADC_IN
#define CURRENT_ADC_IN ADC_2
#endif

/* no voltage divider on board, adjust VoltageOfAdc in airframe file */
#define DefaultVoltageOfAdc(adc) (0.008830925*adc)
#define DefaultMilliAmpereOfAdc(adc) (25*adc)

#define UART2_RX  1
#define SERVO9_PWM_OUT  2

#if defined(RADIO_CONTROL_PPM_PIN) && RADIO_CONTROL_PPM_PIN == UART2_RX

// THE PPM INPUT IS ALSO THE UART2 RX
#define USE_PPM_TIM9        1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM1_BRK_TIM9_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO3
#define PPM_GPIO_AF         GPIO_AF3

#else

#define USE_PPM_TIM1        1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF1

#endif

// SERVO DEFINITIONS
#define PWM_USE_TIM1  0
#define PWM_USE_TIM3  1
#define PWM_USE_TIM4  1
#define PWM_USE_TIM8  1
#define PWM_USE_TIM12 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1
#define USE_PWM7 1
#define USE_PWM8 1
#define USE_PWM9 0

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM4
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO7
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO6
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC1
#define PWM_SERVO_2_OC_BIT (1<<0)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO0
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO1
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM8
#define PWM_SERVO_5_GPIO GPIOC
#define PWM_SERVO_5_PIN GPIO8
#define PWM_SERVO_5_AF GPIO_AF3
#define PWM_SERVO_5_OC TIM_OC3
#define PWM_SERVO_5_OC_BIT (1<<2)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM8
#define PWM_SERVO_6_GPIO GPIOC
#define PWM_SERVO_6_PIN GPIO9
#define PWM_SERVO_6_AF GPIO_AF3
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM12
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO14
#define PWM_SERVO_7_AF GPIO_AF9
#define PWM_SERVO_7_OC TIM_OC1
#define PWM_SERVO_7_OC_BIT (1<<0)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM12
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO15
#define PWM_SERVO_8_AF GPIO_AF9
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 8
#define PWM_SERVO_9_TIMER TIM1
#define PWM_SERVO_9_GPIO GPIOA
#define PWM_SERVO_9_PIN GPIO8
#define PWM_SERVO_9_AF GPIO_AF1
#define PWM_SERVO_9_OC TIM_OC1
#define PWM_SERVO_9_OC_BIT (1<<0)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif


// servos 1-2 on TIM4
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_1_OC_BIT | PWM_SERVO_2_OC_BIT)
// servos 3-4 on TIM3
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_3_OC_BIT | PWM_SERVO_4_OC_BIT)
// servos 5-6 on TIM8
#define PWM_TIM8_CHAN_MASK (PWM_SERVO_5_OC_BIT | PWM_SERVO_6_OC_BIT)
// servos 7-8 on TIM12
#define PWM_TIM12_CHAN_MASK (PWM_SERVO_7_OC_BIT | PWM_SERVO_8_OC_BIT)
// servo 9 on TIM1
#if USE_PWM9
#define PWM_TIM1_CHAN_MASK (PWM_SERVO_9_OC_BIT)
#endif


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO0
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF8
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2

#define SPEKTRUM_UART5_RCC RCC_UART5
#define SPEKTRUM_UART5_BANK GPIOD
#define SPEKTRUM_UART5_PIN GPIO2
#define SPEKTRUM_UART5_AF GPIO_AF8
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5



#endif // CONFIG_MATEK_F405_WING_1_0_H 
