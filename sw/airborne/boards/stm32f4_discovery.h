/*
 * STM32F4 DISCOVERY BOARD DEFINITIONS FOR THE PAPARAZZI AUTOPILOT.
 *
 * DISCOVERY USED FOR THE AUTOPILOT PINS
 * PA0  = PWM9 + user-wake up button switch b1 normaly open.
 * PA1  = PWM8
 * PA2  = PWM7
 * PA3  = PWM6
 * PA4  = ADC_4 (ADC12 IN 4)+CS43L22 DAC out, capacitively coupled+100Kohm, should not interfere.
 * PA5  = SPI1 SCK if STM32F4_DISCOVERY_SPI1_FOR_LIS302
 * PA6  = SPI1 MISO if STM32F4_DISCOVERY_SPI1_FOR_LIS302
 * PA7  = SPI1 MOSI if STM32F4_DISCOVERY_SPI1_FOR_LIS302
 * PA8  = SPECTRUM BIND
 * PA9  = FREE (ONLY if usb is not active during runtime, PC0 must be high or input )
 * PA10 = UART1 RX (Spektrum input)
 * PA11 = FREE if usb is not active during runtime
 * PA12 = FREE if usb is not active during runtime
 * PA13 = FREE
 * PA14 = FREE
 * PA15 = FREE
 *
 * PB0  = FREE
 * PB1  = ADC_1 (ADC12 IN 9)
 * PB2  = FREE
 * PB3  = SPI1 SCL
 * PB4  = SPI1 MISO
 * PB5  = SPI1 MOSI
 * PB6  = UART1 TX + 4K7 Ohm pull up resistor normaly used for i2c
 * PB7  = UART1 RX
 * PB8  = I2C1 SCL
 * PB9  = I2C1 SDA + 4K7 Ohm pull up resistor normaly used for i2c
 * PB10 = I2C2 SCL + MP45DT02 MEMS MIC clock in
 * PB11 = I2C2 SDA
 * PB12 = FREE
 * PB13 = SPI2 SCL
 * PB14 = PWM 10 OR SPI2 MISO
 * PB15 = PWM 11 OR SPI2 MOSI
 *
 * PC0  = CANNOT BE USED, pulled up to 3.3v (10Kohm), low = usb power on directly applied to PA9)
 * PC1  = ADC_5 (ADC123 IN 11)
 * PC2  = ADC_6 (ADC123 IN 12)
 * PC3  = CANNOT BE USED (MEMS microphone output)
 * PC4  = ADC_3 (ADC12 IN 14)
 * PC5  = ADC_2 (ADC12 IN 15)
 * PC6  = UART6 TX
 * PC7  = UART6 RX
 * PC8  = FREE
 * PC9  = PPM INPUT FROM RC RECEIVER
 * PC10 = UART4 TX OR SPI3 SCL
 * PC11 = UART4 RX OR SPI3 MISO
 * PC12 = UART5 TX OR SPI3 MOSI
 * PC13 = FREE max 3ma sink
 * PC14 = FREE max 3ma sink, short circuit the relevant PCB bridge with solder to activate
 * PC15 = FREE max 3ma sink, short circuit the relevant PCB bridge with solder to activate
 *
 * PD0  = FREE
 * PD1  = FREE
 * PD2  = UART5 RX
 * PD3  = FREE
 * PD4  = FREE
 * PD5  = UART2 TX (usb power management fault output open drain)
 * PD6  = UART2 RX
 * PD7  = FREE
 * PD8  = UART3 TX
 * PD9  = UART3 RX
 * PD10 = FREE
 * PD11 = FREE
 * PD12 = LED_4
 * PD13 = LED_3
 * PD14 = LED_5
 * PD15 = LED_6
 *
 * PE0  = CANNOT BE USED (Accel int output)
 * PE1  = CANNOT BE USED (Accel int output)
 * PE2  = SPI SLAVE 0
 * PE3  = SPI SLAVE 2 (used for the LIS302DL accel spi/i2c select pin)
 * PE4  = FREE
 * PE5  = PWM4
 * PE6  = PWM5
 * PE7  = SPI SLAVE 1
 * PE8  = FREE
 * PE9  = PWM0
 * PE10 = FREE
 * PE11 = PWM1
 * PE12 = FREE
 * PE13 = PWM2
 * PE14 = PWM3
 * PE15 = FREE
 *
 * PH0  = CRYSTAL - OSC IN
 * PH1  = CRYSTAL - OSC OUT
 */

#ifndef CONFIG_STM32F4_DISCOVERY_H
#define CONFIG_STM32F4_DISCOVERY_H

#define BOARD_STM32F4_DISCOVERY

/* STM32F4_DISCOVERY has a 8MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* orange, on PD13 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOD
#define LED_3_GPIO_PIN GPIO13
#define LED_3_AFIO_REMAP ((void)0)
#define LED_3_GPIO_ON gpio_set
#define LED_3_GPIO_OFF gpio_clear

/* green, on PD12 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOD
#define LED_4_GPIO_PIN GPIO12
#define LED_4_AFIO_REMAP ((void)0)
#define LED_4_GPIO_ON gpio_set
#define LED_4_GPIO_OFF gpio_clear

/* red, PD14 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOD
#define LED_5_GPIO_PIN GPIO14
#define LED_5_AFIO_REMAP ((void)0)
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* blue, PD15 */
#ifndef USE_LED_6
#define USE_LED_6 1
#endif
#define LED_6_GPIO GPIOD
#define LED_6_GPIO_PIN GPIO15
#define LED_6_AFIO_REMAP ((void)0)
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear



/*
// LED 9 (Green) of the STM32F4 discovery board same as USB power (VBUS).
#ifndef USE_LED_9
#define USE_LED_9 1
#endif
#define LED_9_GPIO GPIOA
#define LED_9_GPIO_PIN GPIO9
#define LED_9_AFIO_REMAP ((void)0)
#define LED_9_GPIO_ON gpio_set
#define LED_9_GPIO_OFF gpio_clear
*/


/***************************************************************************************************/
/**************************************    UART    *************************************************/
/***************************************************************************************************/

// If you don't need all those uarts comment a uart block out to free pins.
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7

// UART2 is used for the Spectrum rx input also
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9

// UARTS 4 AND 5 CANT BE USED IF SPI3 IS NEEDED
#if !USE_SPI3
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_TX GPIOC
#define UART4_GPIO_TX GPIO10
#define UART4_GPIO_PORT_RX GPIOC
#define UART4_GPIO_RX GPIO11

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#endif

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7


/***************************************************************************************************/
/**************************************    SPI     *************************************************/
/***************************************************************************************************/

#if STM32F4_DISCOVERY_SPI1_FOR_LIS302
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#else
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOB
#define SPI1_GPIO_SCK GPIO3
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5

#endif

/* CANNOT BE USED IF PWM CHANNELS 10 & 11 ARE ACTIVE !!! */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15

/* CANNOT BE USED IF UARTS 4 & 5 ARE ACTIVE !!! */
#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_SCK GPIOC
#define SPI3_GPIO_SCK GPIO10
#define SPI3_GPIO_PORT_MISO GPIOC
#define SPI3_GPIO_MISO GPIO11
#define SPI3_GPIO_PORT_MOSI GPIOC
#define SPI3_GPIO_MOSI GPIO12

#define SPI_SELECT_SLAVE0_PORT GPIOE
#define SPI_SELECT_SLAVE0_PIN GPIO2
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO7
#define SPI_SELECT_SLAVE2_PORT GPIOE
#define SPI_SELECT_SLAVE2_PIN GPIO3

#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12

#define SPI3_GPIO_PORT_NSS GPIOA
#define SPI3_GPIO_NSS GPIO15

/***************************************************************************************************/
/**************************************    I2C     *************************************************/
/***************************************************************************************************/
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

//#define I2C3_GPIO_PORT_SCL GPIOA
//#define I2C3_GPIO_SCL GPIO8
//#define I2C3_GPIO_PORT_SDA GPIOC
//#define I2C3_GPIO_SDA GPIO9


/***************************************************************************************************/
/**************************************    ADC     *************************************************/
/***************************************************************************************************/

#define USE_AD_TIM2 1

// For experimenting only
#define USE_ADC_1   1
#define USE_ADC_2   1
#define USE_ADC_3   1
#define USE_ADC_4   1
#define USE_ADC_5   1
#define USE_ADC_6   1
//#define USE_ADC_7   1

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file */
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif
#if !defined(USE_ADC_4)
#define USE_ADC_4   1
#endif

#define DefaultVoltageOfAdc(adc) (0.00485*adc)

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */

// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL 9
#define ADC_1 AD1_1 // THIS CHANNEL USES THE ADC 1 CONVERTER
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO1
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL 15
#define ADC_2 AD1_2 // THIS CHANNEL USES THE ADC 1 CONVERTER
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO5
#endif

// AUX 3
#if USE_ADC_3
#define AD2_1_CHANNEL 14
#define ADC_3 AD2_1 // THIS CHANNEL USES THE ADC 2 CONVERTER
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO4
#endif

// BAT
#if USE_ADC_4
#define AD2_2_CHANNEL 4
#define ADC_4 AD2_2 // THIS CHANNEL USES THE ADC 2 CONVERTER
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO4
#endif

#if USE_ADC_5
#define AD3_1_CHANNEL 11
#define ADC_5 AD3_1 // THIS CHANNEL USES THE ADC 3 CONVERTER
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO1
#endif

#if USE_ADC_6
#define AD3_2_CHANNEL 12
#define ADC_6 AD3_2 // THIS CHANNEL USES THE ADC 3 CONVERTER
#define ADC_6_GPIO_PORT GPIOC
#define ADC_6_GPIO_PIN GPIO2
#endif


/***************************************************************************************************/
/************************************   ACTUATORS   ************************************************/
/***************************************************************************************************/

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/***************************************************************************************************/
/**********************************   SERVO PWM    *************************************************/
/***************************************************************************************************/

#define PWM_USE_TIM1 1
//#define PWM_USE_TIM3 1
#define PWM_USE_TIM5 1
#define PWM_USE_TIM9 1
#define PWM_USE_TIM12 1

#define USE_PWM0  1
#define USE_PWM1  1
#define USE_PWM2  1
#define USE_PWM3  1
#define USE_PWM4  1
#define USE_PWM5  1
#define USE_PWM6  1
#define USE_PWM7  1
#define USE_PWM8  1
#define USE_PWM9  1
#if USE_SPI2
#define USE_PWM10 0
#define USE_PWM11 0
#else
#define USE_PWM10 1
#define USE_PWM11 1
#endif

#define ACTUATORS_PWM_NB 12

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM1
#define PWM_SERVO_0_GPIO GPIOE
#define PWM_SERVO_0_PIN GPIO9
#define PWM_SERVO_0_AF GPIO_AF1
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO11
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_GPIO GPIOE
#define PWM_SERVO_2_PIN GPIO13
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM1
#define PWM_SERVO_3_GPIO GPIOE
#define PWM_SERVO_3_PIN GPIO14
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM9
#define PWM_SERVO_4_GPIO GPIOE
#define PWM_SERVO_4_PIN GPIO5
#define PWM_SERVO_4_AF GPIO_AF3
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM9
#define PWM_SERVO_5_GPIO GPIOE
#define PWM_SERVO_5_PIN GPIO6
#define PWM_SERVO_5_AF GPIO_AF3
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif


#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO3
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_TIMER TIM5
#define PWM_SERVO_7_GPIO GPIOA
#define PWM_SERVO_7_PIN GPIO2
#define PWM_SERVO_7_AF GPIO_AF2
#define PWM_SERVO_7_OC TIM_OC3
#define PWM_SERVO_7_OC_BIT (1<<2)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_TIMER TIM5
#define PWM_SERVO_8_GPIO GPIOA
#define PWM_SERVO_8_PIN GPIO1
#define PWM_SERVO_8_AF GPIO_AF2
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_TIMER TIM5
#define PWM_SERVO_9_GPIO GPIOA
#define PWM_SERVO_9_PIN GPIO0
#define PWM_SERVO_9_AF GPIO_AF2
#define PWM_SERVO_9_OC TIM_OC1
#define PWM_SERVO_9_OC_BIT (1<<0)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

/* PWM10 AND PWM11 cannot be used if SPI2 is active. */
#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_TIMER TIM12
#define PWM_SERVO_10_GPIO GPIOB
#define PWM_SERVO_10_PIN GPIO14
#define PWM_SERVO_10_AF GPIO_AF9
#define PWM_SERVO_10_OC TIM_OC1
#define PWM_SERVO_10_OC_BIT (1<<0)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_TIMER TIM12
#define PWM_SERVO_11_GPIO GPIOB
#define PWM_SERVO_11_PIN GPIO15
#define PWM_SERVO_11_AF GPIO_AF9
#define PWM_SERVO_11_OC TIM_OC2
#define PWM_SERVO_11_OC_BIT (1<<1)
#else
#define PWM_SERVO_11_OC_BIT 0
#endif

#define PWM_TIM1_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM9_CHAN_MASK (PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT)
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT|PWM_SERVO_9_OC_BIT)
#define PWM_TIM12_CHAN_MASK (PWM_SERVO_10_OC_BIT|PWM_SERVO_11_OC_BIT)

/***************************************************************************************************/
/***********************************   PPM INPUT   *************************************************/
/***************************************************************************************************/

/*
#define USE_PPM_TIM1 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF1
*/

#define USE_PPM_TIM8 1

#define PPM_CHANNEL         TIM_IC4
#define PPM_TIMER_INPUT     TIM_IC_IN_TI4
#define PPM_IRQ             NVIC_TIM8_CC_IRQ
#define PPM_IRQ2            NVIC_TIM8_UP_TIM13_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC4IE
#define PPM_CC_IF           TIM_SR_CC4IF
#define PPM_GPIO_PORT       GPIOC
#define PPM_GPIO_PIN        GPIO9
#define PPM_GPIO_AF         GPIO_AF3

/***************************************************************************************************/
/*********************************  SPECTRUM UART  *************************************************/
/***************************************************************************************************/

/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO8
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1


#endif /* CONFIG_STM32F4_DISCOVERY_H */
