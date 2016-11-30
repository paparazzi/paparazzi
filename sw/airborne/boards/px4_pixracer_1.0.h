#ifndef CONFIG_PX4PR_1_0_H
#define CONFIG_PX4PR_1_0_H

#define BOARD_PX4FMU_V4

/* PX4FMU_V4 (Pixracer) board has a 24MHz external clock and 168MHz internal. */

/* STM32F4 STM32F427VIT6 */
#define EXT_CLK 24000000 //OK
#define AHB_CLK 168000000 //OK

/* On PCB Multicolor LED */
/* Red */
//OK
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO11
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* Green */
//OK
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO1
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* Blue */
//OK
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOB
#define LED_3_GPIO_PIN GPIO3
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* On PCB other LEDs */
/* The Green Pixracer Power LED in not controllable, not on a MCU pin */

/* UART SCHTUFFFF*/

/* -ESP Connector, it is just a serial port*/
//OK
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6
//#define UART1_GPIO_PORT_CTS GPIOE
//#define UART1_GPIO_CTS GPIO10
//#define UART1_GPIO_PORT_CTS GPIOE
//#define UART1_GPIO_CTS GPIO8

/* -TELEM1 Connector */
//OK
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
//#define UART1_GPIO_PORT_CTS GPIOE FIXME
//#define UART1_GPIO_CTS GPIO3 FIXME
//#define UART1_GPIO_PORT_CTS GPIOE FIXME
//#define UART1_GPIO_CTS GPIO4 FIXME

/* -TELEM2 Connector */
//OK
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
//#define UART1_GPIO_PORT_CTS GPIOE FIXME
//#define UART1_GPIO_CTS GPIO11 FIXME
//#define UART1_GPIO_PORT_CTS GPIOE FIXME
//#define UART1_GPIO_CTS GPIO12 FIXME

/* -GPS Connector */
//OK
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

/* Serial Debugging Connector, not used with PPRZ, use JTAG, can be put to other good use */
//OK
#define UART7_GPIO_AF GPIO_AF8
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_PORT_TX GPIOE
#define UART7_GPIO_TX GPIO8

/* Connector -FRS FrSky */
//OK
#define UART8_GPIO_AF GPIO_AF8
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO1

/* Spektrum Receivers Devices sometimes with TX out */
/* The line that is pulled low at power up to initiate the bind process */
/* Soft binding possible if module is used */
#define SPEKTRUM_BIND_PIN GPIO0
#define SPEKTRUM_BIND_PIN_PORT GPIOB

//FIXME Check
#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOB
#define SPEKTRUM_UART1_PIN GPIO0
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1
//FIXME Check
/*
#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIOC
#define SPEKTRUM_UART2_PIN GPIO7
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2
*/

/* SPI */

/* SPI1 for MPU and extra accel/gyro/mag */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

/* SPI2 for FRAM, connects to BARO */
#define SPI2_GPIO_AF GPIO_AF10
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO10

/*
 * SPI slave pin declaration
 */

/* note :
Active-Low Push-Pull Data-Ready Output. DRDY goes low when a new conversion result is available in the data register.
When a read-operation of an RTD resistance data register occurs, DRDY returns high.
*/
/*
 * SPI slave pin declaration
 */

//So use best speced sensors as second or ref the other or determine it on task type?
/* EXTRA ACC_GYRO_CS on SPI1 ICM 20609-G*/
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO15

/* EXTRA_MAG_CS on SPI1 HMC5983*/
//See https://docs.google.com/spreadsheets/d/1gVlKZBvRNTXldoxTXwipGaaHmtF9DNPaftDrzKA47mM/edit#gid=0
//OK
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MDL
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO15 */

/* MPU_9250_CS on SPI1 */
//OK
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO2

/* 5611 BARO_CS on SPI1 - FRAM*/
//OK
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7

/* FRAM*/
//ok
#define SPI_SELECT_SLAVE4_PORT GPIOD
#define SPI_SELECT_SLAVE4_PIN GPIO10

/* SPI3 NSS on microSD connector */
/*
#define SPI_SELECT_SLAVE4_PORT GPIOA
#define SPI_SELECT_SLAVE4_PIN GPIO4
*/

/* SDIO to microSD card connector */
#define SDIO_AF GPIO_AF12
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

/* Onboard ADCs */
#define USE_AD_TIM5 1

/* for external sensor use USE_ADC_2 */
#ifdef USE_ADC_2
#define USE_ADC_2 1
#define DefaultVoltageOfAdc(adc) (0.00975f*adc)
#endif

/* External voltage sensor not by default? */
#if USE_ADC_2
#define AD1_2_CHANNEL 12//FIXME
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO2
#endif

/* Allows overruling in airframe for those special occasions */
//#ifndef DefaultVoltageOfAdc
//#define DefaultVoltageOfAdc(adc) (0.00975f*adc)//FIXME: for correct value
//#endif

#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif

/* Current sensor by default */
#if USE_ADC_3
#define AD1_3_CHANNEL 3
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO3
#endif
#define MilliAmpereOfAdc(adc)((float)adc) * (3.3f / 4096.0f) * (90.0f / 5.0f)

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif

#if USE_ADC_4
#define AD1_4_CHANNEL 4
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#define DefaultVoltageOfAdc(adc) (0.001741071f*adc) //If on 5V?? FIXME TEST
#endif

//#define DefaultVoltageOfAdc(adc) (0.00975f*adc) // value comes from px4 code sensors.cpp _parameters.battery_voltage_scaling = 0.0082f; Manual calib on iris = 0.0096...

/*
#if USE_ADC_5
#define AD1_5_CHANNEL 11
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOA
#define ADC_5_GPIO_PIN GPIO4
#endif

#ifndef ADC_CHANNEL_RC_RSSI
#define ADC_CHANNEL_RC_RSSI	AD1_5_CHANNEL
#endif*/


/* I2C mapping */
/* There is no second I2C connector but boards to split to multiple exit, reliability, who knows...*/
//OK
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

/* Maybe Don't Activate onboard baro by default */
//TODO
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/* Another Magnetometer on board a HMC5983 not the one in the IMU 9250*/
//TODO
#ifndef USE_MAGNETOMETER_B
#define USE_MAGNETOMETER_B 0
#endif

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* PWM */
#define PWM_USE_TIM1 1
#define PWM_USE_TIM4 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

/* -ESC Servo 1 */
//OK
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO14
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC4
#define PWM_SERVO_1_OC_BIT (1<<3)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

/* -ESC Servo 2 */
//OK
#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_GPIO GPIOE
#define PWM_SERVO_2_PIN GPIO13
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

/* -ESC Servo 3 */
//OK
#if USE_PWM3
#define PWM_SERVO_3 2  //#define PWM_SERVO_3_IDX 2
#define PWM_SERVO_3_TIMER TIM1
#define PWM_SERVO_3_GPIO GPIOE
#define PWM_SERVO_3_PIN GPIO11
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC2
#define PWM_SERVO_3_OC_BIT (1<<1)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

/* -ESC Servo 4 */
//OK
#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM1
#define PWM_SERVO_4_GPIO GPIOE
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

/* -ESC Servo 5 */
//OK
#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_GPIO GPIOD
#define PWM_SERVO_5_PIN GPIO13
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

/* -ESC Servo 6 */
//OK
#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_GPIO GPIOD
#define PWM_SERVO_6_PIN GPIO14
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC3
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#define PWM_TIM1_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

/* Buzzer (A.k.a. Alarm) */

/* Activate buzzer by default */
#ifndef USE_BUZZER
#define USE_BUZZER 1
#endif

#if USE_BUZZER
#define PWM_BUZZER
#define PWM_BUZZER_TIMER TIM2
#define PWM_BUZZER_GPIO GPIOA
#define PWM_BUZZER_PIN GPIO15
#define PWM_BUZZER_AF GPIO_AF1
#define PWM_BUZZER_OC TIM_OC1
#define PWM_BUZZER_OC_BIT (1<<0)
#else
#define PWM_BUZZER_OC_BIT 0
#endif

#define PWM_TIM2_CHAN_MASK (PWM_BUZZER_OC_BIT)

#endif /* CONFIG_PX4PR_1_0_H */
