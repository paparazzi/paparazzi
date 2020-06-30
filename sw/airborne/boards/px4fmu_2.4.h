#ifndef CONFIG_PX4FMU_2_4_H
#define CONFIG_PX4FMU_2_4_H

#define BOARD_PX4FMU_v2

/* Pixhawk board (PX4FMUv2 has a 24MHz external clock and 168MHz internal. */
//STM32F4
#define EXT_CLK 24000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */
/* red, on PE12 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOE
#define LED_1_GPIO_PIN GPIO12
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/*
 * UART
*/

// conector telem1
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
//#define UART2_CTS PD3
//#define UART2_RTS PD4

//conector telem2
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
//CTS - P11
//RTS - P12

//GPS
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

//debug ligado no processador IO
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6

//conector serial5
#define UART7_GPIO_AF GPIO_AF8
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_PORT_TX GPIOE
#define UART7_GPIO_TX GPIO8

//conector serial4
#define UART8_GPIO_AF GPIO_AF8
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO1

/*
 * SPI
 */

/* SPI1 for MPU and accel/gyro if populated */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

/* SPI2 for FRAM */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13

/* SPI4 Ext SPI connector */
#define SPI4_GPIO_AF GPIO_AF5
#define SPI4_GPIO_PORT_MISO GPIOE
#define SPI4_GPIO_MISO GPIO5
#define SPI4_GPIO_PORT_MOSI GPIOE
#define SPI4_GPIO_MOSI GPIO6
#define SPI4_GPIO_PORT_SCK GPIOE
#define SPI4_GPIO_SCK GPIO2

/*
 * SPI slave pin declaration
 */
/* GYRO_CS on SPI1 */
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO13

/* ACCEL_MAG_CS on SPI1 */
#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MPU_CS on SPI1 */
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO2

/* BARO_CS on SPI1 */
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7

/* FRAM_CS on SPI2 */
#define SPI_SELECT_SLAVE4_PORT GPIOD
#define SPI_SELECT_SLAVE4_PIN GPIO10

/* MAG_CS on SPI1 */
#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO1

/* SPI3 NSS on microSD connector */
/*
#define SPI_SELECT_SLAVE3_PORT GPIOA
#define SPI_SELECT_SLAVE3_PIN GPIO4
*/

// SDIO on microSD connector
//#define SDIO_AF GPIO_AF12
// SDIO_D0  pc8
// SDIO_D1  pc9
// SDIO_D2  pc10
// SDIO_D3  pc11
// SDIO_CK  pc12
// SDIO_CMD pd2

/* Onboard ADCs */

#if USE_AD_TIM2
#undef USE_AD_TIM2 // timer2 is used by the buzzer
#endif
#define USE_AD_TIM3 1

// Interal ADC for battery
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 4 //ADC12_IN4
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO4
#endif

// External ADC for battery
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL 2 // ADC123_IN2 (--> IN2 corresponds to channel 2)
#define ADC_2 AD1_2 // ADC123 means it can be used by ADC 1 and 2 and 3 (the f4 supports 3 adc's), does not matter which. Each ADC can address 4 pins, so in this case we are using ADC 1, on its second pin.
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO2
#endif

// external current sens
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL 3 // ADC123_IN3
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO3
#endif
#define MilliAmpereOfAdc(adc)((float)adc) * (3.3f / 4096.0f) * (90.0f / 5.0f)

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_2
#define DefaultVoltageOfAdc(adc) (0.00975*adc)
#else
#define DefaultVoltageOfAdc(adc) (0.00384*adc)
#endif

/* External adc (pressure / air speed / 6.6v) */
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL 15 // ADC12_IN15
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO5
#endif

/* External adc 3.3v */
#if USE_ADC_5
#define AD1_5_CHANNEL 13 // ADC123_IN13
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO3
#endif

/* External adc 3.3v */
#if USE_ADC_6
#define AD1_6_CHANNEL 14 // ADC12_IN14
#define ADC_6 AD1_6
#define ADC_6_GPIO_PORT GPIOC
#define ADC_6_GPIO_PIN GPIO4
#endif

/*
 * I2C mapping
 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

/*
#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9
*/

/* Activate onboard baro by default */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
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
//#define USE_BUZZER 1

// Servo numbering on the PX4 starts with 1
// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array

//servo AUX1
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

//servo AUX2
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

//servo AUX3
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

//servo AUX4
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

//servo AUX5
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

//servo AUX6
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

//Buzzer (alarm)
#if USE_BUZZER
#define PWM_BUZZER
#define PWM_BUZZER_TIMER TIM2
#define PWM_BUZZER_GPIO GPIOA
#define PWM_BUZZER_PIN GPIO15
#define PWM_BUZZER_AF GPIO_AF1
#define PWM_BUZZER_OC TIM_OC1
#define PWM_BUZZER_OC_BIT (1<<0)
#define PWM_TIM2_CHAN_MASK (PWM_BUZZER_OC_BIT)
#else
#define PWM_BUZZER_OC_BIT 0
#endif

#define PWM_TIM1_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

#endif /* CONFIG_PX4FMU_2_4_H */
