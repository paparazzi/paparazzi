#ifndef CONFIG_PX4FMU_2_4_H
#define CONFIG_PX4FMU_2_4_H

#define BOARD_PX4FMU_v2

/* Pixhawk board (PX4FMUv2 has a 24MHz external clock and 168MHz internal. */
#define EXT_CLK 24000000
#define AHB_CLK 168000000


/*
 * Onboard LEDs
 */
/* red/amber , on PE12 */
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
/*
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO9
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO10
*/

//OK // conector telem1
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
//#define UART2_CTS PD3
//#define UART2_RTS PD4

//OK-2  //conector telem2
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
//CTS - P11
//RTS - P12

//OK-2   // GPS
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

//OK-2 debug ligado no processador IO
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6

//OK-2 //conector serial5
#define UART7_GPIO_AF GPIO_AF8
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_PORT_TX GPIOE
#define UART7_GPIO_TX GPIO8

//OK-2 //conector serial4
#define UART8_GPIO_AF GPIO_AF8
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO1

/*
 * SPI
 */

/* SPI1 for MPU and accel/gyro if populated */
//OK-2
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

/* SPI2 for FRAM */
//OK-2
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13

/* SPI4 Ext SPI connector */
//OK-2
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
//OK-2
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO13

/* ACCEL_MAG_CS on SPI1 */
//OK-2
#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MPU_CS on SPI1 */
//OK-2
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO2

/* BARO_CS on SPI1 */
//OK-2
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7

/* FRAM_CS on SPI2 */
//OK-2
#define SPI_SELECT_SLAVE4_PORT GPIOD
#define SPI_SELECT_SLAVE4_PIN GPIO10

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
#define USE_AD_TIM4 1

#define BOARD_ADC_CHANNEL_1 11
#define BOARD_ADC_CHANNEL_2 12
#define BOARD_ADC_CHANNEL_3 13
#define BOARD_ADC_CHANNEL_4 10

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */

#if USE_ADC_1
#define AD1_1_CHANNEL 11
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO1
#endif
/*
#if USE_ADC_2
#define AD1_2_CHANNEL 12
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO2
#endif
*/

//OK   current sens
#if USE_ADC_3
#define AD1_3_CHANNEL 13
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO3
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL 10
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO2
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif
#define DefaultVoltageOfAdc(adc) (0.006185*adc)


/*
 * I2C mapping
 */
//OK
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

//OK
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

/*
#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9
*/

/*
 * PPM
 */


#define USE_PPM_TIM1 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         GPIO_AF1

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* GPIO_EXT1 on PX4FMU */

/*
#define SPEKTRUM_BIND_PIN GPIO4
#define SPEKTRUM_BIND_PIN_PORT GPIOC
*/
/*
#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2
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

//Buzzer
/*
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
*/


#define PWM_TIM1_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
//#define PWM_TIM2_CHAN_MASK (PWM_BUZZER_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

#endif /* CONFIG_PX4FMU_2_4_H */
