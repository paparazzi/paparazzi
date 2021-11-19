#ifndef CONFIG_PX4FMU_4_0_H
#define CONFIG_PX4FMU_4_0_H

#define BOARD_PX4FMU_V4

/* differences between board not implemented ATM consider them all the same for time being */
//#define BOARD_PX4FMU_V4_R12
//#define BOARD_PX4FMU_V4_R14
//#define BOARD_PX4FMU_V4_R15

/* PX4FMU_V4 a.k.a. Pixracer board has a 24MHz external clock and 168MHz internal. */

/* STM32F4 STM32F427VIT6 */
#define EXT_CLK 24000000
#define AHB_CLK 168000000

//#define STM32F4 //to debug ADC on F4 does no work
//#define ADC_SAMPLE_TIME ADC_SMPR_SMP_56CYC //to debug ADC on F4 does no work

/* On PCB there is a Multicolor LED */

/* Red */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO11
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* Green */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO1
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* Blue */
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

/* UART */

/* -WiFi ESP Connector, it is just a serial port*/
//TODO: Test
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6

//Not used yet unil find some time to test
/*
#define ESP8266_TX
#define ESP8266_RX
#define ESP8266_PD ( power down )
#define ESP8266_GPIO2
#define ESP8266_RESET
#define ESP8266_GPIO0
#define ESP8266_RTS
#define ESP8266_CTS
*/

/* -TELEM1 Connector */
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_CTS GPIOD
#define UART2_GPIO_CTS GPIO3
#define UART2_GPIO_PORT_RTS GPIOD
#define UART2_GPIO_RTS GPIO4

/* -TELEM2 Connector */
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8
#define UART3_GPIO_PORT_CTS GPIOD
#define UART3_GPIO_CTS GPIO11
#define UART3_GPIO_PORT_RTS GPIOD
#define UART3_GPIO_RTS GPIO12

/* -GPS Connector */
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

/* e.g. for a Spektrum satellite receiver rx only)*/
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7

/* Serial Debugging Info Connector, not used with PPRZ as of now, use JTAG for debugging, so this uart can be put to other use if really needed */
#define UART7_GPIO_AF GPIO_AF8
#define UART7_GPIO_PORT_RX GPIOE
#define UART7_GPIO_RX GPIO7
#define UART7_GPIO_PORT_TX GPIOE
#define UART7_GPIO_TX GPIO8

/* Connector -FRS FrSky */
#define UART8_GPIO_AF GPIO_AF8
#define UART8_GPIO_PORT_RX GPIOE
#define UART8_GPIO_RX GPIO0
#define UART8_GPIO_PORT_TX GPIOE
#define UART8_GPIO_TX GPIO1

/* Soft binding Spektrum */
//TODO: Test and or FIXME:
#define RADIO_CONTROL_POWER_PORT GPIOE
#define RADIO_CONTROL_POWER_PIN GPIO4 //SPEKTRUM POWER
#define RADIO_CONTROL_POWER_ON gpio_clear // yes, inverted
#define RADIO_CONTROL_POWER_OFF gpio_set

//A receiver on powered on 3.3v
#define PERIPHERAL3V3_ENABLE_PORT GPIOC //VDD_3V3_PERIPHERAL_EN
#define PERIPHERAL3V3_ENABLE_PIN GPIO5
#define PERIPHERAL3V3_ENABLE_ON gpio_set
#define PERIPHERAL3V3_ENABLE_OFF gpio_clear

/* Turn SBUS invert */
//TODO: Test
#define RC_POLARITY_GPIO_PORT GPIOC
#define RC_POLARITY_GPIO_PIN GPIO13

#define SPEKTRUM_UART6_RCC RCC_USART6
#define SPEKTRUM_UART6_BANK GPIOC
#define SPEKTRUM_UART6_PIN GPIO7
#define SPEKTRUM_UART6_AF GPIO_AF8
#define SPEKTRUM_UART6_IRQ NVIC_USART6_IRQ
#define SPEKTRUM_UART6_ISR usart6_isr
#define SPEKTRUM_UART6_DEV USART6

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
#define SPI2_GPIO_AF GPIO_AF5
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

//So use best spec'd sensors as second or ref the other or determine it on task type?
/* EXTRA ACC_GYRO_CS on SPI1 ICM 20609-G*/
// TODO: Make it useful
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO15

/* EXTRA_MAG_CS on SPI1 HMC5983*/
//See https://docs.google.com/spreadsheets/d/1gVlKZBvRNTXldoxTXwipGaaHmtF9DNPaftDrzKA47mM/edit#gid=0
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MDL */
// FIXME: Test n fix or emoveal
//#define SPI_SELECT_SLAVE1_PORT GPIOE
//#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MPU_9250_CS on SPI1 */
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO2

/* MS5611 BARO_CS on SPI2 - FRAM*/
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7

/* FRAM on SPI2 */
#define SPI_SELECT_SLAVE4_PORT GPIOD
#define SPI_SELECT_SLAVE4_PIN GPIO10

/* SPI3 NSS on microSD connector */
//FIXME: not tested
//#define SPI_SELECT_SLAVE5_PORT GPIOA
//#define SPI_SELECT_SLAVE5_PIN GPIO4

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
#if USE_AD_TIM2
#undef USE_AD_TIM2 // timer2 is used by the buzzer
#endif
#define USE_AD_TIM3 1

// Internal ADC used for board voltage level measurement
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 4 //ADC12_IN4
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO4
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL 2 // ADC123_IN2 (--> IN2 corresponds to channel 2)
#define ADC_2 AD1_2 // ADC123 means it can be used by ADC 1 and 2 and 3 (the f4 supports 3 adc's), does not matter which. Each ADC can address 4 pins, so in this case we are using ADC 1, on its second pin.
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO2
#endif

#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL 3 // ADC123_IN3
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO3
#endif

//ADC_pin_RSSI_IN
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL 11 // ADC123_IN11
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO1
#endif

/* Allow to define another ADC_CHANNEL_VSUPPLY in the airframe file */
#ifndef ADC_CHANNEL_VSUPPLY
  #define ADC_CHANNEL_VSUPPLY ADC_1 // Per default for the board to sense voltage (V) level via external sensor is via ADC_2
  #define DefaultVoltageOfAdc(adc) (10.5 * (float)adc) // FIXME: More precise value scale internal vdd to 5V
#else
  #if USE_ADC_2
    #define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f) * 10.245f * (float)adc) // About the value scale for a common 3DR clone Power Brick 
  #else
    #define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f) * 6.0f * (float)adc) // About the value scale for a common other sensor 
  #endif
#endif

/* Allow to define another ADC for Current measurement in the airframe file */
#ifndef ADC_CHANNEL_CURRENT
#define ADC_CHANNEL_CURRENT ADC_3 // Per default for the board to sense current (I) via external sensor is via ADC_3
#endif

#if USE_ADC_3
#define DefaultMilliAmpereOfAdc(adc) (9.55 * ((float)adc))// Quite close to the value scale for a common 3DR clone Power Brick 
#else
#define DefaultMilliAmpereOfAdc(adc) (0.1 * ((float)adc)) // FIXME: Value scale internal current use, for whatever it is useful
#endif

#ifndef ADC_CHANNEL_RSSI
#define ADC_CHANNEL_RSSI ADC_4
#endif

/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

//Enable Onboard Barometer per default
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/* Another Magnetometer on board on R14 a HMC5983 on R15 ST ,so not the one in the IMU 9250*/
//FIXME: better default option for use of maybe fuse data
#ifndef USE_MAGNETOMETER_B
#define USE_MAGNETOMETER_B 0
#endif

/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* PWM */
#define PWM_USE_TIM1 1
#define PWM_USE_TIM4 1

//TODO: ifdef USE_SERVO6 for PPM out to e.g. servo extender board ...
// Basically a inter mcu Extra device ;)

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

/* Servo 1 */
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

/* Servo 2 */
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

/* Servo 3 */
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

/* Servo 4 */
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

/* Servo 5 */
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

/* Servo 6 */
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

/*
 * PWM input, also could be used for 
 */
// PWM_INPUT1 on TIM4
#ifdef USE_PWM_INPUT1
#define PWM_INPUT1_GPIO_PORT      GPIOD
#define PWM_INPUT1_GPIO_PIN       GPIO12
#define PWM_INPUT1_GPIO_AF        GPIO_AF2

#define PWM_INPUT1_TIMER          TIM4
#define PWM_INPUT1_CHANNEL_PERIOD TIM_IC1
#define PWM_INPUT1_CHANNEL_DUTY   TIM_IC2
#define PWM_INPUT1_TIMER_INPUT    TIM_IC_IN_TI1
#define PWM_INPUT1_SLAVE_TRIG     TIM_SMCR_TS_TI1FP1
#define PWM_INPUT1_IRQ            NVIC_TIM4_IRQ
#define PWM_INPUT1_CC_IE          (TIM_DIER_CC1IE | TIM_DIER_CC2IE)
#define USE_PWM_INPUT_TIM4        TRUE

#ifdef PWM_INPUT1_TICKS_PER_USEC
#define TIM4_TICKS_PER_USEC PWM_INPUT1_TICKS_PER_USEC
#endif
#define TIM4_PWM_INPUT_IDX        0
#define TIM4_CC_IF_PERIOD         TIM_SR_CC1IF
#define TIM4_CC_IF_DUTY           TIM_SR_CC2IF
#define TIM4_CCR_PERIOD           TIM4_CCR1
#define TIM4_CCR_DUTY             TIM4_CCR2
#endif

// PWM_INPUT2 on TIM?
//FIXME: Find some useful not often used pin to outside world...

/* Ofboard LED Safety LED */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

//#if USE_LED_4
//#ifndef SAFETY_WARNING_LED
#define SAFETY_WARNING_LED 4
//#endif
//#endif
// #define LED_SAFETY_GPIO_PORT LED_4_GPIO
// #define LED_SAFETY_GPIO_PIN LED_4_GPIO_PIN
// #define LED_SAFETY_GPIO_ON LED_4_GPIO_ON
// #define LED_SAFETY_GPIO_OFF LED_4_GPIO_OFF
// #define LED_SAFETY_AFIO_REMAP LED_4_AFIO_REMAP

/* Safety Button */
#define BUTTON_SAFETY_GPIO_PORT      GPIOC
#define BUTTON_SAFETY_GPIO_PIN       GPIO4

/* Buzzer (A.k.a. Alarm) */
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

#endif /* CONFIG_PX4FMU_4_0_H */
