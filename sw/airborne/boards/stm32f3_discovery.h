/*
 * STM32F3 DISCOVERY BOARD DEFINITIONS FOR THE PAPARAZZI AUTOPILOT.
 *
 * DISCOVERY USED FOR THE AUTOPILOT PINS
 * PA0  = Pushbutton          pp: TIM2 CH1
 * PA1  = FREE,               pp: TIM2 CH2
 * PA2  = FREE, pp: UART2 TX, pp: TIM2 CH3, TIM15 CH1
 * PA3  = FREE, pp: UART2 RX, pp: TIM2 CH4, TIM15 CH2
 * PA4  = FREE,               pp: TIM3 CH2
 * PA5  = SPI1 SCK if STM32F3_DISCOVERY_SPI1_FOR_L3GD20, pp: SPI1 SCK
 * PA6  = SPI1 MISO if STM32F3_DISCOVERY_SPI1_FOR_L3GD20, pp: SPI1 MISO
 * PA7  = SPI1 MOSI if STM32F3_DISCOVERY_SPI1_FOR_L3GD20, pp: SPI1 MOSI
 * PA8  = FREE,                                                    pp: TIM1  CH1
 * PA9  = FREE, pp: UART1 TX, pp: I2C2 SCL,                        pp: TIM1  CH2, TIM2 CH3
 * PA10 = FREE, pp: UART1 RX, pp: I2C2 SDA,                        pp: TIM1  CH3, TIM2 CH4
 * PA11 = FREE if usb is not active during runtime //check for f3, pp: TIM1  CH4, TIM4 CH1
 * PA12 = FREE if usb is not active during runtime //check for f3, pp: TIM16 CH1, TIM4 CH2
 * PA13 = SWDIO (FREE?),                                           pp: TIM4  CH3
 * PA14 = SWCLK (FREE?), pp: UART2 TX, pp: I2C1 SDA,               pp: TIM8  CH2
 * PA15 = FREE, pp: UART2 RX, pp: I2C1 SCL,                        pp: TIM8  CH1, TIM2 CH1
 *
 * PB0  = FREE,                                                    pp: TIM3  CH3
 * PB1  = FREE,                                                    pp: TIM3  CH4
 * PB2  = FREE
 * PB3  = SWO (FREE?),                    pp: UART2 TX, pp: SPI1 SCK, SPI3 SCK, pp: TIM2  CH2
 * PB4  = FREE, pp: SPI1 MISO, SPI3 MISO, pp: UART2 RX,                         pp: TIM16 CH1, TIM3  CH1
 * PB5  = FREE, pp: SPI1 MOSI, SPI3 MOSI,                                       pp: TIM17 CH1, TIM3  CH2
 * PB6  = I2C1 SCL if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC, pp: I2C1 SCL
 * PB7  = I2C1 SDA if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC, pp: I2C1 SDA
 * PB8  = FREE, pp: I2C1 SCL,                                                   pp: TIM16 CH1, TIM4  CH3, TIM8 CH2
 * PB9  = FREE, pp: I2C1 SDA,                                                   pp: TIM17 CH1, TIM4  CH4, TIM8 CH3
 * PB10 = FREE, pp: UART3 TX,                                                   pp: TIM2  CH3
 * PB11 = FREE, pp: UART3 RX,                                                   pp: TIM2  CH4
 * PB12 = FREE
 * PB13 = FREE, pp: SPI2 SCL
 * PB14 = FREE, pp: SPI2 MISO,                                                  pp: TIM15 CH1
 * PB15 = FREE, pp: SPI2 MOSI,                                                  pp: TIM15 CH2
 *
 * PC0  = FREE
 * PC1  = FREE
 * PC2  = FREE
 * PC3  = FREE
 * PC4  = FREE, pp: UART1 TX
 * PC5  = FREE, pp: UART1 RX
 * PC6  = FREE,                                                                 pp: TIM3  CH1, TIM8 CH1
 * PC7  = FREE,                                                                 pp: TIM3  CH2, TIM8 CH2
 * PC8  = FREE,                                                                 pp: TIM3  CH3, TIM8 CH3
 * PC9  = FREE,                                                                 pp: TIM3  CH4, TIM8 CH4
 * PC10 = FREE, pp: UART3 TX, UART4 TX, pp: SPI3 SCK
 * PC11 = FREE, pp: UART3 RX, UART4 RX, pp: SPI3 MISO
 * PC12 = FREE, pp: UART5 TX, pp: SPI3 MOSI
 * PC13 = FREE
 * PC14 = OSC32_IN
 * PC15 = OSC32_OUT
 *
 * PD0  = FREE
 * PD1  = FREE,                                                                 pp: TIM8  CH4
 * PD2  = FREE, pp: UART5 RX
 * PD3  = FREE,                                                                 pp: TIM2  CH1
 * PD4  = FREE,                                                                 pp: TIM2  CH2
 * PD5  = FREE, pp: UART2 TX
 * PD6  = FREE, pp: UART2 RX,                                                   pp: TIM2  CH4
 * PD7  = FREE,                                                                 pp: TIM2  CH3
 * PD8  = FREE, pp: UART3 TX
 * PD9  = FREE, pp: UART3 RX
 * PD10 = FREE
 * PD11 = FREE
 * PD12 = FREE,                                                                 pp: TIM4  CH1
 * PD13 = FREE,                                                                 pp: TIM4  CH2
 * PD14 = FREE,                                                                 pp: TIM4  CH3
 * PD15 = FREE,                                                                 pp: TIM4  CH4
 *
 * PE0  = INT1 if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
 * PE1  = DRDY/INT2 if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
 * PE2  = DRDY if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC
 * PE3  = CS_I2C/SPI if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
 * PE4  = INT1 if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC
 * PE5  = INT2 if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC
 * PE6  = FREE
 * PE7  = FREE
 * PE8  = LD4/BLUE, pp: LED_4
 * PE9  = LD3/RED, pp: LED_3,                                                   pp: TIM1  CH1
 * PE10 = LD5/ORANGE, pp: LED_5
 * PE11 = LD7/GREEN, pp: LED_7,                                                 pp: TIM1  CH2
 * PE12 = LD9/BLUE, pp: LED_9
 * PE13 = LD10/RED, pp: LED_10,                                                 pp: TIM1  CH3
 * PE14 = LD8/ORANGE, pp: LED_8,                                                pp: TIM1  CH4
 * PE15 = LD6/GREEN, pp: LED_6, pp: UART3 RX
 *
 * PF0  = OSC_IN
 * PF1  = OSC_OUT
 * PF2  = FREE
 * PF4  = FREE
 * PF6  = FREE, pp: I2C2 SCL,                                                   pp: TIM4  CH4
 * PF9  = FREE, pp: SPI2 SCK,                                                   pp: TIM15 CH1
 * PF10 = FREE, pp: SPI2 SCK,                                                   pp: TIM15 CH2

 */
#ifndef CONFIG_STM32F3_DISCOVERY_H
#define CONFIG_STM32F3_DISCOVERY_H

#define BOARD_STM32F3_DISCOVERY

/* STM32F3_DISCOVERY has a 8MHz external clock and 168MHz internal. */
#define INT_CLK 8000000   //Fix!
#define AHB_CLK 64000000 //Fix!
//#define I2C1_CLOCK_SPEED 100000 //now is in the quadrobot xml file

/*
 * Onboard LEDs
 */

/* red, on PE9 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOE
#define LED_3_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_3_GPIO_PIN GPIO9
#define LED_3_AFIO_REMAP ((void)0)
#define LED_3_GPIO_ON gpio_set
#define LED_3_GPIO_OFF gpio_clear

/* blue, on PE8 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOE
#define LED_4_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_4_GPIO_PIN GPIO8
#define LED_4_AFIO_REMAP ((void)0)
#define LED_4_GPIO_ON gpio_set
#define LED_4_GPIO_OFF gpio_clear

/* orange, on PE10 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOE
#define LED_5_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_5_GPIO_PIN GPIO10
#define LED_5_AFIO_REMAP ((void)0)
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* green, on PE15 */
#ifndef USE_LED_6
#define USE_LED_6 1
#endif
#define LED_6_GPIO GPIOE
#define LED_6_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_6_GPIO_PIN GPIO15
#define LED_6_AFIO_REMAP ((void)0)
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* green, on PE11 */
#ifndef USE_LED_7
#define USE_LED_7 1
#endif
#define LED_7_GPIO GPIOE
#define LED_7_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_7_GPIO_PIN GPIO11
#define LED_7_AFIO_REMAP ((void)0)
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* orange, on PE14 */
#ifndef USE_LED_8
#define USE_LED_8 1
#endif
#define LED_8_GPIO GPIOE
#define LED_8_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_8_GPIO_PIN GPIO14
#define LED_8_AFIO_REMAP ((void)0)
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear

/* blue, on PE12 */
#ifndef USE_LED_9
#define USE_LED_9 1
#endif
#define LED_9_GPIO GPIOE
#define LED_9_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_9_GPIO_PIN GPIO12
#define LED_9_AFIO_REMAP ((void)0)
#define LED_9_GPIO_ON gpio_set
#define LED_9_GPIO_OFF gpio_clear

/* red, on PE13 */
#ifndef USE_LED_10
#define USE_LED_10 1
#endif
#define LED_10_GPIO GPIOE
#define LED_10_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_10_GPIO_PIN GPIO13
#define LED_10_AFIO_REMAP ((void)0)
#define LED_10_GPIO_ON gpio_set
#define LED_10_GPIO_OFF gpio_clear


//Check from here

/* UART */ //Perfect FREE!
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOC
#define UART1_GPIO_RX GPIO5
#define UART1_GPIO_PORT_TX GPIOC
#define UART1_GPIO_TX GPIO4

//check for (TIM2 CH4 not critical if TIM2 in PORTD, TIM15 CH1,2 not critical if TIM15 in portf)
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

// All fine!
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_PORT_TX GPIOD
#define UART3_GPIO_TX GPIO8

// check for SPI3
#if !USE_SPI3_PORTC //better check against the port version
#define UART4_GPIO_AF GPIO_AF5
#define UART4_GPIO_PORT_RX GPIOC
#define UART4_GPIO_RX GPIO11
#define UART4_GPIO_PORT_TX GPIOC
#define UART4_GPIO_TX GPIO10
#endif

// check for SPI3
#if !USE_SPI3_PORTC //better check against the port version
#define UART5_GPIO_AF GPIO_AF5
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12
#endif

/* SPI */
#if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#else // check UART2, SPI3, TIM2 CH2, TIM16 CH1, TIM3 CH1,2, TIM17 CH1
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5
#define SPI1_GPIO_PORT_SCK GPIOB
#define SPI1_GPIO_SCK GPIO3
#endif

//check (TIM15 CH1,2 fine if TIM15 in portf)
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13

#if USE_SPI3_PORTB
#if !STM32F3_DISCOVERY_SPI1_FOR_L3GD20
#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_MISO GPIOB
#define SPI3_GPIO_MISO GPIO4
#define SPI3_GPIO_PORT_MOSI GPIOB
#define SPI3_GPIO_MOSI GPIO5
#define SPI3_GPIO_PORT_SCK GPIOB
#define SPI3_GPIO_SCK GPIO3
#endif
#elif USE_SPI3_PORTC
#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_MISO GPIOC
#define SPI3_GPIO_MISO GPIO11
#define SPI3_GPIO_PORT_MOSI GPIOC
#define SPI3_GPIO_MOSI GPIO12
#define SPI3_GPIO_PORT_SCK GPIOC
#define SPI3_GPIO_SCK GPIO10
#endif

#if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
// STM32F3DISCOVERY L3GD20
#define SPI_SELECT_SLAVE0_PORT GPIOE
#define SPI_SELECT_SLAVE0_PIN GPIO3
#endif

/* I2C mapping */
#if STM32F3_DISCOVERY_I2C1_FOR_LSM303DLHC
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7
#elif USE_I2C1_PORTB || !USE_I2C1_PORTA
#define I2C1_GPIO_PORT GPIOB //check TIM16 CH1, TIM17 CH1, TIM4 CH3,4, TIM8 CH2,3, TIM2 CH3,4
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9
#elif USE_I2C1_PORTA
#define I2C1_GPIO_PORT GPIOA //check UART2, TIM8 CH1,2
#define I2C1_GPIO_SCL GPIO15
#define I2C1_GPIO_SDA GPIO14
#endif

#if USE_I2C2 //check UART1, TIM1 CH2,3, TIM2 CH3,4
#define I2C2_GPIO_PORT GPIOA
#define I2C2_GPIO_SCL GPIO9
#define I2C2_GPIO_SDA GPIO10
#endif

/* ADC Pending!! */

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/***************************************************************************************************/
/**********************************   SERVO PWM    *************************************************/
/***************************************************************************************************/

#if !USE_I2C2
//#define PWM_USE_TIM1 0 //Put in portA check I2C2, (TIM2 CH3,4 not critical if we put TIM2 in PORTD pins 0-3), TIM4 CH1
#endif
#define PWM_USE_TIM2 1 //Put in portD check (UART2 but not critical if in PortA and tim15 in portf)
#define PWM_USE_TIM4 1 //Put in portD check perfect!!
//#define PWM_USE_TIM15 1 //Put in portf and all is fine!
#define PWM_USE_TIM3 1 //Put in portc and if TIM8 of PPM is using ch4 in portd all is fine

#if !USE_I2C2
//TIM1
#define USE_PWM0  1
#define USE_PWM1  1
#define USE_PWM2  1
#define USE_PWM3  1
#endif
//TIM2
#define USE_PWM4  1
#define USE_PWM5  1
#define USE_PWM6  1
#define USE_PWM7  1
//TIM4
#define USE_PWM8  1
#define USE_PWM9  1
#define USE_PWM10 1
#define USE_PWM11 1

#define ACTUATORS_PWM_NB 12

/* #if !USE_I2C2 */
/* // PWM_SERVO_x is the index of the servo in the actuators_pwm_values array */
/* #if USE_PWM0 */
/* #define PWM_SERVO_0 0 */
/* #define PWM_SERVO_0_TIMER TIM1 */
/* #define PWM_SERVO_0_RCC_IOP RCC_AHBENR_IOPAEN */
/* #define PWM_SERVO_0_GPIO GPIOA */
/* #define PWM_SERVO_0_PIN GPIO8 */
/* #define PWM_SERVO_0_AF GPIO_AF6 */
/* #define PWM_SERVO_0_OC TIM_OC1 */
/* #define PWM_SERVO_0_OC_BIT (1<<0) */
/* #else */
/* #define PWM_SERVO_0_OC_BIT 0 */
/* #endif */

/* #if USE_PWM1 */
/* #define PWM_SERVO_1 1 */
/* #define PWM_SERVO_1_TIMER TIM1 */
/* #define PWM_SERVO_1_RCC_IOP RCC_AHBENR_IOPAEN */
/* #define PWM_SERVO_1_GPIO GPIOA */
/* #define PWM_SERVO_1_PIN GPIO9 */
/* #define PWM_SERVO_1_AF GPIO_AF6 */
/* #define PWM_SERVO_1_OC TIM_OC2 */
/* #define PWM_SERVO_1_OC_BIT (1<<1) */
/* #else */
/* #define PWM_SERVO_1_OC_BIT 0 */
/* #endif */

/* #if USE_PWM2 */
/* #define PWM_SERVO_2 2 */
/* #define PWM_SERVO_2_TIMER TIM1 */
/* #define PWM_SERVO_2_RCC_IOP RCC_AHBENR_IOPAEN */
/* #define PWM_SERVO_2_GPIO GPIOA */
/* #define PWM_SERVO_2_PIN GPIO10 */
/* #define PWM_SERVO_2_AF GPIO_AF6 */
/* #define PWM_SERVO_2_OC TIM_OC3 */
/* #define PWM_SERVO_2_OC_BIT (1<<2) */
/* #else */
/* #define PWM_SERVO_2_OC_BIT 0 */
/* #endif */

/* #if USE_PWM3 */
/* #define PWM_SERVO_3 3 */
/* #define PWM_SERVO_3_TIMER TIM1 */
/* #define PWM_SERVO_3_RCC_IOP RCC_AHBENR_IOPAEN */
/* #define PWM_SERVO_3_GPIO GPIOA */
/* #define PWM_SERVO_3_PIN GPIO11 */
/* #define PWM_SERVO_3_AF GPIO_AF6 */
/* #define PWM_SERVO_3_OC TIM_OC4 */
/* #define PWM_SERVO_3_OC_BIT (1<<3) */
/* #else */
/* #define PWM_SERVO_3_OC_BIT 0 */
/* #endif */
/* #endif */

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM3
#define PWM_SERVO_0_RCC_IOP RCC_AHBENR_IOPAEN
#define PWM_SERVO_0_GPIO GPIOC
#define PWM_SERVO_0_PIN GPIO6
#define PWM_SERVO_0_AF GPIO_AF6
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_RCC_IOP RCC_AHBENR_IOPAEN
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO7
#define PWM_SERVO_1_AF GPIO_AF6
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_RCC_IOP RCC_AHBENR_IOPAEN
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF GPIO_AF6
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_RCC_IOP RCC_AHBENR_IOPAEN
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF6
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif


#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM2
#define PWM_SERVO_4_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_4_GPIO GPIOD
#define PWM_SERVO_4_PIN GPIO3
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM2
#define PWM_SERVO_5_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_5_GPIO GPIOD
#define PWM_SERVO_5_PIN GPIO4
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif


#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM2
#define PWM_SERVO_6_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_6_GPIO GPIOD
#define PWM_SERVO_6_PIN GPIO6
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_TIMER TIM2
#define PWM_SERVO_7_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_7_GPIO GPIOD
#define PWM_SERVO_7_PIN GPIO7
#define PWM_SERVO_7_AF GPIO_AF2
#define PWM_SERVO_7_OC TIM_OC3
#define PWM_SERVO_7_OC_BIT (1<<2)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_TIMER TIM4
#define PWM_SERVO_8_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_8_GPIO GPIOD
#define PWM_SERVO_8_PIN GPIO12
#define PWM_SERVO_8_AF GPIO_AF2
#define PWM_SERVO_8_OC TIM_OC1
#define PWM_SERVO_8_OC_BIT (1<<0)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_TIMER TIM4
#define PWM_SERVO_9_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_9_GPIO GPIOD
#define PWM_SERVO_9_PIN GPIO13
#define PWM_SERVO_9_AF GPIO_AF2
#define PWM_SERVO_9_OC TIM_OC2
#define PWM_SERVO_9_OC_BIT (1<<1)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_TIMER TIM4
#define PWM_SERVO_10_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_10_GPIO GPIOD
#define PWM_SERVO_10_PIN GPIO14
#define PWM_SERVO_10_AF GPIO_AF2
#define PWM_SERVO_10_OC TIM_OC3
#define PWM_SERVO_10_OC_BIT (1<<2)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_TIMER TIM4
#define PWM_SERVO_11_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_11_GPIO GPIOD
#define PWM_SERVO_11_PIN GPIO15
#define PWM_SERVO_11_AF GPIO_AF2
#define PWM_SERVO_11_OC TIM_OC4
#define PWM_SERVO_11_OC_BIT (1<<3)
#else
#define PWM_SERVO_11_OC_BIT 0
#endif

//#define PWM_TIM1_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_8_OC_BIT|PWM_SERVO_9_OC_BIT|PWM_SERVO_10_OC_BIT|PWM_SERVO_11_OC_BIT)

/*
 * PPM
 */
#define USE_PPM_TIM8 1

#define PPM_CHANNEL         TIM_IC4
#define PPM_TIMER_INPUT     TIM_IC_IN_TI4
#define PPM_IRQ             NVIC_TIM8_CC_IRQ
#define PPM_IRQ2            NVIC_TIM8_UP_IRQ //check why there is a TIM10
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC4IE
#define PPM_CC_IF           TIM_SR_CC4IF
#define PPM_GPIO_PORT       GPIOD
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         GPIO_AF4


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO2
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART3_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART3_RCC_DEV RCC_APB1ENR_USART3EN
#define SPEKTRUM_UART3_BANK GPIOD
#define SPEKTRUM_UART3_PIN GPIO9
#define SPEKTRUM_UART3_AF GPIO_AF7
#define SPEKTRUM_UART3_IRQ NVIC_USART3_EXTI28_IRQ
#define SPEKTRUM_UART3_ISR usart3_exti28_isr
#define SPEKTRUM_UART3_DEV USART3


/* /\* Onboard ADCs *\/ */
/* #define USE_AD_TIM4 1 */

/* #define BOARD_ADC_CHANNEL_1 9 */
/* #define BOARD_ADC_CHANNEL_2 15 */
/* #define BOARD_ADC_CHANNEL_3 14 */
/* #define BOARD_ADC_CHANNEL_4 4 */

/* #ifndef USE_AD1 */
/* #define USE_AD1 1 */
/* #endif */
/* /\* provide defines that can be used to access the ADC_x in the code or airframe file */
/*  * these directly map to the index number of the 4 adc channels defined above */
/*  * 4th (index 3) is used for bat monitoring by default */
/*  *\/ */
/* // AUX 1 */
/* #define ADC_1 ADC1_C1 */
/* #ifdef USE_ADC_1 */
/* #ifndef ADC_1_GPIO_CLOCK_PORT */
/* #define ADC_1_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPBEN */
/* #define ADC_1_INIT() gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1) */
/* #endif */
/* #define USE_AD1_1 1 */
/* #else */
/* #define ADC_1_GPIO_CLOCK_PORT 0 */
/* #define ADC_1_INIT() {} */
/* #endif */

/* // AUX 2 */
/* #define ADC_2 ADC1_C2 */
/* #ifdef USE_ADC_2 */
/* #ifndef ADC_2_GPIO_CLOCK_PORT */
/* #define ADC_2_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN */
/* #define ADC_2_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5) */
/* #endif */
/* #define USE_AD1_2 1 */
/* #else */
/* #define ADC_2_GPIO_CLOCK_PORT 0 */
/* #define ADC_2_INIT() {} */
/* #endif */

/* // AUX 3 */
/* #define ADC_3 ADC1_C3 */
/* #ifdef USE_ADC_3 */
/* #ifndef ADC_3_GPIO_CLOCK_PORT */
/* #define ADC_3_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN */
/* #define ADC_3_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4) */
/* #endif */
/* #define USE_AD1_3 1 */
/* #else */
/* #define ADC_3_GPIO_CLOCK_PORT 0 */
/* #define ADC_3_INIT() {} */
/* #endif */

/* // BAT */
/* #define ADC_4 ADC1_C4 */
/* #ifndef ADC_4_GPIO_CLOCK_PORT */
/* #define ADC_4_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPAEN */
/* #define ADC_4_INIT() gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4) */
/* #endif */
/* #define USE_AD1_4 1 */

/* /\* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*\/ */
/* #ifndef ADC_CHANNEL_VSUPPLY */
/* #define ADC_CHANNEL_VSUPPLY ADC_4 */
/* #endif */

/* #define ADC_GPIO_CLOCK_PORT (ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT | ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT) */

/* /\* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c *\/ */
/* #ifdef USE_AD1 */
/* #define ADC1_GPIO_INIT(gpio) {                  \ */
/*     ADC_1_INIT();                               \ */
/*     ADC_2_INIT();                               \ */
/*     ADC_3_INIT();                               \ */
/*     ADC_4_INIT();                               \ */
/*   } */
/* #endif // USE_AD1 */

/* #define DefaultVoltageOfAdc(adc) (0.00485*adc) */




#endif /* CONFIG_STM32F3_DISCOVERY_H */
