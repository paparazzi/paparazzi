#ifndef CONFIG_KROOZ_SD_H
#define CONFIG_KROOZ_SD_H

#define BOARD_KROOZ

/* KroozSD has a 12MHz external clock and 168MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_PIN GPIO14
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOA
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */

/* PB4, Camera power On/Off */
#define CAM_SW_GPIO GPIOB
#define CAM_SW_GPIO_CLK RCC_GPIOB
#define CAM_SW_GPIO_PIN GPIO4
#define CAM_SW_AFIO_REMAP ((void)0)

/* PC2, Camera shot */
#define CAM_SH_GPIO GPIOC
#define CAM_SH_GPIO_CLK RCC_GPIOC
#define CAM_SH_GPIO_PIN GPIO2
#define CAM_SH_AFIO_REMAP ((void)0)

/* PC15, Camera video */
#define CAM_V_GPIO GPIOC
#define CAM_V_GPIO_CLK RCC_GPIOC
#define CAM_V_GPIO_PIN GPIO15
#define CAM_V_AFIO_REMAP ((void)0)

#define BEEPER_GPIO GPIOC
#define BEEPER_GPIO_CLK RCC_GPIOC
#define BEEPER_GPIO_PIN GPIO14
#define BEEPER_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#define DefaultVoltageOfAdc(adc) (0.008874*adc)

/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO2

/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_SDA GPIO9

/* Onboard ADCs */
#define USE_AD_TIM1 1

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define ADC_CHANNEL_CAM1    ADC_1

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#if USE_ADC_1
#define AD1_1_CHANNEL 12
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO2
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL 10
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO0
#endif

#if USE_ADC_3
#define AD1_3_CHANNEL 11
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO1
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL 13
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO3
#endif


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif


/* PWM */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM4 1
#define PWM_USE_TIM5 1

#define USE_PWM0 1
#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1
#define USE_PWM7 1
#define USE_PWM8 1
#define USE_PWM9 1
//#define USE_PWM10 1

#if USE_PWM10
#define ACTUATORS_PWM_NB 11
#define PWM_USE_TIM2 1
#else
#define ACTUATORS_PWM_NB 10
#endif

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM3
#define PWM_SERVO_0_GPIO GPIOB
#define PWM_SERVO_0_PIN GPIO1
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_OC TIM_OC4
#define PWM_SERVO_0_OC_BIT (1<<3)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO8
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO4
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC1
#define PWM_SERVO_3_OC_BIT (1<<0)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO7
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC2
#define PWM_SERVO_4_OC_BIT (1<<1)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO6
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
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

#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_TIMER TIM2
#define PWM_SERVO_10_GPIO GPIOB
#define PWM_SERVO_10_PIN GPIO3
#define PWM_SERVO_10_AF GPIO_AF1
#define PWM_SERVO_10_OC TIM_OC2
#define PWM_SERVO_10_OC_BIT (1<<1)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

#define PWM_TIM2_CHAN_MASK (PWM_SERVO_10_OC_BIT)
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT)
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT|PWM_SERVO_9_OC_BIT)

/* PPM */

#define USE_PPM_TIM2 1

#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
//#define PPM_IRQ2            NVIC_TIM2_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOB
#define PPM_GPIO_PIN        GPIO3
#define PPM_GPIO_AF         GPIO_AF1

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* CSW pin on CAM connector */
#define SPEKTRUM_BIND_PIN GPIO0
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#endif /* CONFIG_KROOZ_SD_H */
