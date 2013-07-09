#ifndef CONFIG_KROOZ_1_0_H
#define CONFIG_KROOZ_1_0_H

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
#define LED_1_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOA
#define LED_2_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_2_GPIO_PIN GPIO14
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOA
#define LED_3_GPIO_CLK RCC_AHB1ENR_IOPAEN
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */

/* PB4, Camera power On/Off */
#define CAM_SW_GPIO GPIOB
#define CAM_SW_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define CAM_SW_GPIO_PIN GPIO4
#define CAM_SW_AFIO_REMAP ((void)0)

/* PC2, Camera shot */
#define CAM_SH_GPIO GPIOC
#define CAM_SH_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define CAM_SH_GPIO_PIN GPIO2
#define CAM_SH_AFIO_REMAP ((void)0)

/* PC15, Camera video */
#define CAM_V_GPIO GPIOC
#define CAM_V_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define CAM_V_GPIO_PIN GPIO15
#define CAM_V_AFIO_REMAP ((void)0)

#define BEEPER_GPIO             GPIOC
#define BEEPER_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define BEEPER_GPIO_PIN GPIO14
#define BEEPER_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
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


/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/* Onboard ADCs */
#define USE_AD_TIM1 1

#define BOARD_ADC_CHANNEL_1 12
#define BOARD_ADC_CHANNEL_2 10
#define BOARD_ADC_CHANNEL_3 11
#define BOARD_ADC_CHANNEL_4 13
#define BOARD_ADC_CHANNEL_5 14
#define BOARD_ADC_CHANNEL_6 15

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define ADC_CHANNEL_CAM1    ADC_1

#ifndef USE_AD1
#define USE_AD1 1
#endif
/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 ADC1_C1
#ifdef USE_ADC_1
#ifndef ADC_1_GPIO_CLOCK_PORT
#define ADC_1_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_1_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2)
#endif
#define USE_AD1_1 1
#else
#define ADC_1_GPIO_CLOCK_PORT 0
#define ADC_1_INIT() {}
#endif

#define ADC_2 ADC1_C2
#ifdef USE_ADC_2
#ifndef ADC_2_GPIO_CLOCK_PORT
#define ADC_2_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_2_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0)
#endif
#define USE_AD1_2 1
#else
#define ADC_2_GPIO_CLOCK_PORT 0
#define ADC_2_INIT() {}
#endif

#define ADC_3 ADC1_C3
#ifdef USE_ADC_3
#ifndef ADC_3_GPIO_CLOCK_PORT
#define ADC_3_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_3_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1)
#endif
#define USE_AD1_3 1
#else
#define ADC_3_GPIO_CLOCK_PORT 0
#define ADC_3_INIT() {}
#endif

#define ADC_4 ADC1_C4
//#ifdef USE_ADC_4
#ifndef ADC_4_GPIO_CLOCK_PORT
#define ADC_4_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_4_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3)
#endif
#define USE_AD1_4 1
//#else
//#define ADC_4_GPIO_CLOCK_PORT 0
//#define ADC_4_INIT() {}
//#endif

#define ADC_GPIO_CLOCK_PORT (ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT | ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT)

#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) { \
    ADC_1_INIT(); \
    ADC_2_INIT(); \
    ADC_3_INIT(); \
    ADC_4_INIT(); \
  }
#endif // USE_AD1


/* I2C mapping */
#define GPIO_I2C1_SCL GPIO8
#define GPIO_I2C1_SDA GPIO9
#define GPIO_I2C2_SCL GPIO10
#define GPIO_I2C2_SDA GPIO11
#define GPIO_I2C3_SCL GPIO8 //PA8
#define GPIO_I2C3_SDA GPIO9 //PC9

/* Activate onboard baro */
#define BOARD_HAS_BARO 1

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
#define PWM_SERVO_0_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_SERVO_0_GPIO GPIOC
#define PWM_SERVO_0_PIN GPIO6
#define PWM_SERVO_0_AF GPIO_AF1
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO7
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3_IDX 3
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO6
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO7
#define PWM_SERVO_5_AF GPIO_AF1
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO0
#define PWM_SERVO_6_AF GPIO_AF1
#define PWM_SERVO_6_OC TIM_OC1
#define PWM_SERVO_6_OC_BIT (1<<0)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_TIMER TIM5
#define PWM_SERVO_7_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_7_GPIO GPIOA
#define PWM_SERVO_7_PIN GPIO1
#define PWM_SERVO_7_AF GPIO_AF1
#define PWM_SERVO_7_OC TIM_OC2
#define PWM_SERVO_7_OC_BIT (1<<1)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_TIMER TIM5
#define PWM_SERVO_8_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_8_GPIO GPIOA
#define PWM_SERVO_8_PIN GPIO2
#define PWM_SERVO_8_AF GPIO_AF1
#define PWM_SERVO_8_OC TIM_OC3
#define PWM_SERVO_8_OC_BIT (1<<2)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_TIMER TIM5
#define PWM_SERVO_9_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_9_GPIO GPIOA
#define PWM_SERVO_9_PIN GPIO3
#define PWM_SERVO_9_AF GPIO_AF1
#define PWM_SERVO_9_OC TIM_OC4
#define PWM_SERVO_9_OC_BIT (1<<3)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_TIMER TIM2
#define PWM_SERVO_10_RCC_IOP RCC_AHB1ENR_IOPBEN
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
#define SPEKTRUM_BIND_PIN GPIO9
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#endif /* CONFIG_KROOZ_1_0_H */
