#ifndef CONFIG_APOGEE_1_00_H
#define CONFIG_APOGEE_1_00_H

#define BOARD_APOGEE

/* Apogee has a 16MHz external clock and 168MHz internal. */
#define EXT_CLK 16000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PC0 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* orange, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* green, on PC1 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_3_GPIO_PIN GPIO1
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* yellow, on PC3 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

/* AUX1, on PB1, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOB
#define LED_5_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define LED_5_GPIO_PIN GPIO1
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear
#define LED_5_AFIO_REMAP ((void)0)

/* AUX2, on PC5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_6_GPIO_PIN GPIO5
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear
#define LED_6_AFIO_REMAP ((void)0)

/* AUX3, on PC4, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_CLK RCC_AHB1ENR_IOPCEN
#define LED_7_GPIO_PIN GPIO4
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear
#define LED_7_AFIO_REMAP ((void)0)

/* AUX4, on PB15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOB
#define LED_8_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define LED_8_GPIO_PIN GPIO15
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear
#define LED_8_AFIO_REMAP ((void)0)

/* Power Switch, on PB12, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_9
#define USE_LED_9 1
#endif
#define LED_9_GPIO GPIOB
#define LED_9_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define LED_9_GPIO_PIN GPIO12
#define LED_9_GPIO_ON gpio_set
#define LED_9_GPIO_OFF gpio_clear
#define LED_9_AFIO_REMAP ((void)0)

#define POWER_SWITCH_LED 9


/* Pint to set Uart2 RX polarity, on PB13, output high inverts, low doesn't */
#define RC_POLARITY_GPIO_PORT GPIOB
#define RC_POLARITY_GPIO_PIN GPIO13


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#define DefaultVoltageOfAdc(adc) (0.006185*adc)

/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define USE_UART2_TX FALSE

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO9


/* Onboard ADCs */
#define USE_AD_TIM4 1

#define BOARD_ADC_CHANNEL_1 9
#define BOARD_ADC_CHANNEL_2 15
#define BOARD_ADC_CHANNEL_3 14
#define BOARD_ADC_CHANNEL_4 4

#ifndef USE_AD1
#define USE_AD1 1
#endif
/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
// AUX 1
#define ADC_1 ADC1_C1
#ifdef USE_ADC_1
#ifndef ADC_1_GPIO_CLOCK_PORT
#define ADC_1_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPBEN
#define ADC_1_INIT() gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1)
#endif
#define USE_AD1_1 1
#else
#define ADC_1_GPIO_CLOCK_PORT 0
#define ADC_1_INIT() {}
#endif

// AUX 2
#define ADC_2 ADC1_C2
#ifdef USE_ADC_2
#ifndef ADC_2_GPIO_CLOCK_PORT
#define ADC_2_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_2_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5)
#endif
#define USE_AD1_2 1
#else
#define ADC_2_GPIO_CLOCK_PORT 0
#define ADC_2_INIT() {}
#endif

// AUX 3
#define ADC_3 ADC1_C3
#ifdef USE_ADC_3
#ifndef ADC_3_GPIO_CLOCK_PORT
#define ADC_3_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_3_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4)
#endif
#define USE_AD1_3 1
#else
#define ADC_3_GPIO_CLOCK_PORT 0
#define ADC_3_INIT() {}
#endif

// BAT
#define ADC_4 ADC1_C4
#ifndef ADC_4_GPIO_CLOCK_PORT
#define ADC_4_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPAEN
#define ADC_4_INIT() gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4)
#endif
#define USE_AD1_4 1

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define ADC_GPIO_CLOCK_PORT (ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT | ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT)

/* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c */
#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) { \
  ADC_1_INIT(); \
  ADC_2_INIT(); \
  ADC_3_INIT(); \
  ADC_4_INIT(); \
}
#endif // USE_AD1


/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/* Activate onboard baro */
#define BOARD_HAS_BARO 1

/* PWM */
#define PWM_USE_TIM2 1
#define PWM_USE_TIM3 1

#define USE_PWM0 1
#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM3
#define PWM_SERVO_0_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_0_GPIO GPIOB
#define PWM_SERVO_0_PIN GPIO0
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_OC TIM_OC3
#define PWM_SERVO_0_OC_BIT (1<<2)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO2
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO5
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_RCC_IOP RCC_AHB1ENR_IOPBEN
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
#define PWM_SERVO_4_TIMER TIM2
#define PWM_SERVO_4_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO3
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC2
#define PWM_SERVO_4_OC_BIT (1<<1)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM2
#define PWM_SERVO_5_RCC_IOP RCC_AHB1ENR_IOPAEN
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO15
#define PWM_SERVO_5_AF GPIO_AF1
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

// PWM AUX1 (conflict with ADC0)
#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM3
#define PWM_SERVO_6_RCC_IOP RCC_AHB1ENR_IOPBEN
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif


#define PWM_TIM2_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT)
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_6_OC_BIT)

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
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF1

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO8
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#define SPEKTRUM_UART2_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART2_RCC_DEV RCC_APB1ENR_USART2EN
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2

#endif /* CONFIG_APOGEE_1_00_H */
