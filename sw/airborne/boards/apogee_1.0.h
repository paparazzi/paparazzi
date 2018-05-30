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
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* orange, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* green, on PC1 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO1
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* yellow, on PC3 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

/* AUX1, on PB1, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOB
#define LED_5_GPIO_PIN GPIO1
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear
#define LED_5_AFIO_REMAP ((void)0)

/* AUX2, on PC5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_PIN GPIO5
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear
#define LED_6_AFIO_REMAP ((void)0)

/* AUX3, on PC4, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_PIN GPIO4
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear
#define LED_7_AFIO_REMAP ((void)0)

/* AUX4, on PB15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOB
#define LED_8_GPIO_PIN GPIO15
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear
#define LED_8_AFIO_REMAP ((void)0)

/* Power Switch, on PB12 */
#define POWER_SWITCH_GPIO GPIOB,GPIO12


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

// SLAVE0 on SPI connector
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO9
// SLAVE1 on AUX1
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO1
// SLAVE2 on AUX2
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO5
// SLAVE3 on AUX3
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO4
// SLAVE4 on AUX4
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN GPIO15


/* Onboard ADCs */
#define USE_AD_TIM4 1

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL 9
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO1
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL 15
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO5
#endif

// AUX 3
#if USE_ADC_3
#define AD1_3_CHANNEL 14
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO4
#endif

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
#endif


/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif


/* PWM */
#define PWM_USE_TIM2 1
#define PWM_USE_TIM3 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
// enable PWM connectors by default

#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM3
#define PWM_SERVO_0_GPIO GPIOB
#define PWM_SERVO_0_PIN GPIO0
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_OC TIM_OC3
#define PWM_SERVO_0_OC_BIT (1<<2)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO2
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO5
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
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

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM2
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO3
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC2
#define PWM_SERVO_4_OC_BIT (1<<1)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM2
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
 * PWM input
 */
// PWM_INPUT1 on TIM1
#define PWM_INPUT1_TIMER          TIM1
#ifdef PWM_INPUT1_TICKS_PER_USEC
#define TIM1_TICKS_PER_USEC PWM_INPUT1_TICKS_PER_USEC
#endif
#define PWM_INPUT1_CHANNEL_PERIOD TIM_IC1
#define PWM_INPUT1_CHANNEL_DUTY   TIM_IC2
#define PWM_INPUT1_TIMER_INPUT    TIM_IC_IN_TI1
#define PWM_INPUT1_SLAVE_TRIG     TIM_SMCR_TS_TI1FP1
#define PWM_INPUT1_IRQ            NVIC_TIM1_CC_IRQ
#define PWM_INPUT1_IRQ2           NVIC_TIM1_UP_TIM10_IRQ
#define PWM_INPUT1_CC_IE          (TIM_DIER_CC1IE | TIM_DIER_CC2IE)
#define USE_PWM_INPUT_TIM1        TRUE
#define TIM1_PWM_INPUT_IDX        0
#define TIM1_CC_IF_PERIOD         TIM_SR_CC1IF
#define TIM1_CC_IF_DUTY           TIM_SR_CC2IF
#define TIM1_CCR_PERIOD           TIM1_CCR1
#define TIM1_CCR_DUTY             TIM1_CCR2
// PPM in (aka PA8) is used: not compatible with PPM RC receiver
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO8
#define PWM_INPUT1_GPIO_AF        GPIO_AF1

// PWM_INPUT2 on TIM9
#define PWM_INPUT2_TIMER          TIM9
#ifdef PWM_INPUT2_TICKS_PER_USEC
#define TIM9_TICKS_PER_USEC PWM_INPUT2_TICKS_PER_USEC
#endif
#define PWM_INPUT2_CHANNEL_PERIOD TIM_IC1
#define PWM_INPUT2_CHANNEL_DUTY   TIM_IC2
#define PWM_INPUT2_TIMER_INPUT    TIM_IC_IN_TI1
#define PWM_INPUT2_SLAVE_TRIG     TIM_SMCR_TS_TI1FP1
#define PWM_INPUT2_IRQ            NVIC_TIM1_BRK_TIM9_IRQ
#define PWM_INPUT2_CC_IE          (TIM_DIER_CC3IE | TIM_DIER_CC4IE)
#ifdef USE_PWM_INPUT2
#define USE_PWM_INPUT_TIM9        TRUE
#else
#define USE_PWM_INPUT_TIM9        FALSE
#endif
#define TIM9_PWM_INPUT_IDX        1
#define TIM9_CC_IF_PERIOD         TIM_SR_CC1IF
#define TIM9_CC_IF_DUTY           TIM_SR_CC2IF
#define TIM9_CCR_PERIOD           TIM9_CCR1
#define TIM9_CCR_DUTY             TIM9_CCR2
// Servo 1 (aka PA2) is used: not compatible with PWM1 (and should be disabled)
#if (USE_PWM1 && USE_PWM_INPUT2)
#error "PW1 and PWM_INPUT2 are not compatible"
#endif
#define PWM_INPUT2_GPIO_PORT      GPIOA
#define PWM_INPUT2_GPIO_PIN       GPIO2
#define PWM_INPUT2_GPIO_AF        GPIO_AF3

/*
 * Spektrum
 */

/* The line that is pulled low at power up to initiate the bind process
 * PB15: AUX4
 */
#define SPEKTRUM_BIND_PIN GPIO15
#define SPEKTRUM_BIND_PIN_PORT GPIOB

/* The line used to send the pulse train for the bind process
 * When using UART2 on Apogee, this as to be a different pin than the uart2 rx
 * Default pin for this is PA8: PPM_IN
 */
#ifndef SPEKTRUM_PRIMARY_BIND_CONF_PORT
#define SPEKTRUM_PRIMARY_BIND_CONF_PORT GPIOA
#define SPEKTRUM_PRIMARY_BIND_CONF_PIN GPIO8
#endif

/*
 * IRQ Priorities
 */
#define RTOS_PRIO 2
#define NVIC_TIM_IRQ_PRIO (RTOS_PRIO+1)
#define NVIC_I2C_IRQ_PRIO (RTOS_PRIO+2)
#define NVIC_SPI_IRQ_PRIO (RTOS_PRIO+3)
#define NVIC_UART_IRQ_PRIO (RTOS_PRIO+4)
#define NVIC_USART_IRQ_PRIO (RTOS_PRIO+4)
#define NVIC_ADC_IRQ_PRIO (RTOS_PRIO+5)
#define NVIC_TIM6_DAC_IRQ_PRIO (RTOS_PRIO+6)


#endif /* CONFIG_APOGEE_1_00_H */
