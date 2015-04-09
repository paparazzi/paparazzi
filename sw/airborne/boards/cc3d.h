#ifndef CONFIG_CC3D_H
#define CONFIG_CC3D_H

/* CC3D has a 8MHz external clock and 72MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 72000000

/*
 * Onboard LEDs
 */

/* blue status led, on PB3 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO3
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP {                            \
    rcc_periph_clock_enable(RCC_AFIO);                  \
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;  \
  }


/* SPI slave mapping */
/* MPU600 on spi1 */
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4

/* 16Mbit flash on spi2 */
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12

/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
/* UART1 on main port */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

/* UART3 on Flexi Port */
#define UART3_GPIO_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_PR_RX
#define UART3_GPIO_RX GPIO_USART3_PR_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_PR_TX
#define UART3_GPIO_TX GPIO_USART3_PR_TX

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* set to ppm/servo6 input pin on RC connector for now */
#define SPEKTRUM_BIND_PIN GPIO1
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIO_BANK_USART1_RX
#define SPEKTRUM_UART1_PIN GPIO_USART1_RX
#define SPEKTRUM_UART1_AF 0
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#define SPEKTRUM_UART3_RCC RCC_USART3
#define SPEKTRUM_UART3_BANK GPIO_BANK_USART3_PR_RX
#define SPEKTRUM_UART3_PIN GPIO_USART3_PR_RX
#define SPEKTRUM_UART3_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define SPEKTRUM_UART3_IRQ NVIC_USART3_IRQ
#define SPEKTRUM_UART3_ISR usart3_isr
#define SPEKTRUM_UART3_DEV USART3


/* Pint to set Uart1 RX polarity, on PB2, output high inverts, low doesn't */
#define RC_POLARITY_GPIO_PORT GPIOB
#define RC_POLARITY_GPIO_PIN GPIO2


/* PPM
 *
 * Default is PPM config 2, input on GPIOB6: RC input pin 8 (S6_IN)
 */
#ifndef PPM_CONFIG
#define PPM_CONFIG 2
#endif

#if PPM_CONFIG == 1
/* input on PB6 (S1_IN) */
#define USE_PPM_TIM4 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM4_UP_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOB
#define PPM_GPIO_PIN        GPIO6
#define PPM_GPIO_AF         0

#elif PPM_CONFIG == 2
/* input on PA1 (S6_IN) */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         0

#else
#error "Unknown PPM config"
#endif // PPM_CONFIG


/*
 * ADC
 * noting yet, board doesn't seem to have voltage devider for bat measurement
 */


/*
 * I2C
 */
/* flexi port */
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * PWM
 * only servos 1-4 enabled for now
 */
/* servos 1-3 */
#define PWM_USE_TIM4 1
#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
/* servo 4 */
#define PWM_USE_TIM1 1
#define USE_PWM4 1

/* servo 5 */
#define PWM_USE_TIM3 0
#define USE_PWM5 0
/* servo 6 */
#define PWM_USE_TIM2 0
#define USE_PWM6 0

#define ACTUATORS_PWM_NB 4

// Servo output numbering on silkscreen starts with 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM4
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO9
#define PWM_SERVO_1_AF 0
#define PWM_SERVO_1_OC TIM_OC4
#define PWM_SERVO_1_OC_BIT (1<<3)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF 0
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO7
#define PWM_SERVO_3_AF 0
#define PWM_SERVO_3_OC TIM_OC2
#define PWM_SERVO_3_OC_BIT (1<<1)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM1
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO1
#define PWM_SERVO_4_AF 0
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM3
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO4
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM2
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO2
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC3
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servos 1-3 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
/* servo 4 on TIM1 */
#define PWM_TIM1_CHAN_MASK PWM_SERVO_4_OC_BIT
/* servo 5 on TIM3 */
#define PWM_TIM3_CHAN_MASK PWM_SERVO_5_OC_BIT
/* servo 6 on TIM2 */
#define PWM_TIM2_CHAN_MASK PWM_SERVO_6_OC_BIT


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* simply always return 5V for now */
#define DefaultVoltageOfAdc(adc) (50)


#endif /* CONFIG_CC3D_H */
