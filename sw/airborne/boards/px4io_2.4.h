#ifndef CONFIG_PX4IO_2_4_H
#define CONFIG_PX4IO_2_4_H

#define BOARD_PX4IO
//STM32F103c8t6 (medium density!)

/* Pixhawk board (PX4FIOv2 has a 24MHz external clock and 24MHz internal. */
#define EXT_CLK 24000000 //this osc is actually outside of the specs (max 16MHz)
#define AHB_CLK 24000000


/*
 * LEDs
 */
/* blue led, a.k.a. ACT */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

//led Amber a.k.a b/e led
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO15
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

//safety led in the switch, red
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOB
#define LED_3_GPIO_PIN GPIO13
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

//TODO: safety switch is on PB5!

/*
 * UART
*/

// fmu debug / spektrum receiver (only rx)
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9
// intermcu fmu
#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define RADIO_CONTROL_POWER GPIOC
#define RADIO_CONTROL_POWER_PIN GPIO13
#define RADIO_CONTROL_POWER_ON gpio_clear
#define RADIO_CONTROL_POWER_OFF gpio_set

#define SPEKTRUM_TIMER 3

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF 0
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1


/*
 * PPM input
 */
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_UP_IRQ
#define PPM_IRQ2            NVIC_TIM1_CC_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         0

//#define USE_AD_TIM1 1
#ifndef USE_ADC_1
#define USE_ADC_1 0
#endif
#if USE_ADC_1 // VDD servo ADC12_IN4, untested
#define AD1_1_CHANNEL 12
#define ADC_1 AD1_4
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO4
#endif

/*
 * PWM
 *
 */
//sevo outputs on px4io f1:
//chn:           1   2   3   4   5   6   7   8
//pin:           A0  A1  B8  B9  A6  A7  B0  B1
//timer/channel: 2/1 2/2 4/3 4/4 3/1 3/2 3/3 3/4
#define PWM_USE_TIM2 1
//#define PWM_USE_TIM3 1 // spektrum already uses tim3
#define PWM_USE_TIM4 1

//#define ACTUATORS_PWM_NB 4

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
//#define USE_PWM5 1
//#define USE_PWM6 1
//#define USE_PWM7 1
//#define USE_PWM8 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO0
#define PWM_SERVO_1_AF 0
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM2
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO1
#define PWM_SERVO_2_AF 0
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF 0
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF 0
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM3
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO6
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM3
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO7
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM3
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO0
#define PWM_SERVO_7_AF 0
#define PWM_SERVO_7_OC TIM_OC3
#define PWM_SERVO_7_OC_BIT (1<<2)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM3
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO1
#define PWM_SERVO_8_AF 0
#define PWM_SERVO_8_OC TIM_OC4
#define PWM_SERVO_8_OC_BIT (1<<3)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

/* servos 1-2 on TIM2 */
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT)
/* servos 3-4 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-8 on TIM3 */
//#define PWM_TIM3_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT)

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#endif /* CONFIG_PX4IO_2_4_H */
