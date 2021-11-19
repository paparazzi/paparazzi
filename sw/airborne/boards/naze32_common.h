#ifndef CONFIG_NAZE32_COMMON_H
#define CONFIG_NAZE32_COMMON_H

#define BOARD_NAZE32

/*
 * Onboard LEDs
 */


/* L0 red status led, on PB4 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO4
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP {                            \
    rcc_periph_clock_enable(RCC_AFIO);                  \
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;  \
  }

/* L1 green status led, on PB3 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO3
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
/* UART1 on main port, TX (PA9), RX (PA10) */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

/* UART2 on RC I/O, TX (PA2) pin 5/id "3", RX (PA3) pin 6/id "4" */
#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIO_BANK_USART2_RX
#define UART2_GPIO_RX GPIO_USART2_RX
#define UART2_GPIO_PORT_TX GPIO_BANK_USART2_TX
#define UART2_GPIO_TX GPIO_USART2_TX

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

#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIO_BANK_USART2_RX
#define SPEKTRUM_UART2_PIN GPIO_USART2_RX
#define SPEKTRUM_UART2_AF 0
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2



/* PPM
 *
 * Default: PPM_CONFIG 1: servos 3-6, input on PA0: RC I/O rx_ppm pin 3/id "1"
 *          PPM_CONFIG 2: servos 1-6, input on PA7: RC I/O unused pin 8/id "6"
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 1
#endif

#if PPM_CONFIG == 1
/* input on PA0 (ppm_in), TIM2 CH1 */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO0
#define PPM_GPIO_AF         0

// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif
#define USE_AD_TIM1 1

#elif PPM_CONFIG == 2
/* input on PA7 (unused), TIM3 CH2 */
#define USE_PPM_TIM3 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM3_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO7
#define PPM_GPIO_AF         0

// Move default ADC timer
#if USE_AD_TIM1
#undef USE_AD_TIM1
#endif
#define USE_AD_TIM2 1

#else
#error "Unknown PPM config"

#endif // PPM_CONFIG


/*
 * ADC
 */

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

/* 10k/1k resistor divider, 11 * 3.3V / 4096 */
#define DefaultVoltageOfAdc(adc) (0.008862*adc)


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/*
 * I2C
 */
/* RC port */
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * PWM
 *
 * Servo output numbering on silkscreen starts with '1'
 *
 * silk       default   PPM_CONFIG=2 (PPM_PIN=PA7)
 *
 * '1'  PA8   -         servo1
 * '2'  PA11  -         servo2
 * '3'  PB6   servo3    servo3
 * '4'  PB7   servo4    servo4
 * '5'  PB8   servo5    servo5
 * '6'  PB9   servo6    servo6
 *
 * PPM_in:    PA0       PA7
 *
 */

#if PPM_CONFIG == 2

/* use all servos */
#define ACTUATORS_PWM_NB 6
#define PWM_USE_TIM1 1
#define USE_PWM1 1
#define USE_PWM2 1
#define PWM_SERVO_1 0
#define PWM_SERVO_2 1
#define PWM_SERVO_3 2
#define PWM_SERVO_4 3
#define PWM_SERVO_5 4
#define PWM_SERVO_6 5

#else // PPM_CONFIG == 1

/* servos 1-2 not usable */
#define ACTUATORS_PWM_NB 4
/* servos 1-2 not usable */
#define PWM_USE_TIM1 0
#define USE_PWM1 0
#define USE_PWM2 0
#define PWM_SERVO_3 0
#define PWM_SERVO_4 1
#define PWM_SERVO_5 2
#define PWM_SERVO_6 3

#endif

/* servos 3-6 */
#define PWM_USE_TIM4 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

#if USE_PWM1
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO8
#define PWM_SERVO_1_AF 0
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO11
#define PWM_SERVO_2_AF 0
#define PWM_SERVO_2_OC TIM_OC4
#define PWM_SERVO_2_OC_BIT (1<<3)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO6
#define PWM_SERVO_3_AF 0
#define PWM_SERVO_3_OC TIM_OC1
#define PWM_SERVO_3_OC_BIT (1<<0)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO7
#define PWM_SERVO_4_AF 0
#define PWM_SERVO_4_OC TIM_OC2
#define PWM_SERVO_4_OC_BIT (1<<1)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO8
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC3
#define PWM_SERVO_5_OC_BIT (1<<2)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO9
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servos 1-2 on TIM1 */
#define PWM_TIM1_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT)
/* servos 3-6 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#endif /* CONFIG_NAZE32_COMMON_H */

