#ifndef CONFIG_LISA_M_COMMON_H
#define CONFIG_LISA_M_COMMON_H

/* SPI slave mapping */

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12

#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO4


/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIO_BANK_USART2_RX
#define UART2_GPIO_RX GPIO_USART2_RX
#define UART2_GPIO_PORT_TX GPIO_BANK_USART2_TX
#define UART2_GPIO_TX GPIO_USART2_TX

#define UART3_GPIO_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_PR_RX
#define UART3_GPIO_RX GPIO_USART3_PR_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_PR_TX
#define UART3_GPIO_TX GPIO_USART3_PR_TX

#define UART5_GPIO_AF 0
#define UART5_GPIO_PORT_RX GPIO_BANK_UART5_RX
#define UART5_GPIO_RX GPIO_UART5_RX
#define UART5_GPIO_PORT_TX GPIO_BANK_UART5_TX
#define UART5_GPIO_TX GPIO_UART5_TX


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* It was common for all Lisa/M up to version 2.1. */
/* #define SPEKTRUM_BIND_PIN GPIO3 */
/* #define SPEKTRUM_BIND_PIN_PORT GPIOC */

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

#define SPEKTRUM_UART5_RCC RCC_UART5
#define SPEKTRUM_UART5_BANK GPIO_BANK_UART5_RX
#define SPEKTRUM_UART5_PIN GPIO_UART5_RX
#define SPEKTRUM_UART5_AF 0
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5


/* PPM
 *
 * Default is PPM config 2, input on GPIO01 (Servo pin 6)
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 2
#endif

#if PPM_CONFIG == 1
/* input on PA01 (UART1_RX) */
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM1_UP_IRQ
#define PPM_IRQ2            NVIC_TIM1_CC_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC3IE
#define PPM_CC_IF           TIM_SR_CC3IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         0

#elif PPM_CONFIG == 2
/* input on PA10 (Servo 6 pin) */
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

// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif
#define USE_AD_TIM1 1

#else
#error "Unknown PPM config"

#endif // PPM_CONFIG

/*
 * ADC
 */

/* Onboard ADCs */
/*
  ADC1 PC3/ADC13
  ADC2 PC0/ADC10
  ADC3 PC1/ADC11
  ADC4 PC5/ADC15
  ADC6 PC2/ADC12
  BATT PC4/ADC14 (ADC5)
*/

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#if USE_ADC_1
#define AD1_1_CHANNEL 13
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO3
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

#if USE_ADC_4
#define AD2_1_CHANNEL 15
#define ADC_4 AD2_1
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO5
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_5
#define USE_ADC_5 1
#endif
#if USE_ADC_5
#define AD1_4_CHANNEL 14
#define ADC_5 AD1_4
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO4
#endif

#if USE_ADC_6
#define AD2_2_CHANNEL 12
#define ADC_6 AD2_2
#define ADC_6_GPIO_PORT GPIOC
#define ADC_6_GPIO_PIN GPIO2
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_5
#endif

/*
 * I2C
 *
 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM5 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1

#if DUAL_PWM_ON
#define DUAL_PWM_USE_TIM5 1

#define USE_DUAL_PWM5 1
#define USE_DUAL_PWM6 1
#else
#define USE_PWM5 1
#define USE_PWM6 1
#endif



#if USE_SERVOS_7AND8
#if USE_I2C1
#error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
#else
#define ACTUATORS_PWM_NB 8
#define USE_PWM7 1
#define USE_PWM8 1
#define PWM_USE_TIM4 1
#endif
#else
#define ACTUATORS_PWM_NB 6
#endif

// Servo numbering on LisaM silkscreen/docs starts with 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO0
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#elif USE_DUAL_PWM5
#define DUAL_PWM_SERVO_5 4

#define DUAL_PWM_SERVO_5_P1 0
#define DUAL_PWM_SERVO_5_P2 1

#define DUAL_PWM_SERVO_5_TIMER TIM5
#define DUAL_PWM_SERVO_5_GPIO GPIOA
#define DUAL_PWM_SERVO_5_PIN GPIO0
#define DUAL_PWM_SERVO_5_AF 0
#define DUAL_PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#elif USE_DUAL_PWM6
#define DUAL_PWM_SERVO_6 5

#define DUAL_PWM_SERVO_6_P1 0
#define DUAL_PWM_SERVO_6_P2 1

#define DUAL_PWM_SERVO_6_TIMER TIM5
#define DUAL_PWM_SERVO_6_GPIO GPIOA
#define DUAL_PWM_SERVO_6_PIN GPIO1
#define DUAL_PWM_SERVO_6_AF 0
#define DUAL_PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif






#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM4
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO6
#define PWM_SERVO_7_AF 0
#define PWM_SERVO_7_OC TIM_OC1
#define PWM_SERVO_7_OC_BIT (1<<0)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM4
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO7
#define PWM_SERVO_8_AF 0
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

/* servos 1-4 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)
/* servos 7-8 on TIM4 if USE_SERVOS_7AND8 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT)


#endif

