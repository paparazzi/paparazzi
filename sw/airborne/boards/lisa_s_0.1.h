#ifndef CONFIG_LISA_S_0_1_H
#define CONFIG_LISA_S_0_1_H

#define BOARD_LISA_S

/* Lisa/M has a 12MHz external clock and 72MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 72000000

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_1_GPIO_PIN GPIO10
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_2_GPIO_PIN GPIO11
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOD
#define LED_3_GPIO_CLK RCC_APB2ENR_IOPDEN
#define LED_3_GPIO_PIN GPIO2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */

/* PB1, DRDY on EXT SPI connector*/
#define LED_BODY_GPIO GPIOB
#define LED_BODY_GPIO_CLK RCC_APB2ENR_IOPBEN
#define LED_BODY_GPIO_PIN GPIO1
#define LED_BODY_GPIO_ON gpio_set
#define LED_BODY_GPIO_OFF gpio_clear
#define LED_BODY_AFIO_REMAP ((void)0)

/* PC12, on GPIO connector*/
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_CLK RCC_APB2ENR_IOPCEN
#define LED_12_GPIO_PIN GPIO12
#define LED_12_GPIO_ON gpio_clear
#define LED_12_GPIO_OFF gpio_set
#define LED_12_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#define DefaultVoltageOfAdc(adc) (0.0045*adc)

/* Onboard ADCs */
/*
   ADC1 PC3/ADC13
   ADC2 PC0/ADC10
   ADC3 PC1/ADC11
   ADC4 PC5/ADC15
   ADC6 PC2/ADC12
   BATT PC4/ADC14
*/
/* We should remove the first three adc channels, as Lisa/S does not provide those. */
#define BOARD_ADC_CHANNEL_1 10
#define BOARD_ADC_CHANNEL_2 11
#define BOARD_ADC_CHANNEL_3 12
// we can only use ADC1,2,3; the last channel is for bat monitoring
#define BOARD_ADC_CHANNEL_4 2

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 0
#define ADC_2 1
#define ADC_3 2

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
#endif

/* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c */
// FIXME, this is not very nice, is also locm3 lib specific
#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) {                                          \
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,                               \
		  GPIO_CNF_INPUT_ANALOG,                                \
		  GPIO2);                       \
  }
#endif // USE_AD1

#define BOARD_HAS_BARO 1

/* SPI slave mapping */

/* IMU_MPU_CS */
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4

/* IMU_BARO_CS */
#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO4

/* RADIO_SS */
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
/* DIAG port */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

/* GPS */
#define UART3_GPIO_AF 0
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_RX
#define UART3_GPIO_RX GPIO_USART3_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_TX
#define UART3_GPIO_TX GPIO_USART3_TX

/* LED1 & LED2 */
/*
#define UART3_GPIO_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_PR_RX
#define UART3_GPIO_RX GPIO_USART3_PR_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_PR_TX
#define UART3_GPIO_TX GPIO_USART3_PR_TX
*/

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO2
#define SPEKTRUM_BIND_PIN_PORT GPIOB

/* Diag Port RX */
#define SPEKTRUM_UART1_RCC_REG &RCC_APB2ENR
#define SPEKTRUM_UART1_RCC_DEV RCC_APB2ENR_USART1EN
#define SPEKTRUM_UART1_BANK GPIO_BANK_USART1_RX
#define SPEKTRUM_UART1_PIN GPIO_USART1_RX
#define SPEKTRUM_UART1_AF 0
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

/* AUX Radio RX */
#define SPEKTRUM_UART2_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART2_RCC_DEV RCC_APB1ENR_USART2EN
#define SPEKTRUM_UART2_BANK GPIO_BANK_USART2_RX
#define SPEKTRUM_UART2_PIN GPIO_USART2_RX
#define SPEKTRUM_UART2_AF 0
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2

/* LED2 */
#define SPEKTRUM_UART3_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART3_RCC_DEV RCC_APB1ENR_USART3EN
#define SPEKTRUM_UART3_BANK GPIO_BANK_USART3_PR_RX
#define SPEKTRUM_UART3_PIN GPIO_USART3_PR_RX
#define SPEKTRUM_UART3_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define SPEKTRUM_UART3_IRQ NVIC_USART3_IRQ
#define SPEKTRUM_UART3_ISR usart3_isr
#define SPEKTRUM_UART3_DEV USART3

/* LED3 */
#define SPEKTRUM_UART5_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART5_RCC_DEV RCC_APB1ENR_UART5EN
#define SPEKTRUM_UART5_BANK GPIO_BANK_UART5_RX
#define SPEKTRUM_UART5_PIN GPIO_UART5_RX
#define SPEKTRUM_UART5_AF 0
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5

/* PPM
 */

/*
 * On Lisa/S there is no really dedicated port available. But we could use a
 * bunch of different pins to do PPM Input.
 */

/*
 * Default is PPM config 2, input on GPIO01 (Servo pin 6)
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 3
#endif


#if PPM_CONFIG == 1
/* input on PA01 (UART1_RX) Diag port */
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
/* input on PA1 (Servo 6 pin)
   On Lisa/S we could also use TIM5 IC2 instead. */
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

#elif PPM_CONFIG == 3
/* input on PA3 (Aux RX pin)
   On Lisa/S we could also use TIM5 IC4 instead. */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC4
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC4IE
#define PPM_CC_IF           TIM_SR_CC4IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO3
#define PPM_GPIO_AF         0

// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif
#define USE_AD_TIM1 1

#else
#error "Unknown PPM config"

#endif // PPM_CONFIG

/* ADC */

// active ADC
#define USE_AD1 1
#define USE_AD1_1 1
#define USE_AD1_2 1
#define USE_AD1_3 1
#define USE_AD1_4 1

/*
 * I2C
 *
 */
/* Servo1 & Servo2 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

/* GPS TX & RX */
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

/*
 * PWM
 *
 */
#define PWM_USE_TIM4 1
#define PWM_USE_TIM5 1


#if USE_SERVOS_1AND2
  #if USE_I2C1
    #error "You cannot USE_SERVOS_1AND2 and USE_I2C1 at the same time"
  #else
    #define ACTUATORS_PWM_NB 6
    #define USE_PWM1 1
    #define USE_PWM2 1
    #define PWM_USE_TIM4 1
  #endif
#else
  #define ACTUATORS_PWM_NB 4
#endif

#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

// Servo numbering on LisaM silkscreen/docs starts with 1

/* PWM_SERVO_x is the index of the servo in the actuators_pwm_values array */
/* Because PWM1 and 2 can be disabled we put them at the index 4 & 5 so that it
 * is easy to disable.
 */
#if USE_PWM1
#define PWM_SERVO_1 4
#define PWM_SERVO_1_TIMER TIM4
#define PWM_SERVO_1_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF 0
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 5
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF 0
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 0
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF 0
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 1
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_RCC_IOP RCC_APB2ENR_IOPBEN
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF 0
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 2
#define PWM_SERVO_5_TIMER TIM5
#define PWM_SERVO_5_RCC_IOP RCC_APB2ENR_IOPAEN
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO0
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 3
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_RCC_IOP RCC_APB2ENR_IOPAEN
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servos 1-4 or 3-4 on TIM4 depending on USE_SERVOS_1AND2 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

/* SuperbitRF mounted */
#define SUPERBITRF_SPI_DEV spi2
#define SUPERBITRF_RST_PORT GPIOC
#define SUPERBITRF_RST_PIN GPIO9
#define SUPERBITRF_DRDY_PORT GPIOC
#define SUPERBITRF_DRDY_PIN GPIO6
#define SUPERBITRF_FORCE_DSM2 FALSE

#endif /* CONFIG_LISA_S_0_1_H */
