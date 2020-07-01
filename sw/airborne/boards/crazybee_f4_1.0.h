/* Since there is atm no schematic of the board, many pin to pin io was deducted
 *  by measuring on the PCB.
 * Not all, e.g. the Volt and Current values are measured for the moment
 * Would be great if one could determine the resitor bridge true values
 */

#ifndef CONFIG_CRAZYBEE_F4_1_0_H
#define CONFIG_CRAZYBEE_F4_1_0_H

#define BOARD_CRAZYBEE_F4_V1

/** System memory base - for DFU bootloader */
#define SYSTEM_MEMORY_BASE 0x1FFF0000

/** Clock config - STM32F4 - STM32F411CEU6 in 48 pin package UFQFPN48 **/
#define EXT_CLK 8000000  // 8mHz
#define AHB_CLK 84000000

/** LEDs **/
/* Green LED on flight controller */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* Red/white RX LEDs */
/* Note: all LEDs are connected to a single pin */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO9
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear
#define LED_2_AFIO_REMAP ((void)0)

/** UART's **/
/* UART1 */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10

/* UART2 */
//Can connect to built-in DSMX receiver is availabe on UART RX
//Not to be confused with DSMX over SPI, that is unrelated
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3

/* SBUS inverted on UARTx is a separate physical pad on the board
 * To be used for an receiverv that gives an inverted SBUS out signal */

/* FIXME: (re)setting UART based (e.g. Spektum) Serial RADIO_CONTROL_POWER_PORT
#define RADIO_CONTROL_POWER_PORT GPIOA
#define RADIO_CONTROL_POWER_PIN GPIO10
#define RADIO_CONTROL_POWER_ON gpio_clear // yes, inverted
#define RADIO_CONTROL_POWER_OFF gpio_set
*/

/* FIXME: Soft binding Spektrum */
/*
#define SPEKTRUM_UART2_RCC RCC_USART1 //But uard 2 if embedded CYRF chip Speksat can be on UART1
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO10
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART2_ISR usart1_isr
#define SPEKTRUM_UART2_DEV USART1
*/

/* FIXME to relate this to ifddefs of  PPM config possibilities
 */
#ifndef USE_LED_STRIP
#define USE_LED_STRIP 1
#endif

#if USE_LED_STRIP
#define LED_STRIP_GPIO_PORT GPIOA
#define LED_STRIP_GPIO_PIN GPIO0
#endif

/* PPM
 *
 * FIXME: Default is PPM config 1, alternative 2 is CPPM input on RX2 pin but
 * than no UART RX at the same time, but a need for that scenario is unlikely anyhow
 *
 * Originaly intended for 2812LED board - DMA1_ST2
 * Can be re- used for input to connect a receiver that outputs CPPM pulsestrain
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 1
#endif

#ifdef PPM_CONFIG
#define USE_PPM_TIM5 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM5_IRQ
//#define PPM_IRQ             NVIC_TIM5_CC_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO0
#define PPM_GPIO_AF         GPIO_AF2

//#elif PPM_CONFIG == 2
///* RX SBUS/Spektumserial or CPPM input on PA3 (RX2 pin) */
//#define USE_PPM_TIM9 1
//#define PPM_CHANNEL         TIM_IC2
//#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
//#define PPM_IRQ             NVIC_TIM9_IRQ
//// Capture/Compare InteruptEnable and InterruptFlag
//#define PPM_CC_IE           TIM_DIER_CC2IE
//#define PPM_CC_IF           TIM_SR_CC2IF
//#define PPM_GPIO_PORT       GPIOA
//#define PPM_GPIO_PIN        GPIO3
//#define PPM_GPIO_AF         GPIO_AF3
///* TODO: add option 3 of input on RX1 pin) */
//#else
//#error "Unknown PPM config"

#endif // PPM_CONFIG

/** SPI **/
/* SPI1 for MPU accel/gyro (MPU6000*/
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7

/* SPI slave pin declaration ACC_GYRO_CS on SPI1 ICM 20609-G*/
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4

/* SPI2 for embedded OSD MAX chip*/
//#ifndef USE_MAX7456
//#define USE_MAX7456 0
//#endif

//#if USE_MAX7456
#define SPI2_GPIO_AF GPIO_AF5 //TODO check datasheet
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15

/* SPI slave pin declaration OSD */
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12
//#endif

/* Used SPI3 for RX direct, if RX solution is implemented in AP */
#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_SCK GPIOB
#define SPI3_GPIO_SCK GPIO3
#define SPI3_GPIO_PORT_MISO GPIOB
#define SPI3_GPIO_MISO GPIO4
#define SPI3_GPIO_PORT_MOSI GPIOB
#define SPI3_GPIO_MOSI GPIO5

/* SPI slave pin declaration for Receiver */
#define SPI_SELECT_SLAVE2_PORT GPIOA
#define SPI_SELECT_SLAVE2_PIN GPIO15

/* GDO0 pin for receiver */
#define CC2500_GDO0_GPIO GPIOC
#define CC2500_GDO0_PIN  GPIO14

/* Bind button */
#define BIND_BUTTON_GPIO GPIOB
#define BIND_BUTTON_PIN  GPIO2

/** Onboard ADCs **/

#define USE_AD_TIM1 1

/* Voltage */
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif

#if USE_ADC_1
#define AD1_1_CHANNEL 8
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO0

#define ADC_CHANNEL_VSUPPLY ADC_1
#define DefaultVoltageOfAdc(adc) (0.009*adc)// TODO: determine 100% correct value
#endif

/* Current */
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL 9
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOB
#define ADC_2_GPIO_PIN GPIO1

#define ADC_CHANNEL_CURRENT ADC_2
#define MilliAmpereOfAdc(adc)((float)adc) * (3.3f / 4096.0f) * (90.0f / 5.0f)// TODO: determine 100% correct value
#endif

/* TODO: Somehere on the board find PHiSICAL easily reachable I2C SDL SDA so to connect e.g. GNSS, Baro, Magneto
 * TODO: Finish the I2C mapping if Pins found */

//#define I2C1_GPIO_AF GPIO_AF4
//#define I2C1_GPIO_PORT GPIOB
//#define I2C1_GPIO_SCL GPIO8
//#define I2C1_GPIO_SDA GPIO9

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* PWM */
#define PWM_USE_TIM2 1
#define PWM_USE_TIM4 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1

/* ESC 1 (B10, TIM2, CH3)*/
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO10
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

/* ESC 2 (B6, TIM4, CH1)*/
#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM4
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO6
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC1
#define PWM_SERVO_2_OC_BIT (1<<0)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

/* ESC 3 (B7, TIM4, CH2) */
#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO7
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC2
#define PWM_SERVO_3_OC_BIT (1<<1)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

/* ESC 4 (B8, TIM4, CH3) */
#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO8
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC3
#define PWM_SERVO_4_OC_BIT (1<<2)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#define PWM_TIM2_CHAN_MASK (PWM_SERVO_1_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)

/* Buzzer (C15, inverted) */
#if USE_BUZZER
#define PWM_BUZZER
#define PWM_BUZZER_GPIO GPIOC
#define PWM_BUZZER_PIN GPI15
#define PWM_BUZZER_GPIO_ON  gpio_clear
#define PWM_BUZZER_GPIO_OFF gpio_set
#endif

#endif /* CONFIG_CRAZYBEE_F4_1_0_H */
