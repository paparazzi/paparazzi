#ifndef CONFIG_LISA_M_COMMON_H
#define CONFIG_LISA_M_COMMON_H

/* SPI slave mapping */

#define SPI_SELECT_SLAVE0_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PERIPH RCC_APB2ENR_IOPBEN
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12

#define SPI_SELECT_SLAVE5_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO4

/* PPM */

#if PPM_CONFIG == 1
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM1_UP_IRQ
#define PPM_IRQ2            NVIC_TIM1_CC_IRQ
#define PPM_IRQ_FLAGS       (TIM_DIER_CC3IE | TIM_DIER_UIE)
#define PPM_GPIO_PERIPHERAL RCC_APB2ENR_IOPAEN
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10

#elif PPM_CONFIG == 2
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
#define PPM_IRQ_FLAGS       (TIM_DIER_CC2IE | TIM_DIER_UIE)
#define PPM_GPIO_PERIPHERAL RCC_APB2ENR_IOPAEN
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1

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

/* PWM */
// TODO correctly set PWM for new driver
// exmaple in apogee board file
// following code is the old config from the C file

//#if USE_SERVOS_7AND8
//#if (defined(BOARD_LISA_M) || defined(BOARD_LIA)) && USE_I2C1
//#error "You cannot use Servos 7and8 and I2C1 at the same time"
//#else
//#define ACTUATORS_PWM_NB 8
//#endif
//#else
//#define ACTUATORS_PWM_NB 6
//#endif

//#if defined(STM32F1)
//
//#ifndef PWM_USE_TIM3
//#define PWM_USE_TIM3 1
//#endif
//
//#ifndef PWM_USE_TIM4
//#if (!REMAP_SERVOS_5AND6 || USE_SERVOS_7AND8)
//#if !REMAP_SERVOS_5AND6
//PRINT_CONFIG_MSG("Not remapping servos 5 and 6 using PB8 and PB9 -> TIM4")
//#endif
//#if USE_SERVOS_7AND8
//PRINT_CONFIG_MSG("Enabling sevros 7 and 8 on PB6, PB7 -> TIM4")
//#endif
//#define PWM_USE_TIM4 1
//#endif // REMAP||7&8
//#endif // USE_TIM4
//
//#ifndef PWM_USE_TIM5
//#if REMAP_SERVOS_5AND6
//#ifdef REMAP_SERVOS_5AND6
//PRINT_CONFIG_MSG("Remapping servo outputs 5 and 6 to PA0,PA1 -> TIM5")
//#endif
//#define PWM_USE_TIM5 1
//#endif // 5&6
//#endif // USE_TIM5
//
//#error "PWM timer config should be in board.h files"
//#endif // F1

//  /* Disable outputs. */
//#if USE_SERVOS_7AND8
//  timer_disable_oc_output(TIM4, TIM_OC1);
//  timer_disable_oc_output(TIM4, TIM_OC2);
//#endif
//#if !REMAP_SERVOS_5AND6
//  timer_disable_oc_output(TIM4, TIM_OC3);
//  timer_disable_oc_output(TIM4, TIM_OC4);
//#endif
//
//  /* -- Channel configuration -- */
//#if USE_SERVOS_7AND8
//  actuators_pwm_arch_channel_init(TIM4, TIM_OC1);
//  actuators_pwm_arch_channel_init(TIM4, TIM_OC2);
//#endif
//#if !REMAP_SERVOS_5AND6
//  actuators_pwm_arch_channel_init(TIM4, TIM_OC3);
//  actuators_pwm_arch_channel_init(TIM4, TIM_OC4);
//#endif
//
//  /* Disable outputs. */
//  timer_disable_oc_output(TIM5, TIM_OC1);
//  timer_disable_oc_output(TIM5, TIM_OC2);
//
//  /* -- Channel configuration -- */
//  actuators_pwm_arch_channel_init(TIM5, TIM_OC1);
//  actuators_pwm_arch_channel_init(TIM5, TIM_OC2);

#endif

