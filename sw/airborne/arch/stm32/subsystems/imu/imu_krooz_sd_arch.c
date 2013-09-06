#include "subsystems/imu.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/exti.h>

#include "subsystems/imu/imu_krooz_sd_arch.h"

void imu_krooz_sd_arch_init(void) {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);

  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  exti_select_source(EXTI5, GPIOB);
  exti_select_source(EXTI6, GPIOC);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI6, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI5);
  exti_enable_request(EXTI6);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
}

void exti9_5_isr(void) {
  /* clear EXTI */
  if(EXTI_PR & EXTI6) {
    exti_reset_request(EXTI6);
    imu_krooz.hmc_eoc = TRUE;
  }
  if(EXTI_PR & EXTI5) {
    exti_reset_request(EXTI5);
    imu_krooz.mpu_eoc = TRUE;
  }
}
