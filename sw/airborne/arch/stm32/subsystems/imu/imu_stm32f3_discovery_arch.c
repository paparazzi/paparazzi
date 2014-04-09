#include "subsystems/imu.h"

#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/gpio.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include "subsystems/imu/imu_stm32f3_discovery_arch.h"

void imu_stm32f3_discovery_arch_init(void) {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPCEN);
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO10);

  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);
  gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2);
  gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4);
  gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);

  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
  nvic_enable_irq(NVIC_EXTI2_TSC_IRQ);
  nvic_enable_irq(NVIC_EXTI4_IRQ);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  exti_select_source(EXTI10, GPIOC);
  exti_select_source(EXTI2, GPIOE);
  exti_select_source(EXTI4, GPIOE);
  exti_select_source(EXTI5, GPIOE);
  exti_set_trigger(EXTI10, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI4, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI10);
  exti_enable_request(EXTI2);
  exti_enable_request(EXTI4);
  exti_enable_request(EXTI5);
  nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x0F);
  nvic_set_priority(NVIC_EXTI2_TSC_IRQ, 0x0F);
  nvic_set_priority(NVIC_EXTI4_IRQ, 0x0F);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
}

void exti15_10_isr(void) {
  if(EXTI_PR & EXTI10) {
    exti_reset_request(EXTI10);
    imu_stm32f3_discovery.mpu_eoc = TRUE;
  }
}

void exti2_tsc_isr(void) {
  lsm_isr();
}

void exti4_isr(void) {
  lsm_isr();
}

void exti9_5_isr(void) {
  lsm_isr();
}

void lsm_isr(void) {
  /* clear EXTI */
  if(EXTI_PR & EXTI2) {
    exti_reset_request(EXTI2);
    imu_stm32f3_discovery.lsm_m_eoc = TRUE;
  }
  if(EXTI_PR & EXTI4) {
    exti_reset_request(EXTI4);
    imu_stm32f3_discovery.lsm_a_eoc = TRUE;
  }
  if(EXTI_PR & EXTI5) {
    exti_reset_request(EXTI5);
    imu_stm32f3_discovery.lsm_a_eoc = TRUE;
  }
}
