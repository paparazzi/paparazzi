#include "subsystems/imu.h"

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>

#include "mcu_periph/i2c.h"

/* gyro int handler */
void exti15_10_isr(void);
/* mag int handler  */
void exti9_5_isr(void);
/* accelerometer int handler */
void exti2_isr(void);
/* dma1 channel 4 int handler */
void dma1_channel4_isr(void);
/* accelerometer SPI selection */
#define Adxl345Unselect() GPIOB_BSRR = GPIO12
#define Adxl345Select()   GPIOB_BRR = GPIO12
/* accelerometer dma end of rx handler */
void dma1_c4_irq_handler(void);

void imu_aspirin_arch_int_enable(void) {

#ifdef ASPIRIN_USE_GYRO_INT
  nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
#endif

  nvic_set_priority(NVIC_EXTI2_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI2_IRQ);

  /* Enable DMA1 channel4 IRQ Channel ( SPI RX) */
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

}

void imu_aspirin_arch_int_disable(void) {

#ifdef ASPIRIN_USE_GYRO_INT
  nvic_disable_irq(NVIC_EXTI15_10_IRQ);
#endif

  nvic_disable_irq(NVIC_EXTI2_IRQ);

  /* Enable DMA1 channel4 IRQ Channel ( SPI RX) */
  nvic_disable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void imu_aspirin_arch_init(void) {

  /* Set "mag ss" and "mag reset" as floating inputs ------------------------*/
  /* "mag ss"    (PC12) is shorted to I2C2 SDA       */
  /* "mag reset" (PC13) is shorted to I2C2 SCL       */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO12 | GPIO13);

  /* Gyro --------------------------------------------------------------------*/
  /* configure external interrupt exti15_10 on PC14( gyro int ) */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN |
			                    RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO14);

#ifdef ASPIRIN_USE_GYRO_INT
  exti_select_source(EXTI14, GPIOC);
  exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI14);
#endif

  /* Accel */
  /* set accel slave select as output and assert it ( on PB12) */
  Adxl345Unselect();
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

  /* configure external interrupt exti2 on PB2( accel int ) */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO2);
  exti_select_source(EXTI2, GPIOB);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI2);

  /* Enable SPI2 Periph clock -------------------------------------------------*/
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

  /* Configure GPIOs: SCK, MISO and MOSI  --------------------------------*/
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
	                                        GPIO_SPI2_MISO |
	                                        GPIO_SPI2_MOSI);

  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);

  // reset SPI
  spi_reset(SPI2);

  // Disable SPI peripheral
  spi_disable(SPI2);

  // configure SPI
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  // Enable SPI2 periph.
  spi_enable(SPI2);

  /* Enable SPI_2 DMA clock ---------------------------------------------------*/
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

}


void adxl345_write_to_reg(uint8_t addr, uint8_t val) {

  Adxl345Select();
  SPI_DR(SPI2) = addr;
  while ((SPI_SR(SPI2) & SPI_SR_TXE) == 0);
  SPI_DR(SPI2) = val;
  while ((SPI_SR(SPI2) & SPI_SR_TXE) == 0);
  while ((SPI_SR(SPI2) & SPI_SR_BSY) != 0);
  Adxl345Unselect();

}

void adxl345_clear_rx_buf(void) {
  uint8_t __attribute__ ((unused)) ret = SPI_DR(SPI2);
}

void adxl345_start_reading_data(void) {
   Adxl345Select();

   imu_aspirin.accel_tx_buf[0] = (1<<7|1<<6|ADXL345_REG_DATA_X0);

  // SPI2_Rx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)imu_aspirin.accel_rx_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, 7);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL4);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL4);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL4, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

  // SPI2_Tx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)imu_aspirin.accel_tx_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL5, 7);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL5);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL5, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_MEDIUM);

  // Enable DMA1 Channel4
  dma_enable_channel(DMA1, DMA_CHANNEL4);
  // Enable SPI_2 Rx request
  spi_enable_rx_dma(SPI2);

  // Enable DMA1 Channel5
  dma_enable_channel(DMA1, DMA_CHANNEL5);
  // Enable SPI_2 Tx request
  spi_enable_tx_dma(SPI2);

  // Enable DMA1 Channel4 Transfer Complete interrupt
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

}

/*
 *
 * Gyro data ready
 *
 */
void exti15_10_isr(void) {

  /* clear EXTI */
  exti_reset_request(EXTI14);

#ifdef ASPIRIN_USE_GYRO_INT
  imu_aspirin.gyro_eoc = TRUE;
  imu_aspirin.status = AspirinStatusReadingGyro;
#endif

}

/*
 *
 * Accel data ready
 *
 */
void exti2_isr(void) {

  /* clear EXTI */
  exti_reset_request(EXTI2);

  adxl345_start_reading_data();

}

/*
 *
 * Accel end of DMA transferred
 *
 */
void dma1_channel4_isr(void) {
  Adxl345Unselect();

  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;

    // mark as available
    imu_aspirin.accel_available = TRUE;
  }

  // disable DMA Channel
  dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

  // Disable SPI_2 Rx and TX request
  spi_disable_rx_dma(SPI2);
  spi_disable_tx_dma(SPI2);

  // Disable DMA1 Channel4 and 5
  dma_disable_channel(DMA1, DMA_CHANNEL4);
  dma_disable_channel(DMA1, DMA_CHANNEL5);

}
