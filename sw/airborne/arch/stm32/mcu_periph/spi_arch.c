#include "subsystems/imu.h"

#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>

#include "mcu_periph/spi.h"

// SPI2 Slave Selection
#define Spi2Slave0Unselect() GPIO_BSRR(GPIOB) = GPIO12
#define Spi2Slave0Select()   GPIO_BRR(GPIOB) = GPIO12

// spi dma end of rx handler
// XXX: should be provided by libopencm3?
void dma1_channel4_isr(void);

void spi_arch_int_enable(void) {

  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void spi_arch_int_disable(void) {

  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_disable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void spi_init(void) {

  // Enable SPI2 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
	                                        GPIO_SPI2_MISO |
	                                        GPIO_SPI2_MOSI);

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

  // Enable SPI_2 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // SLAVE 0
  // set accel slave select as output and assert it ( on PB12)
  Spi2Slave0Unselect();
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

  spi_arch_int_enable();

}

/*
void adxl345_write_to_reg(uint8_t addr, uint8_t val) {

  Adxl345Select();
  SPI_I2S_SendData(SPI2, addr);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, val);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  Adxl345Unselect();
}

void spi_clear_rx_buf(void) {
  uint8_t __attribute__ ((unused)) ret = SPI_I2S_ReceiveData(SPI2);
}
*/

struct spi_transaction* slave0;

void spi_rw(struct spi_transaction  * _trans)
{
  // Store local copy to notify of the results
  slave0 = _trans;
  slave0->status = SPITransRunning;

  Spi2Slave0Select();


  // SPI2_Rx_DMA_Channel configuration ------------------------------------

  dma_channel_reset(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)slave0->miso_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, slave0->length);
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
  dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)slave0->mosi_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL5, slave0->length);
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

// Accel end of DMA transferred
void dma1_channel4_isr(void)
{

  Spi2Slave0Unselect();

  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;

    // mark as available
    spi_message_received = TRUE;
  }

  // disable DMA Channel
  dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

  // Disable SPI_2 Rx and TX request
  spi_disable_rx_dma(SPI2);
  spi_disable_tx_dma(SPI2);

  // Disable DMA1 Channel4 and 5
  dma_disable_channel(DMA1, DMA_CHANNEL4);
  dma_disable_channel(DMA1, DMA_CHANNEL5);

  slave0->status = SPITransSuccess;
  *(slave0->ready) = 1;

}



