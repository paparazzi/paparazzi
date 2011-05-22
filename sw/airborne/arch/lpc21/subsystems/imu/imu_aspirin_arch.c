#include "subsystems/imu.h"


#include "mcu_periph/i2c.h"


void imu_aspirin_arch_int_enable(void)
{
}

void imu_aspirin_arch_int_disable(void)
{
}

void imu_aspirin_arch_init(void)
{
}


void adxl345_write_to_reg(uint8_t addr, uint8_t val) {

//  Adxl345Select();
//  SPI_I2S_SendData(SPI2, addr);
//  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//  SPI_I2S_SendData(SPI2, val);
//  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
//  Adxl345Unselect();

}

void adxl345_clear_rx_buf(void) {
}

//void adxl345_start_reading_data(void) {
//}

/*
 *
 * Gyro data ready
 *
 */

void exti15_10_irq_handler(void) {

  /* clear EXTI */
//  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
//    EXTI_ClearITPendingBit(EXTI_Line14);

  imu_aspirin.gyro_eoc = TRUE;
  imu_aspirin.status = AspirinStatusReadingGyro;

}

/*
 *
 * Accel data ready
 *
 */
void exti2_irq_handler(void) {

  /* clear EXTI */
//  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
//    EXTI_ClearITPendingBit(EXTI_Line2);

//  adxl345_start_reading_data();

}

/*
 *
 * Accel end of DMA transfert
 *
 */
void dma1_c4_irq_handler(void) {

  imu_aspirin.accel_available = TRUE;
}
