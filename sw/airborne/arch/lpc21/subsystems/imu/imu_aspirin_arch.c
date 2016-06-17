#include "subsystems/imu.h"


#include "LPC21xx.h"

//#define ASPIRIN_GYRO_EOC_PINSEL PINSEL1
//#define ASPIRIN_GYRO_EOC_PINSEL_BIT 0
//#define ASPIRIN_GYRO_EOC_PINSEL_VAL 1


void imu_aspirin_arch_int_enable(void)
{
}

void imu_aspirin_arch_int_disable(void)
{
}

void imu_aspirin_arch_init(void)
{
  /* set input pin for gyro eoc */
  ClearBit(ASPIRIN_GYRO_EOC_IODIR, ASPIRIN_GYRO_EOC_PIN);

  //ASPIRIN_GYRO_EOC_PINSEL |= ASPIRIN_GYRO_EOC_PINSEL_VAL << ASPIRIN_GYRO_EOC_PINSEL_BIT;
}


/****** the interrupts should be handled in the peripheral drivers *******/

/*
 * Gyro data ready
 */
void exti15_10_irq_handler(void)
{

  /* clear EXTI */
//  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
//    EXTI_ClearITPendingBit(EXTI_Line14);

  //imu_aspirin.gyro_eoc = true;
  //imu_aspirin.status = AspirinStatusReadingGyro;
}

/*
 * Accel data ready
 */
void exti2_irq_handler(void)
{

  /* clear EXTI */
//  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
//    EXTI_ClearITPendingBit(EXTI_Line2);

}

/*
 * Accel end of DMA transfer
 */
void dma1_c4_irq_handler(void)
{

  //imu_aspirin.accel_available = true;
}
