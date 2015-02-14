#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/dma.h>
#include <stm32/adc.h>
#include <stm32/i2c.h>

#include <string.h>

#include "firmwares/beth/bench_sensors.h"

/*
 *
 *  PC.01 (ADC Channel11) ext1-20 coder_values[1]
Paul : using channel 10 instead of 14
 *  PC.04 (ADC Channel14) ext2-12 coder_values[0]
 *
 *  PB.10 I2C2 SCL        ext2-14
 *  PB.11 I2C2 SDA        ext2-15
 *
 */


static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static inline void main_init_adc(void);
//static inline void main_on_bench_sensors( void );

//static inline void main_init_i2c2(void);
//void i2c2_ev_irq_handler(void);
//void i2c2_er_irq_handler(void);

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
static uint16_t coder_values[3];

//azimuth potentiometer board address is 0x30
//#define I2C2_SLAVE_ADDRESS7     0x30
//elevation and tilt pot board is 0x40
#define I2C2_SLAVE_ADDRESS7     0x40
#define I2C2_ClockSpeed       200000

#define MY_I2C2_BUF_LEN 4
//static uint8_t i2c2_idx;
//static uint8_t i2c2_buf[MY_I2C2_BUF_LEN];

uint16_t servos[4];

int main(void)
{
  main_init();

  servos[0] = 1;
  servos[1] = 2;
  servos[2] = 3;
  servos[3] = 4;

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }
  return 0;
}


static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  main_init_adc();
  bench_sensors_init();
  mcu_int_enable();
}

static inline void main_periodic(void)
{

  /*RunOnceEvery(10, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});*/

  //RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &coder_values[0], &coder_values[1]);});
  //RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &can1_status, &can1_pending);});

  /*RunOnceEvery(5, {DOWNLINK_SEND_BETH(DefaultChannel, &bench_sensors.angle_1,
    &bench_sensors.angle_2,&bench_sensors.angle_3, &bench_sensors.current);});*/

  servos[0] = coder_values[0];
  servos[1] = coder_values[1];
  //use id=1 for azimuth board
  can_transmit(1, (uint8_t *)servos, 8);
  LED_TOGGLE(5);
}


static inline void main_event(void)
{
  //BenchSensorsEvent(main_on_bench_sensors);
}


/*static inline void main_on_bench_sensors( void ) {

}*/


#if 0
/*
 *
 *  I2C2 : autopilot link
 *
 */
void i2c2_init(void)
{
//  static inline void main_init_i2c2(void) {

  /* System clocks configuration ---------------------------------------------*/
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* NVIC configuration ------------------------------------------------------*/
  NVIC_InitTypeDef  NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Configure and enable I2C2 event interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Configure and enable I2C2 error interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);


  /* GPIO configuration ------------------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Configure I2C2 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable I2C2 -------------------------------------------------------------*/
  I2C_Cmd(I2C2, ENABLE);
  /* I2C2 configuration ------------------------------------------------------*/
  I2C_InitTypeDef   I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C2_ClockSpeed;
  I2C_Init(I2C2, &I2C_InitStructure);

  /* Enable I2C1 event and buffer interrupts */
  //  I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
  I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

}


void i2c2_ev_irq_handler(void)
{
  switch (I2C_GetLastEvent(I2C2)) {
      /* Slave Transmitter ---------------------------------------------------*/
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:  /* EV1 */
      memcpy(i2c2_buf, coder_values, MY_I2C2_BUF_LEN);
      i2c2_idx = 0;

    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:             /* EV3 */
      /* Transmit I2C2 data */
      if (i2c2_idx < MY_I2C2_BUF_LEN) {
        I2C_SendData(I2C2, i2c2_buf[i2c2_idx]);
        i2c2_idx++;
      }
      break;


    case I2C_EVENT_SLAVE_STOP_DETECTED:                /* EV4 */
      LED_ON(1);
      /* Clear I2C2 STOPF flag: read of I2C_SR1 followed by a write on I2C_CR1 */
      (void)(I2C_GetITStatus(I2C2, I2C_IT_STOPF));
      I2C_Cmd(I2C2, ENABLE);
      break;

  }
}


void i2c2_er_irq_handler(void)
{
  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C2, I2C_IT_AF))  {
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
  }
}


#endif

/*
 *
 *  ADC : coders
 *
 */




static inline void main_init_adc(void)
{

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                         RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure PC.01 (ADC Channel11) and PC.04 (ADC Channel14) as analog input-*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&coder_values;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5);
  //Paul: Changing to use chan 10 instead
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);


  /* ADC2 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  /* ADC2 regular channels configuration */
  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);
  /* Enable ADC2 external trigger conversion */
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);


  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while (ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while (ADC_GetCalibrationStatus(ADC1));

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC2 reset calibaration register */
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while (ADC_GetResetCalibrationStatus(ADC2));

  /* Start ADC2 calibaration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while (ADC_GetCalibrationStatus(ADC2));


  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

