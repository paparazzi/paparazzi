#include "subsystems/imu.h"

#include "led.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/spi_arch.h"

// Peripherials
#include "../../peripherals/mpu60X0.h"


struct ImuAspirin2 imu_aspirin2;

struct spi_transaction aspirin2_mpu60x0;



// initialize peripherals
static void configure(void);
/*
static void configure_accel(void);
//static void configure_mag(void);

*/

void imu_impl_init(void) {

  imu_aspirin2.status = Aspirin2StatusUninit;
  imu_aspirin2.imu_available = FALSE;

  aspirin2_mpu60x0.mosi_buf = imu_aspirin2.imu_tx_buf;
  aspirin2_mpu60x0.miso_buf = imu_aspirin2.imu_rx_buf;
  aspirin2_mpu60x0.ready = &(imu_aspirin2.imu_available);
  aspirin2_mpu60x0.length = 2;

//  imu_aspirin2_arch_init();

}


void imu_periodic(void) 
{

  if (imu_aspirin2.status == Aspirin2StatusUninit) 
  {
    configure();
    // imu_aspirin_arch_int_enable();
    imu_aspirin2.status = Aspirin2StatusIdle;

    aspirin2_mpu60x0.length = 22;
    aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_INT_STATUS + MPU60X0_SPI_READ;
    {
      for (int i=1;i<aspirin2_mpu60x0.length;i++)
        aspirin2_mpu60x0.mosi_buf[i] = 0;
    }
  }
  else
  {

    // imu_aspirin2.imu_tx_buf[0] = MPU60X0_REG_WHO_AM_I + MPU60X0_SPI_READ;
    // imu_aspirin2.imu_tx_buf[1] = 0x00;

    spi_rw(&aspirin2_mpu60x0);

/*
    imu_aspirin.time_since_last_reading++;
    imu_aspirin.time_since_last_accel_reading++;
    if (imu_aspirin.time_since_last_accel_reading > ASPIRIN_ACCEL_TIMEOUT) 
    {
      configure_accel();
      imu_aspirin.time_since_last_accel_reading=0;
    }
*/
  } 
}

static void configure(void) 
{
  aspirin2_mpu60x0.length = 2;

  ///////////////////
  // Configure power:

  // MPU60X0_REG_AUX_VDDIO = 0 (good on startup)

  // MPU60X0_REG_USER_CTRL:
  // -Enable Aux I2C Master Mode
  // -Enable SPI

  // MPU60X0_REG_PWR_MGMT_1
  // -switch to gyroX clock
  aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_PWR_MGMT_1;
  aspirin2_mpu60x0.mosi_buf[1] = 0x01;
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);

  // MPU60X0_REG_PWR_MGMT_2: Nothing should be in standby: default OK

  /////////////////////////
  // Measurement Settings

  // MPU60X0_REG_CONFIG
  // -ext sync on gyro X (bit 3->6)
  // -digital low pass filter: 1kHz sampling of gyro/acc with 44Hz bandwidth: since reading is at 100Hz
#if PERIODIC_FREQUENCY == 60
#else
#  if PERIODIC_FREQUENCY == 120
#  else
#  error PERIODIC_FREQUENCY should be either 60Hz or 120Hz. Otherwise manually fix the sensor rates
#  endif
#endif
  aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_CONFIG;
  aspirin2_mpu60x0.mosi_buf[1] = (2 << 3) | (3 << 0);
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);

  // MPU60X0_REG_SMPLRT_DIV
  // -100Hz output = 1kHz / (9 + 1)
  aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_SMPLRT_DIV;
  aspirin2_mpu60x0.mosi_buf[1] = 9;
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);

  // MPU60X0_REG_GYRO_CONFIG
  // -2000deg/sec
  aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_GYRO_CONFIG;
  aspirin2_mpu60x0.mosi_buf[1] = (3<<3);
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);

  // MPU60X0_REG_ACCEL_CONFIG
  // 16g, no HPFL
  aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_ACCEL_CONFIG;
  aspirin2_mpu60x0.mosi_buf[1] = (3<<3);
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);

/*
  struct i2c_transaction t;
  t.type = I2CTransTx;
  t.slave_addr = ITG3200_ADDR;
  // set gyro range to 2000deg/s and low pass at 256Hz
  t.mosi_buf[0] = ITG3200_REG_DLPF_FS;
  t.mosi_buf[1] = (0x03<<3);
  t.len_w = 2;
  send_i2c_msg_with_retry(&t);
  // set sample rate to 533Hz 
  t.mosi_buf[0] = ITG3200_REG_SMPLRT_DIV;
  t.mosi_buf[1] = 0x0E;
  send_i2c_msg_with_retry(&t);
  // switch to gyroX clock 
  t.mosi_buf[0] = ITG3200_REG_PWR_MGM;
  t.mosi_buf[1] = 0x01;
  send_i2c_msg_with_retry(&t);
  // enable interrupt on data ready, idle high, latch until read any register
  t.mosi_buf[0] = ITG3200_REG_INT_CFG;
  t.mosi_buf[1] = (0x01 | (0x1<<4) | (0x1<<5) | 0x01<<7);
  send_i2c_msg_with_retry(&t);
*/
}


/*
static void configure_accel(void) 
{
  // set data rate to 800Hz
  adxl345_write_to_reg(ADXL345_REG_BW_RATE, 0x0D);
  // switch to measurememnt mode 
  adxl345_write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
  // enable data ready interrupt 
  adxl345_write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7);
  // Enable full res and interrupt active low 
  adxl345_write_to_reg(ADXL345_REG_DATA_FORMAT, 1<<3|1<<5);
  // clear spi rx reg to make DMA happy 
  adxl345_clear_rx_buf();
  // reads data once to bring interrupt line up 
  adxl345_start_reading_data();

}
*/


