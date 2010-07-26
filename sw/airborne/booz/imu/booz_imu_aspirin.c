#include "booz_imu.h"

#include "i2c.h"

struct BoozImuAspirin imu_aspirin;

/* initialize peripherals */
static void configure_gyro(void);
static void configure_mag(void);
static void configure_accel(void);


void booz_imu_impl_init(void) {

  imu_aspirin.status = AspirinStatusUninit;
  imu_aspirin.i2c_done = FALSE;
  imu_aspirin.gyro_available = FALSE;
  imu_aspirin.gyro_available_blaaa = FALSE;
  imu_aspirin.mag_ready_for_read = FALSE;
  imu_aspirin.mag_available = FALSE;
  imu_aspirin.accel_available = FALSE;
  
  booz_imu_aspirin_arch_init();

}


void booz_imu_periodic(void) {
  if (imu_aspirin.status == AspirinStatusUninit) {
    configure_gyro();
    configure_mag();
    //    configure_accel();
    imu_aspirin.status = AspirinStatusIdle;
  }
  else
    imu_aspirin.gyro_available_blaaa = TRUE; 
}


/* sends a serie of I2C commands to configure the ITG3200 gyro */
static void configure_gyro(void) {
  
  /* set gyro range to 2000deg/s and low pass at 256Hz */
  i2c2.buf[0] = ITG3200_REG_DLPF_FS;
  i2c2.buf[1] = 0x03;
  i2c2_transmit(ITG3200_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);
  /* switch to gyroX clock */
  i2c2.buf[0] = ITG3200_REG_PWR_MGM;
  i2c2.buf[1] = 0x01;
  i2c2_transmit(ITG3200_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);
  /* enable interrupt on data ready, idle hight */
  i2c2.buf[0] = ITG3200_REG_INT_CFG;
  i2c2.buf[1] = (0x01 | 0x01<<7);
  i2c2_transmit(ITG3200_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);

}

/* sends a serie of I2C commands to configure the ITG3200 gyro */
static void configure_mag(void) {

  /* set to rate to 50Hz */
  i2c2.buf[0] = HMC5843_REG_CFGA; 
  i2c2.buf[1] = 0x00 | (0x06 << 2);
  i2c2_transmit(HMC5843_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);
  /* set to gain to 1 Gauss */
  i2c2.buf[0] = HMC5843_REG_CFGB;
  i2c2.buf[1] = 0x01<<5;
  i2c2_transmit(HMC5843_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);
  /* set to continuous mode */
  i2c2.buf[0] = HMC5843_REG_MODE; 
  i2c2.buf[1] = 0x00;
  i2c2_transmit(HMC5843_ADDR, 2, &imu_aspirin.i2c_done);
  while (!imu_aspirin.i2c_done);

}

static void configure_accel(void) {

  /* set data rate to 800Hz */
  adxl345_write_to_reg(ADXL345_REG_BW_RATE, 0x0D);
  /* switch to measurememnt mode */
  adxl345_write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
  /* enable data ready interrupt */
  adxl345_write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7); 
  /* Enable full res and interrupt active low */
  adxl345_write_to_reg(ADXL345_REG_DATA_FORMAT, 1<<3|1<<5);
  /* clear spi rx reg to make DMA happy */
  adxl345_clear_rx_buf();
  /* reads data once to bring interrupt line up */
  adxl345_start_reading_data();
}
