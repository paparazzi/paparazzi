#include "subsystems/imu.h"

#include "led.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/spi_arch.h"

// Peripherials
#include "../../peripherals/mpu60X0.h"
#include "../../peripherals/hmc58xx.h"


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

static inline void mpu_set(uint8_t _reg, uint8_t _val)
{
  aspirin2_mpu60x0.mosi_buf[0] = _reg;
  aspirin2_mpu60x0.mosi_buf[1] = _val;
  spi_rw(&aspirin2_mpu60x0);
    while(aspirin2_mpu60x0.status != SPITransSuccess);
}

static inline void mpu_wait_slave4_ready(void)
{
  uint8_t ret = 0x80;
  while (ret & 0x80)
  {
    aspirin2_mpu60x0.mosi_buf[0] = MPU60X0_REG_I2C_SLV4_CTRL | MPU60X0_SPI_READ ;
    aspirin2_mpu60x0.mosi_buf[1] = 0;
    spi_rw(&aspirin2_mpu60x0);
      while(aspirin2_mpu60x0.status != SPITransSuccess);

    ret = aspirin2_mpu60x0.miso_buf[1];
  }
}



static void configure(void)
{
  aspirin2_mpu60x0.length = 2;

  ///////////////////
  // Configure power:

  // MPU60X0_REG_PWR_MGMT_1
  mpu_set( MPU60X0_REG_PWR_MGMT_1,
           0x01);		// -switch to gyroX clock

  // MPU60X0_REG_PWR_MGMT_2: Nothing should be in standby: default OK
  // -No standby and no wake timer

  /////////////////////////
  // Measurement Settings

#if PERIODIC_FREQUENCY == 60
// Accelerometer: Bandwidth 44Hz, Delay 4.9ms
// Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1Khz
#  define MPU_DIG_FILTER 3
// -100Hz output = 1kHz / (9 + 1)
#  define MPU_SMPLRT_DIV 9
#else
#  if PERIODIC_FREQUENCY == 120
//   Accelerometer: Bandwidth 44Hz, Delay 4.9ms
//   Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1Khz
#    define MPU_DIG_FILTER 3
//   -100Hz output = 1kHz / (9 + 1)
#    define MPU_SMPLRT_DIV 9
#  else
#    if PERIODIC_FREQUENCY == 512
//     Accelerometer: Bandwidth 260Hz, Delay 0ms
//     Gyroscope: Bandwidth 256Hz, Delay 0.89ms sampling 8Khz
#      define MPU_DIG_FILTER 0
//     -500Hz output = 1kHz / (1 + 1)
#      define MPU_SMPLRT_DIV 1
#    else
#    error PERIODIC_FREQUENCY should be either 60Hz, 120Hz or 512Hz. Otherwise manually fix the sensor rates
#    endif
#  endif
#endif

  // MPU60X0_REG_CONFIG
  mpu_set( MPU60X0_REG_CONFIG,
           (2 << 3) | 			// Fsync / ext sync on gyro X (bit 3->6)
           (MPU_DIG_FILTER << 0) );	// Low-Pass Filter

  // MPU60X0_REG_SMPLRT_DIV
  mpu_set( MPU60X0_REG_SMPLRT_DIV, MPU_SMPLRT_DIV);

  // MPU60X0_REG_GYRO_CONFIG
  mpu_set( MPU60X0_REG_GYRO_CONFIG,
           (3 << 3) );			// -2000deg/sec

  // MPU60X0_REG_ACCEL_CONFIG
  mpu_set( MPU60X0_REG_ACCEL_CONFIG,
           (0 << 0) |			// No HPFL
           (3 << 3) );			// Full Scale = 16g

#ifndef MPU6000_NO_SLAVES

  /////////////////////////////////////
  // SPI Slave Configuration Section

  // Power the Aux I2C Circuit:
  // MPU60X0_REG_AUX_VDDIO = 0 (good on startup):  (0 << 7);	// MPU6000: 0=Vdd. MPU6050 : 0=VLogic 1=Vdd

  // MPU60X0_REG_USER_CTRL:
  mpu_set( MPU60X0_REG_USER_CTRL,
           (1 << 5) |		// I2C_MST_EN: Enable Aux I2C Master Mode
           (1 << 4) |		// I2C_IF_DIS: Disable I2C on primary interface
           (0 << 1) );		// Trigger a I2C_MST_RESET

  // Enable the aux i2c
  mpu_set( MPU60X0_REG_I2C_MST_CTRL,
           (0 << 7) | 		// no multimaster
	   (0 << 6) |		// do not delay IRQ waiting for all external slaves
	   (0 << 5) | 		// no slave 3 FIFO
	   (0 << 4) | 		// restart or stop/start from one slave to another: read -> write is always stop/start
	   (8 << 0) );		// 0=348kHz 8=256kHz, 9=500kHz


  // HMC5883 Magnetometer Configuration

  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_CFGA);
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_CRA);
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (0 << 0) );		// Full Speed

  mpu_wait_slave4_ready();

  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_CFGB);
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_CRB);
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (0 << 0) );		// Full Speed

  mpu_wait_slave4_ready();

  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_MODE);
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_MD);
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (0 << 0) );		// Full Speed


  // HMC5883 Reading:
  // a) write hmc-register to HMC
  // b) read 6 bytes from HMC

  mpu_set( MPU60X0_REG_I2C_SLV0_ADDR, (HMC58XX_ADDR >> 1) | MPU60X0_SPI_READ);
  mpu_set( MPU60X0_REG_I2C_SLV0_REG,  HMC58XX_REG_DATXM);
  // Put the enable command as last.
  mpu_set( MPU60X0_REG_I2C_SLV0_CTRL,
           (1 << 7) |		// Slave 0 enable
           (0 << 6) |		// Byte Swap
           (6 << 0) );		// Read 6 bytes

	// Slave 0 Control:


#endif

}

