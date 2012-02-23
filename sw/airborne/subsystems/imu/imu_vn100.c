#include "subsystems/imu.h"

#include "led.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/spi_arch.h"

// Peripherials
#include "../../peripherals/VN100.h"

struct spi_transaction vn100_spi;
volatile uint8_t vn100_spi_complete;
uint8_t imu_vn100_available;
/* last received SPI packet */
VN100_Res_Packet last_received_packet;
/* last send packet */
VN100_Req_Packet last_send_packet;

void vn100_parse_msg(void);

// initialize peripherals
static void configure(void);

void imu_impl_init(void) {
  vn100_spi_complete = 0;
  imu_vn100_available = 0;
}


void imu_periodic(void) 
{
  if (0)
  {
    configure();
  }
  else
  {
    last_send_packet.CmdID = VN100_CmdID_ReadRegister;
    last_send_packet.RegID = VN100_REG_CALMEAS;
    vn100_spi.length = 4 + VN100_REG_CALMEAS_SIZE;
    vn100_spi.mosi_buf = (uint8_t*)&last_send_packet;
    vn100_spi.miso_buf = (uint8_t*)&last_received_packet;
    vn100_spi.ready = (uint8_t*)&vn100_spi_complete;
    vn100_spi.slave_idx = 0;
    spi_submit(&spi2,&vn100_spi);
  }
}

void imu_event_task(void) {
  if (vn100_spi_complete) {
    vn100_parse_msg();
    vn100_spi_complete = 0;
  }
}

static void configure(void) 
{
}

void vn100_parse_msg(void)
{
  if (last_received_packet.ErrID != VN100_Error_None) {
    //TODO send error
    return;
  }
  switch (last_received_packet.RegID) {
    case VN100_REG_CALMEAS :
      imu.gyro_unscaled.p=RATE_BFP_OF_REAL(last_received_packet.Data[6].Float);
      imu.gyro_unscaled.q=RATE_BFP_OF_REAL(last_received_packet.Data[7].Float);
      imu.gyro_unscaled.r=RATE_BFP_OF_REAL(last_received_packet.Data[8].Float);
      imu.accel_unscaled.x=ACCEL_BFP_OF_REAL(last_received_packet.Data[3].Float);
      imu.accel_unscaled.y=ACCEL_BFP_OF_REAL(last_received_packet.Data[4].Float);
      imu.accel_unscaled.z=ACCEL_BFP_OF_REAL(last_received_packet.Data[5].Float);
      imu.mag_unscaled.x=MAG_BFP_OF_REAL(last_received_packet.Data[0].Float);
      imu.mag_unscaled.y=MAG_BFP_OF_REAL(last_received_packet.Data[1].Float);
      imu.mag_unscaled.z=MAG_BFP_OF_REAL(last_received_packet.Data[2].Float);
      imu_vn100_available = 1;
      break;
    default:
      break;
  }
}

