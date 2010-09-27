
/** \file ir_mlx.c
 *  \brief Melexis 90614 I2C
 *
 *   This reads the values for temperatures from the Melexis 90614 IR sensor through I2C.
 */


#include "ir_mlx.h"

#include "sys_time.h"
#include "i2c.h"
#include "led.h"
#include "ap_downlink.h"
#include "settings.h"
#include "cam.h"
#include "link_mcu.h"
#include "sys_time.h"
#include "flight_plan.h"
#include "datalink.h"
#include "xbee.h"
#include "gpio.h"
#include "light.h"

#include <string.h>

uint8_t  ir_mlx_status;
uint16_t ir_mlx_itemp_case;
int32_t  ir_mlx_temp_case;
uint16_t ir_mlx_itemp_obj;
int32_t  ir_mlx_temp_obj;
bool_t   ir_mlx_available;
volatile bool_t ir_mlx_i2c_done;

#define MLX90614_ADDR 0x06
#define MLX90614_TA   0x06
#define MLX90614_TOBJ 0x07


//    printf("Ta    = %2.2f°C (0x%04X)\n", (tp*0.02)-273.15, tp);
//    printf("Tobj1 = %2.2f°C (0x%04X)\n", (tp*0.02)-273.15, tp);


void ir_mlx_init( void ) {
  ir_mlx_status = IR_MLX_UNINIT;
  ir_mlx_i2c_done = FALSE;
  ir_mlx_available = FALSE;
}

void ir_mlx_periodic( void ) {
  if (ir_mlx_status == IR_MLX_UNINIT && cpu_time_sec > 1) {
    ir_mlx_status = IR_MLX_RD_CASE_TEMP;
  } 

  else if (ir_mlx_status == IR_MLX_RD_CASE_TEMP) {

    /* start two byte obj temperature */
    i2c0_buf[0] = MLX90614_TOBJ;
    ir_mlx_status = IR_MLX_RD_OBJ_TEMP;
    i2c0_transceive(MLX90614_ADDR, 1, 2, &ir_mlx_i2c_done);
  }

  else if (ir_mlx_status == IR_MLX_RD_OBJ_TEMP) {

    /* start two byte case temperature */
    i2c0_buf[0] = MLX90614_TA;
    ir_mlx_status = IR_MLX_RD_CASE_TEMP;
    i2c0_transceive(MLX90614_ADDR, 1, 2, &ir_mlx_i2c_done);
  }
}

void ir_mlx_event( void ) {
  if (ir_mlx_i2c_done == TRUE) {
    ir_mlx_i2c_done = FALSE;
    if (ir_mlx_status == IR_MLX_RD_CASE_TEMP) {

      /* read two byte case temperature */
      ir_mlx_itemp_case  = i2c0_buf[1] << 8;
      ir_mlx_itemp_case |= i2c0_buf[0];
      ir_mlx_temp_case = ir_mlx_itemp_case*2 - 27315;
    }

    else if (ir_mlx_status == IR_MLX_RD_OBJ_TEMP) {

      /* read two byte case temperature */
      ir_mlx_itemp_obj  = i2c0_buf[1] << 8;
      ir_mlx_itemp_obj |= i2c0_buf[0];
      ir_mlx_temp_obj = ir_mlx_itemp_obj*2 - 27315;
      ir_mlx_available = TRUE;
    }
  }
}

