/*
 * Copyright (C) 2024 Fabien-B <name.surname@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/lidar/lidar_vl53l5cx.c"
 * @author Fabien-B <name.surname@gmail.com>
 * VL53L5CX multizone range sensor.
 */

#include "modules/lidar/lidar_vl53l5cx.h"
#include "mcu_periph/i2c.h"
#include "ch.h"
#include "vl53l5cx_platform.h"
#include "vl53l5cx_api.h"
#include "mcu_periph/ram_arch.h"
#include "modules/core/abi.h"
#include "modules/datalink/downlink.h"

#ifndef LIDAR_VL53L5CX_I2C_ADDR
#define LIDAR_VL53L5CX_I2C_ADDR 0x29
#endif

#define SUBTYPE_DISTANCE 0

static IN_DMA_SECTION(VL53L5CX_Configuration 	Dev);
static IN_DMA_SECTION(VL53L5CX_ResultsData 	Results);

static abi_event lidar_ev;


static THD_WORKING_AREA(wa_thd_lidar_vl53l5cx, 1024);
static void thd_lidar_vl53l5cx(void* arg);


char* VL53L5CX_ERROR_MSGS[] = {
  "VL53L5CX_NO_ERROR",
  "VL53L5CX_NOT_DETECTED",
  "VL53L5CX_ULD_LOADING_FAILED",
  "VL53L5CX_SET_RESOLUTION_FAILED",
  "VL53L5CX_RUNTIME_ERROR",
};

static void lidar_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     uint32_t numRows, uint32_t numCols, uint16_t size,
                     uint8_t subtype, uint8_t* data) {
    (void)numRows;
    (void)numCols;
    (void)size;
    (void)subtype;
    (void)data;
    // example
    // float f[16];
    // uint16_t* distances_mm = (uint16_t*) data;
    // for(int i=0; i<16; i++) {
    //   f[i] = distances_mm[i];
    // }
    // DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 16, f);
}


void lidar_vl53l5cx_init(void)
{
  Dev.platform.i2cdev = &LIDAR_VL53L5CX_I2C_DEV;
  Dev.platform.address = LIDAR_VL53L5CX_I2C_ADDR;
  Dev.platform.thread_handle = NULL;
  Dev.platform.error_code = VL53L5CX_NO_ERROR;

  AbiBindMsgLIDAR_DATA(ABI_BROADCAST, &lidar_ev, lidar_cb);

  // Create thread
  Dev.platform.thread_handle = chThdCreateStatic(wa_thd_lidar_vl53l5cx, sizeof(wa_thd_lidar_vl53l5cx),
                    NORMALPRIO, thd_lidar_vl53l5cx, (void *)&Dev);
}

void lidar_vl53l5cx_periodic(void)
{

  if(Dev.platform.thread_handle != NULL) {
    // check thread status
    if(Dev.platform.thread_handle->state == CH_STATE_FINAL) {
      Dev.platform.error_code = (enum VL53L5CX_ERRORS) chThdWait(Dev.platform.thread_handle);
      Dev.platform.thread_handle = NULL;
    }
  }
  else {
    
    // thread exited, send error code periodically
    size_t len = strlen(VL53L5CX_ERROR_MSGS[Dev.platform.error_code]);
    // send exitcode to telemetry
    RunOnceEvery(10, DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, len, VL53L5CX_ERROR_MSGS[Dev.platform.error_code]));
    ;
  }


  if(Dev.platform.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgLIDAR_DATA(LIDAR_DATA_VL53L5CX_ID, now_ts,
      8, 8, sizeof(Dev.platform.distances_mm[0]), SUBTYPE_DISTANCE,
      (uint8_t*)Dev.platform.distances_mm);
      
    Dev.platform.data_available = false;
  }


}



static void thd_lidar_vl53l5cx(void* arg) {
  chRegSetThreadName("vl53l5cx");
  VL53L5CX_Configuration* dev = (VL53L5CX_Configuration*) arg;

  uint8_t 				status, isAlive, isReady;


  chThdSleepMilliseconds(2000);

  status = vl53l5cx_is_alive(dev, &isAlive);
  if(!isAlive || status)
  {
    chThdExit(VL53L5CX_NOT_DETECTED);
  }

  status = vl53l5cx_init(dev);
  if(status) {
    chThdExit(VL53L5CX_ULD_LOADING_FAILED);
  }

  status = vl53l5cx_set_resolution(dev, VL53L5CX_RESOLUTION_4X4);
  if(status) {
    chThdExit(VL53L5CX_SET_RESOLUTION_FAILED);
  }

  status = vl53l5cx_start_ranging(dev);

  while(true) {
    status = vl53l5cx_check_data_ready(dev, &isReady);
    if(isReady) {
      status = vl53l5cx_get_ranging_data(dev, &Results);
      if(status == 0) {
        memcpy(dev->platform.distances_mm, Results.distance_mm, 64*sizeof(Results.distance_mm[0]));
        dev->platform.data_available = true;
      }
    }

    chThdSleepMilliseconds(100);
  }


}

