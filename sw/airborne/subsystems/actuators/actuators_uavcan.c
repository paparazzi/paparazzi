/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file subsystems/actuators/actuators_uavcan.c
 * UAVCan actuators using RAWCOMMAND message and ESC_STATUS telemetry
 * 
 */

#include "actuators_uavcan.h"
#include "subsystems/electrical.h"

/* uavcan ESC status telemetry structure */
struct actuators_uavcan_telem_t {
  float voltage;
  float current;
  float temperature;
  int32_t rpm;
  uint32_t energy;
};

#ifdef SERVOS_UAVCAN1_NB
int16_t actuators_uavcan1_values[SERVOS_UAVCAN1_NB];
static struct actuators_uavcan_telem_t uavcan1_telem[SERVOS_UAVCAN1_NB];
#endif
#ifdef SERVOS_UAVCAN2_NB
int16_t actuators_uavcan2_values[SERVOS_UAVCAN2_NB];
static struct actuators_uavcan_telem_t uavcan2_telem[SERVOS_UAVCAN2_NB];
#endif

/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_ESC_STATUS_ID                     1034
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE              (0xA9AF28AEA2FBB254ULL)
#define UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE               ((110 + 7)/8)

/* uavcan EQUIPMENT_ESC_RAWCOMMAND message definition */
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID                 1030
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE          (0x217F5C87D7EC951DULL)
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE           ((285 + 7)/8)

static bool actuators_uavcan_initialized = false;
static uavcan_event esc_status_ev;


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void actuators_uavcan_send_esc(struct transport_tx *trans, struct link_device *dev)
{
  /*static uint8_t esc_idx = 0;
  float power = uavcan_telem[esc_idx].current * uavcan_telem[esc_idx].voltage;
  float rpm = uavcan_telem[esc_idx].rpm;
  float energy = uavcan_telem[esc_idx].energy;
  pprz_msg_send_ESC(trans, dev, AC_ID, &uavcan_telem[esc_idx].current, &electrical.vsupply, &power,
                                        &rpm, &uavcan_telem[esc_idx].voltage, &energy, &esc_idx);
  esc_idx++;

  if(esc_idx >= ACTUATORS_UAVCAN_NB)
    esc_idx = 0;*/
}
#endif

/**
 * Whevener an ESC_STATUS message from the EQUIPMENT group is received
 */
static void actuators_uavcan_esc_status_cb(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
  /*uint8_t esc_idx;
  uint16_t tmp_float;

  canardDecodeScalar(transfer, 105, 5, false, (void*)&esc_idx);
  if(iface == &can2_iface)
    esc_idx += 10;
  if(esc_idx >= ACTUATORS_UAVCAN_NB)
    break;
  
  canardDecodeScalar(transfer, 0, 32, false, (void*)&uavcan_telem[esc_idx].energy);
  canardDecodeScalar(transfer, 32, 16, true, (void*)&tmp_float);
  uavcan_telem[esc_idx].voltage = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, 48, 16, true, (void*)&tmp_float);
  uavcan_telem[esc_idx].current = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, 64, 16, true, (void*)&tmp_float);
  uavcan_telem[esc_idx].temperature = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, 80, 18, true, (void*)&uavcan_telem[esc_idx].rpm);

  // Update total current
  electrical.current = 0;
  for(uint8_t i = 0; i < ACTUATORS_UAVCAN_NB; ++i)
    electrical.current += uavcan_telem[i].current;*/
}

/**
 * Initialize an uavcan interface
 */
void actuators_uavcan_init(struct uavcan_iface_t* iface __attribute__((unused)))
{
  // Check if not already initialized (for multiple interfaces, needs only 1)
  if(actuators_uavcan_initialized) return;

  // Bind uavcan ESC_STATUS message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_ESC_STATUS_ID, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE, &esc_status_ev, &actuators_uavcan_esc_status_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, actuators_uavcan_send_esc);
#endif

  // Set initialization
  actuators_uavcan_initialized = true;
}

/**
 * Commit actuator values to the uavcan interface
 */
void actuators_uavcan_commit(struct uavcan_iface_t* iface, int16_t *values, uint8_t nb)
{ 
  uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
  uint32_t offset = 0;

  // Encode the values as 14-bit signed integers
  for(uint8_t i = 0; i < nb; i++) {
    canardEncodeScalar(buffer, offset, 14, (void *)&values[i]);
    offset += 14;
  }

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID,
                      CANARD_TRANSFER_PRIORITY_HIGH, buffer, (offset+7)/8);
}
