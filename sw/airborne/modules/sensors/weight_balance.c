/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/weight_balance.c
 * Weight and center of gravity from load cells in the landing gear
 *
 * Load cells are received over uavcan (pprz.equipment.loadcell.Measurement)
 * and optionally from one ADS1220 directly on an SPI bus of the autopilot.
 * Every wheel is mapped to a load cell id. From the vertical wheel forces the
 * total mass and the CG position in the body frame are computed, corrected for
 * the pitch and roll of the aircraft when the CG height is known.
 */

#include "modules/sensors/weight_balance.h"
#include "generated/airframe.h"
#include "modules/uavcan/uavcan.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include <math.h>
#include "pprz.equipment.loadcell.Measurement.h"

#ifndef WEIGHT_BALANCE_NB_WHEELS
#define WEIGHT_BALANCE_NB_WHEELS 3
#endif
#if WEIGHT_BALANCE_NB_WHEELS > WEIGHT_BALANCE_MAX_WHEELS
#error "WEIGHT_BALANCE_NB_WHEELS is larger than WEIGHT_BALANCE_MAX_WHEELS"
#endif

/** Load cell id per wheel */
#ifndef WEIGHT_BALANCE_WHEEL_IDS
#define WEIGHT_BALANCE_WHEEL_IDS {0, 1, 2}
#endif

/** Wheel contact points in body frame [m] (forward, right) */
#ifndef WEIGHT_BALANCE_WHEEL_X
#define WEIGHT_BALANCE_WHEEL_X {0, 0, 0}
#endif
#ifndef WEIGHT_BALANCE_WHEEL_Y
#define WEIGHT_BALANCE_WHEEL_Y {0, 0, 0}
#endif

/** Load cell calibration per wheel: force = (raw - offset) * scale */
#ifndef WEIGHT_BALANCE_WHEEL_OFFSET
#define WEIGHT_BALANCE_WHEEL_OFFSET {0, 0, 0}
#endif
#ifndef WEIGHT_BALANCE_WHEEL_SCALE
#define WEIGHT_BALANCE_WHEEL_SCALE {0, 0, 0}
#endif

/** Height of the CG above the wheel contact plane, for attitude correction [m] */
#ifndef WEIGHT_BALANCE_CG_HEIGHT
#define WEIGHT_BALANCE_CG_HEIGHT 0
#endif

/** Maximum age of a load cell reading [s] */
#ifndef WEIGHT_BALANCE_TIMEOUT
#define WEIGHT_BALANCE_TIMEOUT 0.5f
#endif

/** Minimum total force to compute the CG [N] */
#ifndef WEIGHT_BALANCE_MIN_FORCE
#define WEIGHT_BALANCE_MIN_FORCE 5.0f
#endif

struct weight_balance_t weight_balance;

static void weight_balance_set_raw(uint8_t cell_id, float raw)
{
  if (isnan(raw)) {
    return;
  }
  float now = get_sys_time_float();
  for (uint8_t i = 0; i < weight_balance.nb_wheels; i++) {
    struct weight_balance_wheel_t *wheel = &weight_balance.wheels[i];
    if (wheel->cell_id == cell_id) {
      wheel->raw = raw;
      wheel->force = (raw - wheel->offset) * wheel->scale;
      wheel->last_time = now;
    }
  }
}

/*
 * Load cells over uavcan
 */
static uavcan_event weight_balance_uavcan_ev;

static void weight_balance_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  struct pprz_equipment_loadcell_Measurement msg;
  if (pprz_equipment_loadcell_Measurement_decode(transfer, &msg)) {
    return; // decode error
  }

  for (uint8_t i = 0; i < msg.raw.len; i++) {
    weight_balance_set_raw(msg.sensor_id + i, msg.raw.data[i]);
  }
}

/*
 * Load cell on a local ADS1220
 */
#ifdef WEIGHT_BALANCE_SPI_DEV
#include "peripherals/ads1220.h"

#ifndef WEIGHT_BALANCE_SPI_SLAVE_IDX
#error "WEIGHT_BALANCE_SPI_SLAVE_IDX needs to be defined with WEIGHT_BALANCE_SPI_DEV"
#endif

/** Load cell id of the local ADS1220 */
#ifndef WEIGHT_BALANCE_ADS1220_ID
#define WEIGHT_BALANCE_ADS1220_ID 0
#endif

/** Input of the bridge on the ADS1220 */
#ifndef WEIGHT_BALANCE_ADS1220_MUX
#define WEIGHT_BALANCE_ADS1220_MUX ADS1220_MUX_AIN0_AIN1
#endif

/** ADS1220 gain */
#ifndef WEIGHT_BALANCE_ADS1220_GAIN
#define WEIGHT_BALANCE_ADS1220_GAIN ADS1220_GAIN_128
#endif

/** -3dB cutoff of the 2nd order low-pass filter [Hz], 0 to disable */
#ifndef WEIGHT_BALANCE_ADS1220_CUTOFF
#define WEIGHT_BALANCE_ADS1220_CUTOFF 5.0f
#endif

static struct {
  struct Ads1220 ads;
  int32_t med_buf[3];
  uint8_t med_cnt;
  uint8_t med_idx;
  float lp1;
  float lp2;
  float tau;
  float last_time;
} wb_ads;

static int32_t median3(int32_t a, int32_t b, int32_t c)
{
  if ((a <= b && b <= c) || (c <= b && b <= a)) {
    return b;
  } else if ((b <= a && a <= c) || (c <= a && a <= b)) {
    return a;
  }
  return c;
}

static void weight_balance_ads1220_init(void)
{
  wb_ads.ads.config.mux = WEIGHT_BALANCE_ADS1220_MUX;
  wb_ads.ads.config.gain = WEIGHT_BALANCE_ADS1220_GAIN;
  wb_ads.ads.config.pga_bypass = false;
  wb_ads.ads.config.rate = ADS1220_RATE_175_HZ;
  wb_ads.ads.config.conv = ADS1220_CONTINIOUS_CONVERSION;
  wb_ads.ads.config.vref = ADS1220_VREF_VDD; // ratiometric to the bridge excitation
  wb_ads.ads.config.idac = ADS1220_IDAC_OFF;
  wb_ads.ads.config.i1mux = ADS1220_IMUX_OFF;
  wb_ads.ads.config.i2mux = ADS1220_IMUX_OFF;
  ads1220_init(&wb_ads.ads, &(WEIGHT_BALANCE_SPI_DEV), WEIGHT_BALANCE_SPI_SLAVE_IDX);

  // Two equal 1st order stages with a combined -3dB point at the cutoff
  if (WEIGHT_BALANCE_ADS1220_CUTOFF > 0) {
    float stage_cutoff = WEIGHT_BALANCE_ADS1220_CUTOFF / sqrtf(sqrtf(2.0f) - 1.0f);
    wb_ads.tau = 1.0f / (2.0f * M_PI * stage_cutoff);
  } else {
    wb_ads.tau = 0;
  }
  wb_ads.med_cnt = 0;
  wb_ads.med_idx = 0;
}

static void weight_balance_ads1220_event(void)
{
  ads1220_event(&wb_ads.ads);
  if (!wb_ads.ads.data_available) {
    return;
  }
  wb_ads.ads.data_available = false;

  // Sign extend the 24 bit reading
  uint32_t data = wb_ads.ads.data;
  if (data & 0x800000) {
    data |= 0xFF000000;
  }
  int32_t sample = (int32_t)data;

  // Median of the last 3 samples against single sample spikes
  wb_ads.med_buf[wb_ads.med_idx] = sample;
  wb_ads.med_idx = (wb_ads.med_idx + 1) % 3;
  if (wb_ads.med_cnt < 3) {
    wb_ads.med_cnt++;
  }
  float x = (wb_ads.med_cnt < 3) ? (float)sample : (float)median3(wb_ads.med_buf[0], wb_ads.med_buf[1], wb_ads.med_buf[2]);

  // 2nd order low-pass
  float now = get_sys_time_float();
  if (wb_ads.med_cnt == 1 || wb_ads.tau <= 0) {
    wb_ads.lp1 = x;
    wb_ads.lp2 = x;
  } else {
    float dt = now - wb_ads.last_time;
    float alpha = dt / (wb_ads.tau + dt);
    wb_ads.lp1 += alpha * (x - wb_ads.lp1);
    wb_ads.lp2 += alpha * (wb_ads.lp1 - wb_ads.lp2);
  }
  wb_ads.last_time = now;

  weight_balance_set_raw(WEIGHT_BALANCE_ADS1220_ID, wb_ads.lp2);
}
#endif /* WEIGHT_BALANCE_SPI_DEV */

/*
 * Telemetry
 */
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void weight_balance_send(struct transport_tx *trans, struct link_device *dev)
{
  // mass, cg_x, cg_y, valid, then the force and raw reading of every wheel
  float values[4 + 2 * WEIGHT_BALANCE_MAX_WHEELS];
  uint8_t nb = 0;
  values[nb++] = weight_balance.mass;
  values[nb++] = weight_balance.cg_x;
  values[nb++] = weight_balance.cg_y;
  values[nb++] = weight_balance.valid ? 1.f : 0.f;
  for (uint8_t i = 0; i < weight_balance.nb_wheels; i++) {
    values[nb++] = weight_balance.wheels[i].force;
    values[nb++] = weight_balance.wheels[i].raw;
  }
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, nb, values);
}
#endif

/*
 * Preflight checks
 */
#if PREFLIGHT_CHECKS
#include "modules/checks/preflight_checks.h"

static struct preflight_check_t weight_balance_pfc;

static void weight_balance_preflight(struct preflight_result_t *result)
{
  if (!weight_balance.valid) {
    preflight_error(result, "Weight and balance not available");
    return;
  }
#ifdef WEIGHT_BALANCE_MASS_MAX
  if (weight_balance.mass > WEIGHT_BALANCE_MASS_MAX) {
    preflight_error(result, "Mass %.1fkg above %.1fkg", weight_balance.mass, (float)WEIGHT_BALANCE_MASS_MAX);
    return;
  }
#endif
#if defined(WEIGHT_BALANCE_CG_X_MIN) && defined(WEIGHT_BALANCE_CG_X_MAX)
  if (weight_balance.cg_x < WEIGHT_BALANCE_CG_X_MIN || weight_balance.cg_x > WEIGHT_BALANCE_CG_X_MAX) {
    preflight_error(result, "CG x %.3fm outside [%.3f, %.3f]", weight_balance.cg_x,
                    (float)WEIGHT_BALANCE_CG_X_MIN, (float)WEIGHT_BALANCE_CG_X_MAX);
    return;
  }
#endif
#ifdef WEIGHT_BALANCE_CG_Y_MAX
  if (fabsf(weight_balance.cg_y) > WEIGHT_BALANCE_CG_Y_MAX) {
    preflight_error(result, "CG y %.3fm outside +-%.3f", weight_balance.cg_y, (float)WEIGHT_BALANCE_CG_Y_MAX);
    return;
  }
#endif
  preflight_success(result, "Mass %.1fkg, CG x %.3fm y %.3fm", weight_balance.mass, weight_balance.cg_x,
                    weight_balance.cg_y);
}
#endif

void weight_balance_init(void)
{
  const uint8_t ids[] = WEIGHT_BALANCE_WHEEL_IDS;
  const float x[] = WEIGHT_BALANCE_WHEEL_X;
  const float y[] = WEIGHT_BALANCE_WHEEL_Y;
  const float offset[] = WEIGHT_BALANCE_WHEEL_OFFSET;
  const float scale[] = WEIGHT_BALANCE_WHEEL_SCALE;

  weight_balance.nb_wheels = WEIGHT_BALANCE_NB_WHEELS;
  for (uint8_t i = 0; i < WEIGHT_BALANCE_NB_WHEELS; i++) {
    struct weight_balance_wheel_t *wheel = &weight_balance.wheels[i];
    wheel->cell_id = ids[i];
    wheel->x = x[i];
    wheel->y = y[i];
    wheel->offset = offset[i];
    wheel->scale = scale[i];
    wheel->raw = 0;
    wheel->force = 0;
    wheel->last_time = -1.f;
  }
  weight_balance.cg_height = WEIGHT_BALANCE_CG_HEIGHT;
  weight_balance.valid = false;
  weight_balance.mass = 0;
  weight_balance.cg_x = 0;
  weight_balance.cg_y = 0;
  weight_balance.cal_wheel = 0;

  uavcan_bind(PPRZ_EQUIPMENT_LOADCELL_MEASUREMENT_ID, PPRZ_EQUIPMENT_LOADCELL_MEASUREMENT_SIGNATURE,
              &weight_balance_uavcan_ev, &weight_balance_uavcan_cb);

#ifdef WEIGHT_BALANCE_SPI_DEV
  weight_balance_ads1220_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, weight_balance_send);
#endif

#if PREFLIGHT_CHECKS
  preflight_check_register(&weight_balance_pfc, weight_balance_preflight);
#endif
}

void weight_balance_periodic(void)
{
#ifdef WEIGHT_BALANCE_SPI_DEV
  ads1220_periodic(&wb_ads.ads);
#endif

  // Sum the forces and moments of all wheels
  float now = get_sys_time_float();
  bool valid = true;
  float force = 0, mom_x = 0, mom_y = 0;
  for (uint8_t i = 0; i < weight_balance.nb_wheels; i++) {
    struct weight_balance_wheel_t *wheel = &weight_balance.wheels[i];
    if (wheel->last_time < 0 || (now - wheel->last_time) > WEIGHT_BALANCE_TIMEOUT) {
      valid = false;
    }
    force += wheel->force;
    mom_x += wheel->force * wheel->x;
    mom_y += wheel->force * wheel->y;
  }
  if (force < WEIGHT_BALANCE_MIN_FORCE) {
    weight_balance.valid = false;
    return;
  }

  /* The load cells measure the force perpendicular to the wheel plane. With the
   * CG at height h above that plane and attitude (phi, theta):
   *   sum(F)      = m g cos(phi) cos(theta)
   *   sum(F x)/F  = cg_x - h tan(theta) / cos(phi)
   *   sum(F y)/F  = cg_y + h tan(phi)
   */
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float cphi = cosf(att->phi);
  float ctheta = cosf(att->theta);
  weight_balance.mass = force / (9.81f * cphi * ctheta);
  weight_balance.cg_x = mom_x / force + weight_balance.cg_height * tanf(att->theta) / cphi;
  weight_balance.cg_y = mom_y / force - weight_balance.cg_height * tanf(att->phi);
  weight_balance.valid = valid;
}

void weight_balance_event(void)
{
#ifdef WEIGHT_BALANCE_SPI_DEV
  weight_balance_ads1220_event();
#endif
}

/**
 * Set the offsets of all wheels to the current readings (wheels unloaded)
 */
void weight_balance_tare(float tare)
{
  if (tare < 0.5f) {
    return;
  }
  for (uint8_t i = 0; i < weight_balance.nb_wheels; i++) {
    weight_balance.wheels[i].offset = weight_balance.wheels[i].raw;
  }
}

/**
 * Set the scale of the selected wheel from a known mass on that wheel [kg]
 */
void weight_balance_calibrate(float mass)
{
  if (mass <= 0 || weight_balance.cal_wheel >= weight_balance.nb_wheels) {
    return;
  }
  struct weight_balance_wheel_t *wheel = &weight_balance.wheels[weight_balance.cal_wheel];
  float delta = wheel->raw - wheel->offset;
  if (fabsf(delta) > 1.f) {
    wheel->scale = mass * 9.81f / delta;
  }
}
