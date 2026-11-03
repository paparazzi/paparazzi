/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file sw/tools/ekf2_replay/ekf2_replay.cpp
 *
 * Replay the EKF2 of ins_ekf2 on the inputs logged with INS_EKF2_LOG_INPUTS
 * (EKF2_INPUT messages in a PPRZLOG file: the .LOG of the SD card or the .tlm
 * copy of sd2log), with the same EKF code (sw/ext/px4_ekf2). The parameters are
 * the logged ones, they can be changed with --set NAME=VALUE (PX4 name without
 * EKF2_, e.g. MAG_NOISE). The estimate is written to a CSV file, and compared
 * with the logged EKF output.
 */

#define INS_EKF2_PARAM_TABLE
#include "EKF/ekf.h"
#include "generated/ekf2_params.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace estimator;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Quatf;
using matrix::Eulerf;

/** EKF2_INPUT message and its types, as logged by ins_ekf2.cpp */
static const uint8_t EKF2_INPUT_ID = 195;
enum ekf2_input_type {
  EKF2_IN_IMU, EKF2_IN_UPDATE, EKF2_IN_MAG, EKF2_IN_BARO, EKF2_IN_GNSS, EKF2_IN_GNSS_YAW,
  EKF2_IN_AIRSPEED, EKF2_IN_RANGE, EKF2_IN_FLOW, EKF2_IN_EV, EKF2_IN_FLAGS, EKF2_IN_ORIGIN,
  EKF2_IN_PARAMS, EKF2_IN_CONFIG, EKF2_IN_OUTPUT
};

struct input_t {
  uint64_t time_us;
  uint8_t type;
  std::vector<int32_t> ints;
  std::vector<float> values;
};

/** Read the EKF2_INPUT messages of a PPRZLOG file */
static bool read_inputs(const char *path, std::vector<input_t> &inputs, int &ac_id)
{
  FILE *f = fopen(path, "rb");
  if (!f) {
    perror(path);
    return false;
  }
  uint64_t ref = 0;
  bool have_ref = false;
  int c;
  while ((c = fgetc(f)) != EOF) {
    if (c != 0x99) {
      continue;  // STX of a PPRZLOG frame
    }
    uint8_t head[6];  // length, source, timestamp
    if (fread(head, 1, 6, f) != 6) {
      break;
    }
    uint8_t len = head[0];
    uint8_t payload[256];
    uint8_t ck = 0;
    if (fread(payload, 1, len + 1, f) != (size_t)len + 1) {
      break;
    }
    for (int i = 0; i < 6; i++) { ck += head[i]; }
    for (int i = 0; i < len; i++) { ck += payload[i]; }
    if (ck != payload[len]) {
      fseek(f, -(long)(len + 1 + 6), SEEK_CUR);  // not a frame, continue after this STX
      continue;
    }
    // pprz payload: sender, receiver, class/component, message id, fields
    if (head[1] != 0 || len < 4 + 6 || payload[3] != EKF2_INPUT_ID) {
      continue;
    }
    if (ac_id < 0) {
      ac_id = payload[0];
    } else if (payload[0] != ac_id) {
      continue;
    }
    const uint8_t *p = payload + 4;
    const uint8_t *end = payload + len;
    input_t in;
    uint32_t t32;
    memcpy(&t32, p, 4); p += 4;
    in.type = *p++;
    uint8_t nb = *p++;
    if (p + nb * 4 > end) { continue; }
    in.ints.resize(nb);
    memcpy(in.ints.data(), p, nb * 4); p += nb * 4;
    if (p >= end) { continue; }
    nb = *p++;
    if (p + nb * 4 > end) { continue; }
    in.values.resize(nb);
    memcpy(in.values.data(), p, nb * 4);
    // 64 bit time from the lower 32 bits (samples can be older than the previous ones)
    if (!have_ref) {
      ref = t32;
      have_ref = true;
    }
    in.time_us = (uint64_t)((int64_t)ref + (int32_t)(t32 - (uint32_t)ref));
    if (in.time_us > ref) {
      ref = in.time_us;
    }
    inputs.push_back(in);
  }
  fclose(f);
  return true;
}

static int param_index(const char *name)
{
  for (int i = 0; i < ins_ekf2_params_nb; i++) {
    if (!strcmp(ins_ekf2_params[i].name, name)) {
      return i;
    }
  }
  return -1;
}

static float finite_or_zero(float v) { return std::isfinite(v) ? v : 0.f; }

static void usage(const char *prog)
{
  fprintf(stderr,
          "Usage: %s <log with EKF2_INPUT (.LOG or .tlm)> [options]\n"
          "  -o <file.csv>        write the estimate (default: <log>.ekf2.csv)\n"
          "  --set NAME=VALUE     change an EKF parameter (PX4 name without EKF2_, e.g. MAG_NOISE=0.08)\n"
          "  --every N            write every N-th update (default 10)\n"
          "  --list               list the parameters with their logged values and exit\n", prog);
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }
  const char *input = argv[1];
  std::string out_path = std::string(input) + ".ekf2.csv";
  std::map<int, float> overrides;
  int every = 10;
  bool list = false;
  for (int i = 2; i < argc; i++) {
    if (!strcmp(argv[i], "-o") && i + 1 < argc) {
      out_path = argv[++i];
    } else if (!strcmp(argv[i], "--every") && i + 1 < argc) {
      every = atoi(argv[++i]);
      if (every < 1) { every = 1; }
    } else if (!strcmp(argv[i], "--list")) {
      list = true;
    } else if (!strcmp(argv[i], "--set") && i + 1 < argc) {
      std::string kv = argv[++i];
      size_t eq = kv.find('=');
      std::string name = kv.substr(0, eq);
      if (name.rfind("EKF2_", 0) == 0) { name = name.substr(5); }
      int idx = eq == std::string::npos ? -1 : param_index(name.c_str());
      if (idx < 0) {
        fprintf(stderr, "Unknown parameter in '%s' (see --list)\n", argv[i]);
        return 1;
      }
      overrides[idx] = (float)atof(kv.substr(eq + 1).c_str());
    } else {
      usage(argv[0]);
      return 1;
    }
  }

  std::vector<input_t> inputs;
  int ac_id = -1;
  if (!read_inputs(input, inputs, ac_id) || inputs.empty()) {
    fprintf(stderr, "No EKF2_INPUT messages in %s (logged with INS_EKF2_LOG_INPUTS?)\n", input);
    return 1;
  }

  // Logged parameters (header before the first sample)
  static Ekf ekf;
  parameters *params = ekf.getParamHandle();
  std::vector<float> logged(ins_ekf2_params_nb, NAN);
  for (const input_t &in : inputs) {
    if (in.type == EKF2_IN_CONFIG) { break; }
    if (in.type == EKF2_IN_PARAMS && !in.ints.empty()) {
      for (size_t j = 0; j < in.values.size() && in.ints[0] + j < logged.size(); j++) {
        logged[in.ints[0] + j] = in.values[j];
      }
    }
  }
  if (list) {
    for (int i = 0; i < ins_ekf2_params_nb; i++) {
      printf("%-16s %g\n", ins_ekf2_params[i].name, logged[i]);
    }
    return 0;
  }

  FILE *out = fopen(out_path.c_str(), "w");
  if (!out) {
    perror(out_path.c_str());
    return 1;
  }
  fprintf(out, "time,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,roll,pitch,yaw,"
          "pos_acc_h,pos_acc_v,vel_acc_h,vel_acc_v,control_status,fault_status,"
          "ratio_heading,ratio_vel,ratio_pos,ratio_hgt,ratio_tas,ratio_hagl,ratio_beta,"
          "wind_n,wind_e,gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
          "mag_bias_x,mag_bias_y,mag_bias_z,hagl,baro_bias\n");

  bool configured = false;
  uint64_t t0 = inputs.front().time_us;
  unsigned long updates = 0, outputs = 0;
  float max_pos_diff = 0, max_vel_diff = 0, max_att_diff = 0;

  for (const input_t &in : inputs) {
    const std::vector<int32_t> &I = in.ints;
    const std::vector<float> &V = in.values;
    switch (in.type) {
      case EKF2_IN_PARAMS:
        // at the start all parameters, later the ones changed while running
        if (configured && !I.empty()) {
          for (size_t j = 0; j < V.size(); j++) {
            int idx = I[0] + j;
            if (idx < ins_ekf2_params_nb && !overrides.count(idx)) {
              ins_ekf2_params[idx].set(*params, V[j]);
            }
          }
        }
        break;
      case EKF2_IN_CONFIG: {
        // configure the EKF as ins_ekf2_init()
        for (int i = 0; i < ins_ekf2_params_nb; i++) {
          if (std::isfinite(logged[i])) { ins_ekf2_params[i].set(*params, logged[i]); }
        }
        for (auto &o : overrides) { ins_ekf2_params[o.first].set(*params, o.second); }
        ekf.output_predictor().set_imu_offset(params->imu_pos_body);
        ekf.output_predictor().set_pos_correction_tc(V[0]);
        ekf.output_predictor().set_vel_correction_tc(V[1]);
        ekf.set_gnss_yaw_reset_wait(V[2] * 1000000);
        FusionControl *fc = ekf.getFusionControlHandle();
        fc->gps.enabled = true;
        fc->of.enabled = true;
        fc->ev.enabled = true;
        fc->baro.enabled = true;
        fc->rng.enabled = true;
        fc->mag.enabled = true;
        fc->aspd.enabled = true;
        ekf.updateParameters();
        ekf.set_rangefinder_limits(V[3], V[4]);
        ekf.set_optical_flow_limits(V[5], V[3], V[4]);
        configured = true;
        break;
      }
      case EKF2_IN_ORIGIN:
        ekf.setEkfGlobalOrigin(I[0] * 1e-7, I[1] * 1e-7, V[0]);
        break;
      case EKF2_IN_IMU: {
        imuSample s{};
        s.time_us = in.time_us;
        s.delta_ang_dt = (uint32_t)I[0] * 1.e-6f;
        s.delta_ang = Vector3f{V[0], V[1], V[2]};
        s.delta_vel_dt = (uint32_t)I[1] * 1.e-6f;
        s.delta_vel = Vector3f{V[3], V[4], V[5]};
        ekf.setIMUData(s);
        break;
      }
      case EKF2_IN_UPDATE:
        ekf.update();
        ekf.resetVelocityDerivativeAccumulation();
        if (++updates % every == 0) {
          const Vector3f pos = ekf.getPosition();
          const Vector3f vel = ekf.getVelocity();
          const Eulerf e{Quatf{ekf.getQuaternion()}};
          float eph, epv, evh, evv;
          ekf.get_ekf_gpos_accuracy(&eph, &epv);
          ekf.get_ekf_vel_accuracy(&evh, &evv);
          const Vector2f wind = ekf.getWindVelocity();
          const Vector3f gb = ekf.getGyroBias(), ab = ekf.getAccelBias(), mb = ekf.getMagBias();
          fprintf(out, "%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,0x%" PRIx64 ",0x%x,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f\n",
                  (in.time_us - t0) * 1e-6, pos(0), pos(1), pos(2), vel(0), vel(1), vel(2),
                  e.phi() * 180 / M_PI, e.theta() * 180 / M_PI, e.psi() * 180 / M_PI, eph, epv, evh, evv,
                  ekf.control_status().value, (unsigned)ekf.fault_status().value,
                  finite_or_zero(ekf.getHeadingInnovationTestRatio()),
                  finite_or_zero(fmaxf(ekf.getHorizontalVelocityInnovationTestRatio(), ekf.getVerticalVelocityInnovationTestRatio())),
                  finite_or_zero(ekf.getHorizontalPositionInnovationTestRatio()),
                  finite_or_zero(ekf.getVerticalPositionInnovationTestRatio()),
                  finite_or_zero(ekf.getAirspeedInnovationTestRatio()),
                  finite_or_zero(ekf.getHeightAboveGroundInnovationTestRatio()),
                  finite_or_zero(ekf.getSyntheticSideslipInnovationTestRatio()),
                  wind(0), wind(1), gb(0), gb(1), gb(2), ab(0), ab(1), ab(2), mb(0), mb(1), mb(2),
                  ekf.isTerrainEstimateValid() ? ekf.getHagl() : NAN, ekf.getBaroBiasEstimatorStatus().bias);
        }
        break;
      case EKF2_IN_OUTPUT: {
        // the logged estimate right after the previous update
        const Vector3f pos = ekf.getPosition();
        const Vector3f vel = ekf.getVelocity();
        const Quatf q = ekf.getQuaternion();
        for (int j = 0; j < 3; j++) {
          max_pos_diff = fmaxf(max_pos_diff, fabsf(pos(j) - V[j]));
          max_vel_diff = fmaxf(max_vel_diff, fabsf(vel(j) - V[3 + j]));
        }
        // angle of the rotation between both (from the vector part, precise for small angles)
        const Quatf q_log{V[6], V[7], V[8], V[9]};
        const Quatf q_err = q_log.inversed() * q;
        const float angle = 2.f * asinf(fminf(Vector3f{q_err(1), q_err(2), q_err(3)}.norm(), 1.f));
        max_att_diff = fmaxf(max_att_diff, angle * 180.f / M_PI);
        outputs++;
        break;
      }
      case EKF2_IN_MAG: {
        magSample s{};
        s.time_us = in.time_us;
        s.mag = Vector3f{V[0], V[1], V[2]};
        ekf.setMagData(s);
        break;
      }
      case EKF2_IN_BARO: {
        ekf.set_air_density(V[1]);
        baroSample s{};
        s.time_us = in.time_us;
        s.hgt = V[0];
        ekf.setBaroData(s);
        break;
      }
      case EKF2_IN_GNSS: {
        gnssSample s{};
        s.time_us = in.time_us;
        s.lat = I[0] * 1e-7;
        s.lon = I[1] * 1e-7;
        s.fix_type = I[2];
        s.nsats = I[3];
        s.usable = I[4] & 1;
        s.sacc_passes_strict = I[4] & 2;
        s.spoofed = I[4] & 4;
        s.jammed = I[4] & 8;
        s.alt = V[0];
        s.vel = Vector3f{V[1], V[2], V[3]};
        s.hacc = V[4];
        s.vacc = V[5];
        s.sacc = V[6];
        s.pdop = V[7];
        s.pos_body = Vector3f{V[8], V[9], V[10]};
        ekf.setGpsData(s);
        break;
      }
      case EKF2_IN_GNSS_YAW: {
        gnssYawSample s{};
        s.time_us = in.time_us;
        s.usable = I[0];
        s.yaw = V[0];
        s.yaw_acc = V[1];
        s.yaw_offset = V[2];
        ekf.setGnssYawData(s);
        break;
      }
      case EKF2_IN_AIRSPEED: {
        airspeedSample s{};
        s.time_us = in.time_us;
        s.true_airspeed = V[0];
        s.eas2tas = V[1];
        ekf.setAirspeedData(s);
        break;
      }
      case EKF2_IN_RANGE: {
        sensor::rangeSample s{};
        s.time_us = in.time_us;
        s.quality = I[0];
        s.rng = V[0];
        ekf.setRangeData(s);
        break;
      }
      case EKF2_IN_FLOW: {
        flowSample s{};
        s.time_us = in.time_us;
        s.quality = I[0];
        s.flow_rate = Vector2f{V[0], V[1]};
        s.gyro_rate = Vector3f{V[2], V[3], V[4]};
        ekf.setOpticalFlowData(s);
        break;
      }
      case EKF2_IN_EV: {
        extVisionSample s{};
        s.time_us = in.time_us;
        s.pos_frame = (PositionFrame)I[0];
        s.vel_frame = (VelocityFrame)I[1];
        s.quality = I[2];
        s.reset_counter = I[3];
        s.pos = Vector3f{V[0], V[1], V[2]};
        s.vel = Vector3f{V[3], V[4], V[5]};
        s.quat = Quatf{V[6], V[7], V[8], V[9]};
        s.position_var = Vector3f{V[10], V[11], V[12]};
        s.velocity_var = Vector3f{V[13], V[14], V[15]};
        s.orientation_var = Vector3f{V[16], V[17], V[18]};
        ekf.setExtVisionData(s);
        break;
      }
      case EKF2_IN_FLAGS: {
        systemFlagUpdate s{};
        s.time_us = in.time_us;
        s.armed = I[0] & 1;
        s.at_rest = I[0] & 2;
        s.in_air = I[0] & 4;
        s.is_fixed_wing = I[0] & 8;
        s.gnd_effect = I[0] & 16;
        s.constant_pos = I[0] & 32;
        s.in_transition = I[0] & 64;
        ekf.setSystemFlagData(s);
        break;
      }
      default:
        break;
    }
  }
  fclose(out);

  printf("A/C %d: %zu inputs, %lu updates, %.1f s, estimate in %s\n", ac_id, inputs.size(), updates,
         (inputs.back().time_us - t0) * 1e-6, out_path.c_str());
  if (!configured) {
    printf("Warning: no parameters in the log (the log started after the EKF?)\n");
  }
  if (outputs > 0) {
    printf("Compared with the logged estimate (%lu times)%s: max position %.6f m, velocity %.6f m/s, attitude %.6f deg\n",
           outputs, overrides.empty() ? "" : " (with changed parameters)", max_pos_diff, max_vel_diff, max_att_diff);
  }
  return 0;
}
