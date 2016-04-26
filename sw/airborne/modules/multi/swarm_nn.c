/*
 * Copyright (C) Kirk Scheper
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
/**
 * @file "modules/multi/swarm_nn.c"
 * @author Kirk Scheper
 * Neural network based swarming algorithm
 */

#include "modules/multi/swarm_nn.h"

#include "generated/airframe.h"             // AC_ID
#include "subsystems/datalink/downlink.h"   // DefaultChannel, DefaultDevice

#include "subsystems/gps.h"                 // gps
#include "state.h"                          // my position

#include "autopilot.h"                      // autopilot_guided_move_ned

#include "modules/multi/traffic_info.h"     // other aircraft info

#ifdef EXTRA_DOWNLINK_DEVICE
#include "modules/datalink/extra_pprz_dl.h"
#endif

float max_hor_speed;
float max_vert_speed;
uint8_t use_height;

#ifndef MAX_HOR_SPEED
#define MAX_HOR_SPEED 0.5
#endif

#ifndef MAX_VERT_SPEED
#define MAX_VERT_SPEED 0.5
#endif

#ifndef USE_HEIGHT
#define USE_HEIGHT 0
#endif

void swarm_nn_init(void)
{
  max_hor_speed = MAX_HOR_SPEED;
  max_vert_speed = MAX_VERT_SPEED;
  use_height = USE_HEIGHT;
}

// the following does not include the bias neuron
static const uint8_t nr_input_neurons = 6;
static const uint8_t nr_hidden_neurons = 8;
static const uint8_t nr_output_neurons = 2;

double input_layer_out[7]; //nr_input_neurons+1
double hidden_layer_out[9]; //nr_hidden_neurons+1
const double layer1_weights[7][8] = {
  { -0.249, 0.6456, 0.2576, -0.0324,  -0.1644,  0.7892, -0.313, -0.5042,},
  { 0.0736, 0.9582, 0.6542, 0.5952, -0.7244,  -0.367, -0.2034,  -0.3356,},
  { 0.1306, -0.6844,  0.8562, 0.1484, -0.4876,  -0.345, -0.611, -0.7448,},
  { 0.1306, -0.6844,  0.8562, 0.1484, -0.4876,  -0.345, -0.611, -0.7448,},
  { 0.039,  0.5906, 0.822,  0.8968, 0.2194, -0.8234,  -0.0694,  -0.3796,},
  { -0.044, -0.2248,  0.645,  -0.5442,  -0.5442,  -0.8088,  -0.67,  -0.9184,},
  { -0.142, -0.363, 2.412,  -1.586, 0.041,  -1.86,  0.443,  1.194,},
};
/* controid nn
{
{ 0.0066, 0.4742, 0.8316, -0.0842,  -0.563, 0.1264, -0.7996,  -0.0832,},
{ -0.0564,  0.2682, 0.0388, 0.009,  -0.075, 0.9812, -0.7254,  0.7934,},
{ 0.1564, -0.1882,  -0.1216,  0.0216, -0.3684,  0.448,  0.833,  0.5022,},
{ -0.8944,  -0.2914,  -0.2838,  -0.0084,  -0.9336,  0.3802, 0.1388, -0.3276,},
{ 0.1306, 0.7086, -0.18,  -0.4894,  0.924,  -0.5994,  0.4542, 0.9976,},
{ -0.593, -0.711, -0.9474,  -0.3176,  1.1,  2.585,  1.776,  -0.967,},};
*/

double layer2_out[2]; //nr_output_neurons
const double layer2_weights[9][2] = {
  { -0.3672,  -0.3484,},
  { 0.3754, -0.537,},
  { 0.886,  0.1956,},
  { -0.1652,  0.684,},
  { -0.1538,  -0.1288,},
  { 0.4614, -0.0138,},
  { 0.2718, 0.1088,},
  { 0.1112, 0.0138,},
  { -0.161, -0.072,},
};

/* controid nn
{
{ 0.5738, 0.2046,},
{ 0.3906, -0.6196,},
{ -0.4542,  0.2314,},
{ -0.5594,  0.7498,},
{ 0.3346, -0.5458,},
{ -0.2122,  0.371,},
{ 0.3254, -0.2864,},
{ 0.6944, -0.7304,},
{ -0.2692,  0.053,},};
 */

/*
 * Send my gps position to the other members of the swarm
 * Generate velocity commands based on relative position in swarm
 * using a neural network
 */
void swarm_nn_periodic(void)
{
  /* This algorithm only works if I have a gps fix. */
  //if (gps.fix == 0) {
  //  /* set default speed */
  //    autopilot_guided_move_ned(0, 0, 0, 0);
  //    return;
// }

  /* First send my position to others in the swarm */

  /* The GPS messages are most likely too large to be send over either the datalink
   * The local position is an int32 and the 11 LSBs of the x and y axis are compressed into
   * a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
   * used.
   */
  uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(gps.course) / 1e7) / 2)) & 0x7FF) <<
                             21; // bits 31-21 x position in cm
  multiplex_speed |= (((uint32_t)(gps.gspeed)) & 0x7FF) << 10;         // bits 20-10 y position in cm
  multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);               // bits 9-0 z position in cm

  int16_t alt = (int16_t)(gps.hmsl / 10);

#ifdef EXTRA_DOWNLINK_DEVICE
  DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat,
                          &gps.lla_pos.lon, &alt);
#else
  DOWNLINK_SEND_GPS_SMALL(DefaultChannel, DefaultDevice, &multiplex_speed, &gps.lla_pos.lat,
                          &gps.lla_pos.lon, &alt);
#endif

  /* initialise velocity setpoint */
  struct EnuCoor_f speed_sp = {.x = 0., .y = 0., .z = 0.};

  /* counters */
  uint8_t i, j;

  /* nn inputs */
  float rx = 0.;
  float ry = 0.;
  float rz = 0.;
  float d  = 0.;

  struct UtmCoor_i my_pos = utm_int_from_gps(&gps, 0);
  my_pos.alt = gps.hmsl;

  // compute nn inputs
  for (i = 0; i < ti_acs_idx; i++) {
    if (ti_acs[i].ac_id == 0 || ti_acs[i].ac_id == AC_ID) { continue; }
    struct ac_info_ * ac = get_ac_info(ti_acs[i].ac_id);

    // if AC not responding for too long, continue, else compute force
    uint32_t delta_t = ABS((int32_t)(gps.tow - ac->itow));
    if(delta_t > 5000) { continue; }

    // get distance to other with the assumption of constant velocity since last position message
    float de = (ac->utm.east - my_pos.east) / 100. + sinf(ac->course) * delta_t / 1000.;
    float dn = (ac->utm.north - my_pos.north) / 100. + cosf(ac->course) * delta_t / 1000.;
    float da = (ac->utm.alt - my_pos.alt + ac->climb * delta_t) / 1000.;

    float dist2 = de * de + dn * dn;
    if (use_height) { dist2 += da * da; }

    rx += de;
    ry += dn;
    if (use_height) { rz += da; }
    d  += sqrtf(dist2);
  }

  /* set nn inputs */
  input_layer_out[0] = (double)rx;
  input_layer_out[1] = (double)ry;
  input_layer_out[2] = (double)rz;
  input_layer_out[3] = (double)d;
  input_layer_out[4] = 0; //(double)my_enu_pos.x;    // todo can add offset here later to move swarm centre
  input_layer_out[5] = 0; //(double)my_enu_pos.y;
  input_layer_out[nr_input_neurons] = 1.;   // bias neuron

  /* Compute output for hidden layer */
  for (i = 0; i < nr_hidden_neurons; i++) {
    hidden_layer_out[i] = 0.;
    for (j = 0; j <= nr_input_neurons; j++) {
      hidden_layer_out[i] += input_layer_out[j] * layer1_weights[j][i];
    }
    hidden_layer_out[i] = tanh(hidden_layer_out[i]);
  }

  /* Compute output for ouput layer */
  hidden_layer_out[nr_hidden_neurons] = 1.;  // set bias node
  for (i = 0; i < nr_output_neurons; i++) {
    layer2_out[i] = 0.;
    for (j = 0; j <= nr_hidden_neurons; j++) {
      layer2_out[i] += hidden_layer_out[j] * layer2_weights[j][i];
    }
    layer2_out[i] = tanh(layer2_out[i]);
  }

  /* scale output to max speed setting */
  speed_sp.x = max_hor_speed * (float)layer2_out[0];
  speed_sp.y = max_hor_speed * (float)layer2_out[1];
  if (use_height && nr_output_neurons > 2) {
    speed_sp.z = max_vert_speed * (float)layer2_out[2];
  }

  /* extra safety check (not strictly required) */
  BoundAbs(speed_sp.x, max_hor_speed);
  BoundAbs(speed_sp.y, max_hor_speed);
  BoundAbs(speed_sp.z, max_vert_speed);

  /* set speed */
  autopilot_guided_move_ned(speed_sp.y, speed_sp.x, speed_sp.z, 0);

  /* send log data to gs */
  struct ac_info_ * ac1 = get_ac_info(ti_acs[2].ac_id);
  struct ac_info_ * ac2 = get_ac_info(ti_acs[3].ac_id);

  int32_t tempx1 = (ac1->utm.east  - my_pos.east);
  int32_t tempy1 = (ac1->utm.north - my_pos.north);
  int32_t tempx2 = (ac2->utm.east  - my_pos.east);
  int32_t tempy2 = (ac2->utm.north - my_pos.north);

  struct EnuCoor_f my_enu_pos = *stateGetPositionEnu_f();

  DOWNLINK_SEND_SWARMNN(DefaultChannel, DefaultDevice, &speed_sp.x, &speed_sp.y, &speed_sp.z,
                        &rx, &ry, &rz, &d,
                        &my_enu_pos.x, &my_enu_pos.y, &my_enu_pos.z,
                        &ac1->ac_id, &tempx1, &tempy1,
                        &ac2->ac_id, &tempx2, &tempy2);

}
