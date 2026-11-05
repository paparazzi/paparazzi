/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
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

/** @file "modules/digital_cam/thermal_ctl.c"
 * @author Fabien-B <fabien-b@github.com>
 *
 * Control a thermal camera connected to a companion computer.
 *
 * See https://github.com/enacuavlab/thermal_imav2026 for more information
 * about the integration of a MLX90640 thermal camera.
 *
 * Payload is controlled via PAYLOAD_COMMAND messages.
 * Designed for IMAV2026 competition.
 *
 */

#include "modules/digital_cam/thermal_ctl.h"
#include "modules/datalink/downlink.h"
#include "modules/datalink/extra_pprz_dl.h"
#include <stdio.h>
#include <string.h>

float thermal_ctl = 0;
float thermal_threshold = 50;

#define THERMAL_CTL_NONE    0
#define THERMAL_CTL_PWR_OFF 1
#define THERMAL_CTL_REC_ON  2
#define THERMAL_CTL_REC_OFF 3

#define NB_SPOTS 3

struct thermal_spot
{
  uint8_t cluster_idx;
  int32_t lat;
  int32_t lon;
  float temp;
  int count;
};


struct thermal_spot hottest_spots[3] = {0};

static void thermal_send_draw(struct thermal_spot* spot);
static void update_hottests(struct thermal_spot* spot);


void thermal_ctl_thermal_ctl_handler(float value) {
  int val = (int)value;
  uint8_t tab[2];
  uint8_t dst_id = 0;
  if(val == THERMAL_CTL_PWR_OFF) {
    tab[0] = 's';
    tab[1] = 0;
  } else if(val == THERMAL_CTL_REC_ON) {
    tab[0] = 'r';
    tab[1] = 1;
  } else if(val == THERMAL_CTL_REC_OFF) {
    tab[0] = 'r';
    tab[1] = 0;
  } else {
    return;
  }

  DOWNLINK_SEND_PAYLOAD_COMMAND(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &dst_id, 2, tab);

}

void thermal_ctl_thermal_threshold_handler(float value) {
  thermal_threshold = value;
  uint8_t kind = value > 0 ? 'h': 'c';
  uint8_t tab[2] = {kind, (uint8_t)fabs(value)};
  uint8_t dst_id = 0;
  DOWNLINK_SEND_PAYLOAD_COMMAND(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &dst_id, 2, tab);
}

void thermal_clusters_cb(uint8_t* buf)
{
  // PAYLOAD_FLOAT: cluster_key,east,north,up,latitude,longitude,median_temperature,member_count

  int nb = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
  if (nb != 8) { return; }

  float* b = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);

  // prevent unaligned access
  float data[8];
  memcpy(data, b, 8*sizeof(float));


  struct thermal_spot spot = {
    .cluster_idx = data[0] + 1,  // first is id 1, 0 is reserved.
    .lat = data[4] * 1e7,
    .lon = data[5] * 1e7,
    .temp = data[6],
    .count = data[7],
  };

  update_hottests(&spot);
  thermal_send_draw(&spot);
}

static void update_hottests(struct thermal_spot* spot) {
  int coldest_idx = 0;
  for (int i=0; i<NB_SPOTS; i++) {
    struct thermal_spot *hp = &hottest_spots[i];

    // cluster found in hottests, just update it.
    if (hp->cluster_idx == spot->cluster_idx) {
      memcpy(hp, spot, sizeof(struct thermal_spot));
      return;
    }

    // find the coldest/hottest amongst the array.
    if ((hp->temp < hottest_spots[coldest_idx].temp && thermal_threshold > 0) ||
        (hp->temp > hottest_spots[coldest_idx].temp && thermal_threshold < 0)) {
      coldest_idx = i;
    }
  }

  // the new spot it hotter than the coldest of the array, save it in place of the coldest.
  // or colder than the hottest.
  if ((spot->temp > hottest_spots[coldest_idx].temp && thermal_threshold > 0) ||
      (spot->temp < hottest_spots[coldest_idx].temp && thermal_threshold < 0)) {
    memcpy(&hottest_spots[coldest_idx], spot, sizeof(struct thermal_spot));
  }
}


static void thermal_send_draw(struct thermal_spot* spot)
{
  uint8_t color = 0 << 6 | 5 << 3 | 5;  // Transparent, Magenta, Magenta
  uint8_t shape = 0;
  uint8_t status = 0;
  float radius = 2;

  char text[10];
  int nb_text = snprintf(text, 10, "%.0fC (%d)", spot->temp, spot->count);

  DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice, &spot->cluster_idx, &color, &shape,
      &status, &radius, 1, &spot->lat, 1, &spot->lon, nb_text, text);
}


void thermal_cluster_periodic() {
  static int hottest_idx = 0;

  if (hottest_spots[hottest_idx].cluster_idx != 0) {
    thermal_send_draw(&hottest_spots[hottest_idx]);
    hottest_idx = (hottest_idx + 1) % NB_SPOTS;
  }
}

