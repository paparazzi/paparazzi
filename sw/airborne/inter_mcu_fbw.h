/* $Id $
 *
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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
 *
 */

extern bool_t ap_ok;
extern uint8_t time_since_last_ap;

/* Prepare data to be sent to mcu0 */
static inline void to_autopilot_from_rc_values (void) {
  struct from_fbw_msg *msg = &(from_fbw.from_fbw);

  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
    msg->channels[i] = rc_values[i];

  uint8_t status;
  status = (radio_ok ? _BV(STATUS_RADIO_OK) : 0);
  status |= (radio_really_lost ? _BV(RADIO_REALLY_LOST) : 0);
  status |= (mode == FBW_MODE_AUTO ? _BV(STATUS_MODE_AUTO) : 0);
  status |= (failsafe_mode ? _BV(STATUS_MODE_FAILSAFE) : 0);
  msg->status  = status;

  if (rc_values_contains_avg_channels) {
    msg->status |= _BV(AVERAGED_CHANNELS_SENT);
    rc_values_contains_avg_channels = FALSE;
  }
  msg->ppm_cpt = last_ppm_cpt;
  msg->vsupply = VoltageOfAdc(vsupply_adc_buf.sum/vsupply_adc_buf.av_nb_sample) * 10;
#if defined IMU_3DMG || defined IMU_ANALOG
  msg->euler_dot[0] = roll_dot;
  msg->euler_dot[1] = pitch_dot;
  msg->euler_dot[2] = yaw_dot;
#endif
#ifdef IMU_3DMG
  msg->euler[0] = roll;
  msg->euler[1] = pitch;
  msg->euler[2] = yaw;
#endif
}

static inline void inter_mcu_event_task( void) {
  if (from_ap_receive_valid) {
    time_since_last_ap = 0;
    ap_ok = TRUE;
    if (mode == FBW_MODE_AUTO) {
      SetCommands(from_ap.from_ap.channels);
    }
    to_autopilot_from_rc_values();
  }
  if (time_since_last_ap == STALLED_TIME) {
    ap_ok = FALSE;
  }
}

static inline void inter_mcu_periodic_task(void) {
  if (time_since_last_ap < STALLED_TIME)
    time_since_last_ap++;
}
