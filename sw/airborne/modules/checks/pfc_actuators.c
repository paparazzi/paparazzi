/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/checks/pfc_actuators.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Checks the actuators with feedback before takeoff
 */

#include "preflight_checks.h"
#include "core/abi.h"
#include "modules/datalink/telemetry.h"
#include <stdio.h>

/**
 * Maximum error for the angle of the actuators (rad)
*/
#ifndef PFC_ACTUATORS_MAX_ANGLE_ERROR
#define PFC_ACTUATORS_MAX_ANGLE_ERROR 0.1f
#endif

/**
 * Maximum error for the RPM of the actuators
*/
#ifndef PFC_ACTUATORS_MAX_RPM_ERROR
#define PFC_ACTUATORS_MAX_RPM_ERROR 250.0f
#endif

/**
 * Enable debugging to set the expected feedback values
*/
#ifndef PFC_ACTUATORS_DEBUG
#define PFC_ACTUATORS_DEBUG false
#endif

/**
 * @brief The status of the preflight checks
*/
enum pfc_actuators_state_t {
  PFC_ACTUATORS_STATE_INIT,
  PFC_ACTUATORS_STATE_RUNNING,
  PFC_ACTUATORS_STATE_SUCCESS,
  PFC_ACTUATORS_STATE_ERROR
};

/**
 * @brief The state of the actuator during the test
*/
enum pfc_actuator_state_t {
  PFC_ACTUATOR_STATE_WAIT,
  PFC_ACTUATOR_STATE_LOW,
  PFC_ACTUATOR_STATE_HIGH,
};

/**
 * @brief The configuration struct of the actuator
*/
struct pfc_actuator_t {
  uint8_t feedback_id;    ///< The feedback id of the actuator (255 for none)
  uint8_t feedback_id2;   ///< The secondary feedback id of the actuator (255 for none)

  int16_t low;            ///< The low value to set the actuator to
  int16_t high;           ///< The high value to set the actuator to
  float low_feedback;     ///< The expected feedback value when the actuator is low
  float high_feedback;    ///< The expected feedback value when the actuator is high

  float timeout;          ///< The timeout for the actuator to move
};

struct pfc_actuators_t {
  enum pfc_actuators_state_t state;     ///< The state of the preflight checks

  uint8_t act_idx;                      ///< The current actuator index
  uint8_t act_nb;                       ///< The number of actuators
  enum pfc_actuator_state_t act_state;  ///< The state of the actuator (during the test)
  float act_start_time;                 ///< The start time of the actuator (during the test)

  float last_feedback;                  ///< The last measured feedback value of the actuator
  float last_feedback_err;              ///< The last expected feedback error of the actuator (based on RPM/angle)
  float last_feedback2;                 ///< The last measured secondary feedback value of the actuator
  float last_feedback_err2;             ///< The last expected secondary feedback error of the actuator (based on RPM/angle)
};

// Local variables and functions
static struct pfc_actuator_t pfc_acts[] = PFC_ACTUATORS;
static struct pfc_actuators_t pfc_actuators;
static struct preflight_check_t actuators_pfc;
static abi_event act_feedback_ev;
static void pfc_actuators_cb(struct preflight_result_t *result);
static void pfc_act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);

/**
 * @brief Send an error message to the ground station
 * @param fmt The format of the message
 * @param ... The arguments for the format
 */
static void pfc_actuators_error(const char *fmt, ...) {
  char msg[200];

  // Add the error
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(msg, 200, fmt, args);
  va_end(args);

  if(rc > 0) {
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, msg);
  }
}

/**
 * @brief Send a debug message to the ground station
*/
#if PFC_ACTUATORS_DEBUG
#define pfc_actuators_debug pfc_actuators_error
#else
#define pfc_actuators_debug(...)
#endif

/**
 * @brief Register the preflight checks for the actuators
 */
void pfc_actuators_init(void) {
  pfc_actuators.state = PFC_ACTUATORS_STATE_INIT;
  pfc_actuators.act_state = PFC_ACTUATOR_STATE_WAIT;
  pfc_actuators.act_idx = 0;
  pfc_actuators.act_nb = sizeof(pfc_acts) / sizeof(struct pfc_actuator_t);
  pfc_actuators.last_feedback = NAN;
  pfc_actuators.last_feedback2 = NAN;
  
  preflight_check_register(&actuators_pfc, pfc_actuators_cb);
  AbiBindMsgACT_FEEDBACK(ABI_BROADCAST, &act_feedback_ev, pfc_act_feedback_cb);
}

/**
 * @brief Move the actuators, should be put in the command laws
 */
void pfc_actuators_run(void) {
  // Only actuate when running
  if(pfc_actuators.state != PFC_ACTUATORS_STATE_RUNNING || pfc_actuators.act_idx >= pfc_actuators.act_nb) {
    return;
  }

  // Get the actuators
  struct pfc_actuator_t *act = &pfc_acts[pfc_actuators.act_idx];

  // Verify the result and continue
  if((get_sys_time_float() - pfc_actuators.act_start_time) > act->timeout) {
    switch(pfc_actuators.act_state) {
      case PFC_ACTUATOR_STATE_WAIT:
        pfc_actuators_debug("Actuator %d starting LOW", pfc_actuators.act_idx);
        pfc_actuators.act_state = PFC_ACTUATOR_STATE_LOW;
        pfc_actuators.act_start_time = get_sys_time_float();
        pfc_actuators.last_feedback = NAN;
        pfc_actuators.last_feedback2 = NAN;
        break;
      case PFC_ACTUATOR_STATE_LOW:
        // Check if the feedback is correct
        if(act->feedback_id != 255 && 
          (pfc_actuators.last_feedback < (act->low_feedback - pfc_actuators.last_feedback_err) || pfc_actuators.last_feedback > (act->low_feedback + pfc_actuators.last_feedback_err))) {
          pfc_actuators_error("Actuator %d not responding correctly LOW %.2f < %.2f < %.2f", pfc_actuators.act_idx, act->low_feedback - pfc_actuators.last_feedback_err, pfc_actuators.last_feedback, act->low_feedback + pfc_actuators.last_feedback_err);
          pfc_actuators.state = PFC_ACTUATORS_STATE_ERROR;
        } else if(act->feedback_id2 != 255 && 
          (pfc_actuators.last_feedback2 < (act->low_feedback - pfc_actuators.last_feedback_err2) || pfc_actuators.last_feedback2 > (act->low_feedback + pfc_actuators.last_feedback_err2))) {
          pfc_actuators_error("Actuator %d not responding correctly LOW2 %.2f < %.2f < %.2f", pfc_actuators.act_idx, act->low_feedback - pfc_actuators.last_feedback_err2, pfc_actuators.last_feedback2, act->low_feedback + pfc_actuators.last_feedback_err2);
          pfc_actuators.state = PFC_ACTUATORS_STATE_ERROR;
        }
        if(pfc_actuators.state != PFC_ACTUATORS_STATE_ERROR || PFC_ACTUATORS_DEBUG) {
          pfc_actuators_debug("Actuator %d LOW ok %.2f, %.2f starting HIGH", pfc_actuators.act_idx, pfc_actuators.last_feedback, pfc_actuators.last_feedback2);
          pfc_actuators.state = PFC_ACTUATORS_STATE_RUNNING;
          pfc_actuators.act_state = PFC_ACTUATOR_STATE_HIGH;
          pfc_actuators.act_start_time = get_sys_time_float();
          pfc_actuators.last_feedback = NAN;
          pfc_actuators.last_feedback2 = NAN;
        }
        break;
      case PFC_ACTUATOR_STATE_HIGH:
        // Check if the feedback is correct
        if(act->feedback_id != 255 && 
          (pfc_actuators.last_feedback < (act->high_feedback - pfc_actuators.last_feedback_err) || pfc_actuators.last_feedback > (act->high_feedback + pfc_actuators.last_feedback_err))) {
          pfc_actuators_error("Actuator %d not responding correctly HIGH %.2f < %.2f < %.2f", pfc_actuators.act_idx, act->high_feedback - pfc_actuators.last_feedback_err, pfc_actuators.last_feedback, act->high_feedback + pfc_actuators.last_feedback_err);
          pfc_actuators.state = PFC_ACTUATORS_STATE_ERROR;
        } else if(act->feedback_id2 != 255 && 
          (pfc_actuators.last_feedback2 < (act->high_feedback - pfc_actuators.last_feedback_err2) || pfc_actuators.last_feedback2 > (act->high_feedback + pfc_actuators.last_feedback_err2))) {
          pfc_actuators_error("Actuator %d not responding correctly HIGH2 %.2f < %.2f < %.2f", pfc_actuators.act_idx, act->high_feedback - pfc_actuators.last_feedback_err2, pfc_actuators.last_feedback2, act->high_feedback + pfc_actuators.last_feedback_err2);
          pfc_actuators.state = PFC_ACTUATORS_STATE_ERROR;
        } 
        if(pfc_actuators.state != PFC_ACTUATORS_STATE_ERROR || PFC_ACTUATORS_DEBUG) {
          pfc_actuators_debug("Actuator %d HIGH ok %.2f, %.2f", pfc_actuators.act_idx, pfc_actuators.last_feedback, pfc_actuators.last_feedback2);
          pfc_actuators.state = PFC_ACTUATORS_STATE_RUNNING;
          pfc_actuators.act_state = PFC_ACTUATOR_STATE_WAIT;
          pfc_actuators.act_start_time = 0;
          pfc_actuators.act_idx++;
        }
        break;
    }
  }

  // Finished testing
  if(pfc_actuators.act_idx >= pfc_actuators.act_nb) {
    pfc_actuators.state = PFC_ACTUATORS_STATE_SUCCESS;
    pfc_actuators_error("Actuators checks done %d", pfc_actuators.act_idx);
  }
}

/**
 * @brief Start the actuator testing
*/
void pfc_actuators_start(bool start) {
  if(start && pfc_actuators.state != PFC_ACTUATORS_STATE_RUNNING) {
    pfc_actuators.act_idx = 0;
    pfc_actuators.act_start_time = 0;
    pfc_actuators.state = PFC_ACTUATORS_STATE_RUNNING;
    pfc_actuators.act_state = PFC_ACTUATOR_STATE_WAIT;
    pfc_actuators.last_feedback = NAN;
    pfc_actuators.last_feedback2 = NAN;
  }
  else if(!start && pfc_actuators.state == PFC_ACTUATORS_STATE_RUNNING) {
    pfc_actuators.act_idx = 0;
    pfc_actuators.act_start_time = 0;
    pfc_actuators.state = PFC_ACTUATORS_STATE_INIT;
    pfc_actuators.act_state = PFC_ACTUATOR_STATE_WAIT;
  }
}

/**
 * @brief Get the actuator value in the command laws to move the actuator during the preflight checks
 * @param idx The index of the actuator in the preflight checks struct
 * @param value The value if no checks are performed on this actuator
*/
int16_t pfc_actuators_value(uint8_t idx, int16_t value) {
  if(idx != pfc_actuators.act_idx || pfc_actuators.state != PFC_ACTUATORS_STATE_RUNNING) {
    return value;
  }

  if(pfc_actuators.act_state == PFC_ACTUATOR_STATE_LOW) {
    return pfc_acts[pfc_actuators.act_idx].low;
  }
  else if(pfc_actuators.act_state == PFC_ACTUATOR_STATE_HIGH) {
    return pfc_acts[pfc_actuators.act_idx].high;
  }

  return value;
}

/**
 * @brief Check the actuators with feedback
 * 
 * @param result The result of the preflight checks
 */
static void pfc_actuators_cb(struct preflight_result_t *result)
{
  switch(pfc_actuators.state) {
    case PFC_ACTUATORS_STATE_INIT:
      preflight_error(result, "Actuators not checked perform checks first[%d]", pfc_actuators.act_nb);
      break;
    case PFC_ACTUATORS_STATE_RUNNING:
      preflight_error(result, "Actuators are currently being checked[%d/%d]",pfc_actuators.act_idx, pfc_actuators.act_nb);
      break;
    case PFC_ACTUATORS_STATE_SUCCESS:
      preflight_success(result, "Actuators checked and moved succesfully[%d]", pfc_actuators.act_nb);
      break;
    case PFC_ACTUATORS_STATE_ERROR:
      preflight_error(result, "Actuators not responding correctly[%d/%d]",pfc_actuators.act_idx, pfc_actuators.act_nb);
      break;
  }
}

/**
 * @brief Callback for the actuator feedback
*/
static void pfc_act_feedback_cb(uint8_t sender_id __attribute__((unused)), struct act_feedback_t *feedback, uint8_t num_act) {
  if(pfc_actuators.state != PFC_ACTUATORS_STATE_RUNNING || pfc_actuators.act_idx >= pfc_actuators.act_nb) {
    return;
  }

  // Save the last feedback values
  for(uint8_t i = 0; i < num_act; i++) {
    if(feedback[i].idx == pfc_acts[pfc_actuators.act_idx].feedback_id && feedback[i].set.rpm) {
      pfc_actuators.last_feedback = feedback[i].rpm;
      pfc_actuators.last_feedback_err = PFC_ACTUATORS_MAX_RPM_ERROR;
    } else if(feedback[i].idx == pfc_acts[pfc_actuators.act_idx].feedback_id && feedback[i].set.position) {
      pfc_actuators.last_feedback = feedback[i].position;
      pfc_actuators.last_feedback_err = PFC_ACTUATORS_MAX_ANGLE_ERROR;
    }

    if(feedback[i].idx == pfc_acts[pfc_actuators.act_idx].feedback_id2 && feedback[i].set.rpm) {
      pfc_actuators.last_feedback2 = feedback[i].rpm;
      pfc_actuators.last_feedback_err2 = PFC_ACTUATORS_MAX_RPM_ERROR;
    } else if(feedback[i].idx == pfc_acts[pfc_actuators.act_idx].feedback_id2 && feedback[i].set.position) {
      pfc_actuators.last_feedback2 = feedback[i].position;
      pfc_actuators.last_feedback_err2 = PFC_ACTUATORS_MAX_ANGLE_ERROR;
    }
  }
}
