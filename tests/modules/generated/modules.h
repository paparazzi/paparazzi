#ifndef GENERATED_MODULES_H
#define GENERATED_MODULES_H

#define MODULES_IDLE  0
#define MODULES_RUN   1
#define MODULES_START 2
#define MODULES_STOP  3

#include "std.h"
#include "core/settings.h"
#include "radio_control/radio_control.h"
#include "./mcu.h"
#include "mcu_periph/gpio.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_orientation_conversion.h"
#include "math/pprz_stat.h"
#include "imu/imu.h"
#include "mcu_periph/i2c.h"
#include "gps/gps.h"
#include "air_data/air_data.h"
#include "ahrs/ahrs.h"
#include "actuators/actuators.h"
#include "mcu_periph/adc.h"
#include "energy/electrical.h"
#include "mcu_periph/spi.h"
#include "./state.h"
#include "mcu_periph/uart.h"
#include "core/commands.h"

// dummy variables
extern int nav_catapult_nav_catapult_highrate_module_status;

static inline void modules_mcu_init(void) {}
static inline void modules_core_init(void) {}
static inline void modules_sensors_init(void) {}
static inline void modules_estimation_init(void) {}
static inline void modules_radio_control_init(void) {}
static inline void modules_control_init(void) {}
static inline void modules_actuators_init(void) {}
static inline void modules_datalink_init(void) {}
static inline void modules_default_init(void) {}

static inline void modules_init(void) {
  modules_mcu_init();
  modules_core_init();
  modules_sensors_init();
  modules_estimation_init();
  modules_radio_control_init();
  modules_control_init();
  modules_actuators_init();
  modules_datalink_init();
  modules_default_init();
}

static inline void modules_mcu_periodic_task(void) {}
static inline void modules_core_periodic_task(void) {}
static inline void modules_sensors_periodic_task(void) {}
static inline void modules_estimation_periodic_task(void) {}
static inline void modules_radio_control_periodic_task(void) {}
static inline void modules_control_periodic_task(void) {}
static inline void modules_actuators_periodic_task(void) {}
static inline void modules_datalink_periodic_task(void) {}
static inline void modules_default_periodic_task(void) {}

static inline void modules_periodic_task(void) {
  modules_mcu_periodic_task();
  modules_core_periodic_task();
  modules_sensors_periodic_task();
  modules_estimation_periodic_task();
  modules_radio_control_periodic_task();
  modules_control_periodic_task();
  modules_actuators_periodic_task();
  modules_datalink_periodic_task();
  modules_default_periodic_task();
}

static inline void modules_mcu_event_task(void) {}
static inline void modules_core_event_task(void) {}
static inline void modules_sensors_event_task(void) {}
static inline void modules_estimation_event_task(void) {}
static inline void modules_radio_control_event_task(void) {}
static inline void modules_control_event_task(void) {}
static inline void modules_actuators_event_task(void) {}
static inline void modules_datalink_event_task(void) {}
static inline void modules_default_event_task(void) {}

static inline void modules_event_task(void) {
  modules_mcu_event_task();
  modules_core_event_task();
  modules_sensors_event_task();
  modules_estimation_event_task();
  modules_radio_control_event_task();
  modules_control_event_task();
  modules_actuators_event_task();
  modules_datalink_event_task();
  modules_default_event_task();
}

#ifdef MODULES_DATALINK_C

#include "pprzlink/messages.h"
#include "generated/airframe.h"
static inline void modules_parse_datalink(uint8_t msg_id __attribute__ ((unused)),
                                          uint8_t class_id __attribute__((unused)),
                                          struct link_device *dev __attribute__((unused)),
                                          struct transport_tx *trans __attribute__((unused)),
                                          uint8_t *buf __attribute__((unused))) {
}

#endif

#endif // GENERATED_MODULES_H
