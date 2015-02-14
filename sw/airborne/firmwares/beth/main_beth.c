#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"

#include "subsystems/datalink/downlink.h"

#include "beth/bench_sensors.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static inline void main_on_bench_sensors(void);


int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  mcu_int_enable();
}

static inline void main_periodic_task(void)
{

  RunOnceEvery(512, { DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  read_bench_sensors();

}


static inline void main_event_task(void)
{

  BenchSensorsEvent(main_on_bench_sensors);

}


static inline void main_on_bench_sensors(void)
{

  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &bench_sensors_angle_1,
                            &bench_sensors_angle_2);

}


