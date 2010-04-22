#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "interrupt_hw.h"

#include "downlink.h"

#include "beth/bench_sensors.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_on_bench_sensors( void );


int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  int_enable();
}

static inline void main_periodic_task( void ) {

  RunOnceEvery(512, { DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  read_bench_sensors();

}


static inline void main_event_task( void ) {

  BenchSensorsEvent(main_on_bench_sensors);

}


static inline void main_on_bench_sensors( void ) {
  
  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &bench_sensors_angle_1,
			    &bench_sensors_angle_2);
  
}


