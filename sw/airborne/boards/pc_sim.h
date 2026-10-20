#ifndef CONFIG_PC_SIM_H
#define CONFIG_PC_SIM_H

#define ADC_1 1
#define ADC_2 2
#define ADC_3 3
#define ADC_4 4
#define ADC_5 5
#define ADC_6 6
#define ADC_7 7
#define ADC_8 8

/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet ActuatorPwmSet
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#define DefaultVoltageOfAdc(adc) (1.0*adc)

#ifndef USE_BARO_BOARD
#if USE_NPS
#define USE_BARO_BOARD 1
#else
#define USE_BARO_BOARD 0
#endif
#endif

#include "peripherals/video_device.h"

extern struct video_config_t webcam;

// Simulated cameras, see modules/computer_vision/video_thread_nps.c
extern struct video_config_t front_camera;
extern struct video_config_t bottom_camera;

#endif /* CONFIG_PC_SIM_H */
