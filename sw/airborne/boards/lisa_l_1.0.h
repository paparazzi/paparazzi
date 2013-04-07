#ifndef CONFIG_LISA_L_1_0_H
#define CONFIG_LISA_L_1_0_H

#define BOARD_LISA_L

/* Lisa/L has an 8MHZ external clock and 72MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 72000000

/* Onboard LEDs */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_STP08

// FIXME, this is just to make it compile
#define POWER_SWITCH_LED 5


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* PA0 - ADC0 */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 2
#endif
#define DefaultVoltageOfAdc(adc) (0.0059*adc)
/* Onboard ADCs */
#define BOARD_ADC_CHANNEL_1 8
#define BOARD_ADC_CHANNEL_2 9
// FIXME - removed for now and used for battery monitoring
//#define BOARD_ADC_CHANNEL_3 13
#define BOARD_ADC_CHANNEL_3 0
#define BOARD_ADC_CHANNEL_4 15

#define BOARD_HAS_BARO 1

#endif /* CONFIG_LISA_L_1_0_H */
