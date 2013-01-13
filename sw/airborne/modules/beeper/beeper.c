
#include "beeper.h"
#include "autopilot.h"
#include "subsystems/gps.h"
#include "subsystems/electrical.h"
#include "subsystems/ahrs.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/ins.h"
#include "generated/airframe.h"
#include BOARD_CONFIG

#if defined (STM32F1) || defined(STM32F4)
#if defined (STM32F1)
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#elif defined(STM32F4)
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#endif
#define BEEPER_OFF 		gpio_clear(BEEPER_GPIO, BEEPER_GPIO_PIN)
#define BEEPER_ON 		gpio_set(BEEPER_GPIO, BEEPER_GPIO_PIN)
#endif

#ifdef LPC21
#define BEEPER_DIR		IO_(BEEPER_PORT, DIR)
#define BEEPER_CLR		IO_(BEEPER_PORT, CLR)
#define BEEPER_SET		IO_(BEEPER_PORT, SET)
#define BEEPER_OFF 		(BEEPER_CLR |= (1 << BEEPER_PIN))
#define BEEPER_ON 		(BEEPER_SET |= (1 << BEEPER_PIN))
#endif

#define BEEPER_SWITCH(_beeper) (_beeper?BEEPER_ON:BEEPER_OFF)

#define 	BEEP_MODE_SYS_INIT_TICKS_ON				1
#define		BEEP_MODE_SYS_INIT_TICKS_OFF			48

#define 	BEEP_MODE_IMU_ERROR_TICKS_ON			16
#define		BEEP_MODE_IMU_ERROR_TICKS_OFF			16

#define 	BEEP_MODE_LOW_BAT_TICKS_ON				2
#define		BEEP_MODE_LOW_BAT_TICKS_OFF				2

#define		BEEP_MODE_OK_TICKS_ON							1
#define		BEEP_MODE_OK_TICKS_OFF						1
#define		BEEP_MODE_OK_TICKS_NUM						2

#define LowBatLevel()	(bat_low == 0)

#define BAT_CRITICAL_TIME	80

uint8_t beep_mode;
uint8_t beep_num;
bool_t	beeper_on;
uint16_t beep_cnt;
bool_t motor_is_on = FALSE;
uint16_t bat_low;

void beeper_init(){
#if defined(STM32F1) || defined(STM32F2)
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(BEEPER_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = BEEPER_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BEEPER_GPIO, &GPIO_InitStructure);
#elif defined(STM32F4)
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, BEEPER_GPIO_CLK);
	gpio_mode_setup(BEEPER_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BEEPER_GPIO_PIN);
#endif
	BEEPER_OFF;
	beep_mode = BEEP_MODE_OK;
	beep_num = BEEP_MODE_OK_TICKS_NUM;
	bat_low = BAT_CRITICAL_TIME;
}

#define BEEP_CYCLE(_TICKS_ON, _TICKS_OFF) {						\
	if(beeper_on){																			\
		if(beep_cnt)	beep_cnt--;																		\
		else {																						\
			BEEPER_OFF;																			\
			beep_cnt = _TICKS_OFF;																\
			beeper_on = FALSE;															\
			if(beep_num)																		\
				beep_num--;																		\
		}																									\
	}																										\
	else {																							\
		if(beep_cnt) beep_cnt--;																		\
		else{																							\
			BEEPER_ON;																			\
			beep_cnt = _TICKS_ON;																\
			beeper_on = TRUE;																\
		}																									\
	}																										\
}																											\

#define CheckHardware() {															\
	if (ahrs.status != AHRS_RUNNING || baro.status != BS_RUNNING || autopilot_mode == AP_MODE_KILL || autopilot_mode == AP_MODE_FAILSAFE)					\
		set_beep_mode(BEEP_MODE_SYS_INIT, BEEP_MODE_SYS_INIT);							\
	else {																																\
		if(autopilot_mode == AP_MODE_HOVER_Z_HOLD || autopilot_mode == AP_MODE_HOVER_CLIMB || autopilot_mode == AP_MODE_HOVER_DIRECT || autopilot_mode == AP_MODE_NAV) {	\
			if (gps.fix == GPS_FIX_3D && ins_ltp_initialised == TRUE)					\
				set_beep_mode(BEEP_MODE_SYS_INIT, BEEP_MODE_OK);								\
			else																															\
				set_beep_mode(BEEP_MODE_SYS_INIT, BEEP_MODE_SYS_INIT);					\
		}																																		\
		else																																\
			set_beep_mode(BEEP_MODE_SYS_INIT, BEEP_MODE_OK);									\
		if(autopilot_motors_on != motor_is_on) {															\
			set_beep_mode(BEEP_MODE_OK, BEEP_MODE_OK);									\
			motor_is_on = autopilot_motors_on;																	\
		}																																		\
	}																																			\
	if(electrical.vsupply < LOW_BAT_LEVEL*10) {                             \
		if(bat_low > 0)                                                       \
			bat_low--;                                                          \
		else                                                                  \
		  set_beep_mode(BEEP_MODE_LOW_BAT, BEEP_MODE_LOW_BAT);								\
	}                                                                       \
	else                                                                    \
		if(bat_low && bat_low < BAT_CRITICAL_TIME) {                                                         \
			bat_low = BAT_CRITICAL_TIME;                              \
			set_beep_mode(BEEP_MODE_LOW_BAT, BEEP_MODE_OK);											\
		}                                                                     \
}																																				

void periodic_task_beeper(){
	RunOnceEvery(4, CheckHardware());

	switch (beep_mode){
		case BEEP_MODE_OK:
			if(beep_num)
				BEEP_CYCLE(BEEP_MODE_OK_TICKS_ON, BEEP_MODE_OK_TICKS_OFF)
			else
				BEEPER_OFF;
			break;
		case BEEP_MODE_SYS_INIT:
			BEEP_CYCLE(BEEP_MODE_SYS_INIT_TICKS_ON, BEEP_MODE_SYS_INIT_TICKS_OFF);
			break;
		case BEEP_MODE_IMU_ERROR:
			BEEP_CYCLE(BEEP_MODE_IMU_ERROR_TICKS_ON, BEEP_MODE_IMU_ERROR_TICKS_OFF);
			break;
		case BEEP_MODE_LOW_BAT:
			BEEP_CYCLE(BEEP_MODE_LOW_BAT_TICKS_ON, BEEP_MODE_LOW_BAT_TICKS_OFF);
			break;
		default:
		  break;
	}
}

void set_beep_mode(uint8_t _old_mode, uint8_t _new_mode) {
	if(_new_mode == BEEP_MODE_OK) {
		if(_old_mode == beep_mode) {
			beep_num = BEEP_MODE_OK_TICKS_NUM;
			beep_mode = BEEP_MODE_OK;
			beep_cnt = 0;
		}
	}
	else
		if(_new_mode > beep_mode)
			beep_mode = _new_mode;
}

