#ifndef KAMERA_H_INCLUDED
#define KAMERA_H_INCLUDED
#include "std.h"

#ifdef LPC21
#ifndef ADC_CHANNEL_CAM1
#define ADC_CHANNEL_CAM1 		ADC_7
#endif

#define CAM1_STATE_ON_LEVEL 	512
#define CAM2_STATE_ON_LEVEL 	512

#define VIDEO

#ifndef CAM1_SHOT_PORT
#define CAM1_SHOT_PORT 			0
#endif
#ifndef CAM1_SHOT_PIN
#define CAM1_SHOT_PIN 			12
#endif
#ifndef CAM1_SWITCH_PORT
#define CAM1_SWITCH_PORT 		0
#endif
#ifndef CAM1_SWITCH_PIN
#define CAM1_SWITCH_PIN 		7
#endif
#ifndef CAM1_VIDEO_PORT
#define CAM1_VIDEO_PORT 		0
#endif
#ifndef CAM1_VIDEO_PIN
#define CAM1_VIDEO_PIN 			13
#endif

#ifndef CAM1_PINSEL
#define CAM1_PINSEL 				PINSEL0
#endif

#ifndef CAM1_PINSEL_VAL
#define CAM1_PINSEL_VAL 		(3<<24)
#endif

#ifdef ADC_CHANNEL_CAM1
//struct adc_buf cam1_state;
int32_t cam1_state_val;
#endif
#ifdef CAMERA_2
#ifdef ADC_CHANNEL_CAM2
struct adc_buf cam2_state;
#endif
#endif

#define CAM1_SHOT_IODIR IO_(CAM1_SHOT_PORT, DIR)
#define CAM1_SHOT_IOSET IO_(CAM1_SHOT_PORT, SET)
#define CAM1_SHOT_IOCLR IO_(CAM1_SHOT_PORT, CLR)
#define CAM1_SHOT_IOPIN IO_(CAM1_SHOT_PORT, PIN)

#define CAM1_SWITCH_IODIR IO_(CAM1_SWITCH_PORT, DIR)
#define CAM1_SWITCH_IOSET IO_(CAM1_SWITCH_PORT, SET)
#define CAM1_SWITCH_IOCLR IO_(CAM1_SWITCH_PORT, CLR)
#define CAM1_SWITCH_IOPIN IO_(CAM1_SWITCH_PORT, PIN)

#define CAM1_VIDEO_IODIR IO_(CAM1_VIDEO_PORT, DIR)
#define CAM1_VIDEO_IOSET IO_(CAM1_VIDEO_PORT, SET)
#define CAM1_VIDEO_IOCLR IO_(CAM1_VIDEO_PORT, CLR)
#define CAM1_VIDEO_IOPIN IO_(CAM1_VIDEO_PORT, PIN)

#ifdef CAMERA_2
#define CAM2_SHOT_IODIR IO_(CAM2_SHOT_PORT, DIR)
#define CAM2_SHOT_IOSET IO_(CAM2_SHOT_PORT, SET)
#define CAM2_SHOT_IOCLR IO_(CAM2_SHOT_PORT, CLR)
#define CAM2_SHOT_IOPIN IO_(CAM2_SHOT_PORT, PIN)

#define CAM2_SWITCH_IODIR IO_(CAM2_SWITCH_PORT, DIR)
#define CAM2_SWITCH_IOSET IO_(CAM2_SWITCH_PORT, SET)
#define CAM2_SWITCH_IOCLR IO_(CAM2_SWITCH_PORT, CLR)
#define CAM2_SWITCH_IOPIN IO_(CAM2_SWITCH_PORT, PIN)
#endif

#define Cam1Clr()	{CAM1_PINSEL &= ~CAM1_PINSEL_VAL; CAM1_SHOT_IODIR |= (1 << CAM1_SHOT_PIN); CAM1_SHOT_IOCLR = (1 << CAM1_SHOT_PIN);}
#define Cam1Set()	{CAM1_SHOT_IODIR &= ~(1 << CAM1_SHOT_PIN); CAM1_SHOT_IOSET = (1 << CAM1_SHOT_PIN); CAM1_PINSEL |= CAM1_PINSEL_VAL;}
#define Cam1SwitchSet() {CAM1_SWITCH_IODIR |= (1 << CAM1_SWITCH_PIN); CAM1_SWITCH_IOSET = (1 << CAM1_SWITCH_PIN);}
#define Cam1SwitchClr() {CAM1_SWITCH_IODIR |= (1 << CAM1_SWITCH_PIN); CAM1_SWITCH_IOCLR = (1 << CAM1_SWITCH_PIN);}
#define Cam1VideoSet() {CAM1_VIDEO_IODIR |= (1 << CAM1_VIDEO_PIN); CAM1_VIDEO_IOSET = (1 << CAM1_VIDEO_PIN);}
#define Cam1VideoClr() {CAM1_VIDEO_IODIR |= (1 << CAM1_VIDEO_PIN); CAM1_VIDEO_IOCLR = (1 << CAM1_VIDEO_PIN);}
#endif

#ifdef STM32F4
#define CAM1_STATE_ON_LEVEL 	2048
#define CAM2_STATE_ON_LEVEL 	2048
#define Cam1Clr()	{}
#define Cam1Set()	{}
#define Cam1SwitchSet() {}
#define Cam1SwitchClr() {}
#define Cam1VideoSet {} //CAM_V_GPIO->BSRR = CAM_V_GPIO_PIN;
#define Cam1VideoClr() {} //CAM_V_GPIO->BRR = CAM_V_GPIO_PIN;
#endif

//#define Cam1Check() (cam1_state.sum / cam1_state.av_nb_sample > CAM1_STATE_ON_LEVEL)
#define Cam1Check() TRUE

extern bool_t   cam_shot;
extern bool_t   cam_alt_on;
extern uint16_t  kamera_interval;
extern int32_t  cam_min_alt;

void kamera_check_gps(void);
void kamera_init( void );
extern void periodic_task_kamera(void);

extern uint16_t last_known_shot;
extern uint8_t shot_period;
extern uint8_t stage_photo_num;

extern int32_t cam1_state_val;

#ifndef SITL
void CacheDel(uint16_t photo_nr);
#endif
#endif
