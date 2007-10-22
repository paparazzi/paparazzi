#ifndef PT_ANT_MOTORS_H
#define PT_ANT_MOTORS_H

#include "std.h"
#include "LPC21xx.h"
#include "sys_time.h"

extern void pt_ant_motors_init(void);

extern float pt_ant_motors_y_power;
extern float pt_ant_motors_z_power;

#define pt_ant_motors_SetYPower(p) {			\
    pt_ant_motors_y_power = p;				\
    int32_t len = p * MOT_CHOP_PERIOD;			\
    if (p>=0)						\
      SetBit(MOT_Y_DIR_CLR_REG,  MOT_Y_DIR_PIN);	\
    else {						\
      SetBit(MOT_Y_DIR_SET_REG,  MOT_Y_DIR_PIN);	\
      len = -len;					\
    }							\
    MOT_Y_PWM_REG = len;				\
    PWMLER = MOT_Y_PWM_LATCH;				\
  }


#define pt_ant_motors_SetZPower(p) { \
    pt_ant_motors_z_power = p;				\
    int32_t len = p * MOT_CHOP_PERIOD;			\
    if (p>=0)						\
      SetBit(MOT_Z_DIR_CLR_REG,  MOT_Z_DIR_PIN);	\
    else {						\
      SetBit(MOT_Z_DIR_SET_REG,  MOT_Z_DIR_PIN);	\
      len = -len;					\
    }							\
    MOT_Z_PWM_REG = len;				\
    PWMLER = MOT_Z_PWM_LATCH;				\
  }

#define MOT_Z_PWM_PINSEL PINSEL0 
#define MOT_Z_PWM_PINSEL_VAL 2
#define MOT_Z_PWM_PINSEL_BIT 14
#define MOT_Z_PWM_ENA PWMPCR_ENA2
#define MOT_Z_PWM_LATCH PWMLER_LATCH2
#define MOT_Z_PWM_REG PWMMR2
#define MOT_Z_DIR_DIR_REG IO0DIR
#define MOT_Z_DIR_CLR_REG IO0CLR
#define MOT_Z_DIR_SET_REG IO0SET
#define MOT_Z_DIR_PIN 15


#define MOT_Y_PWM_PINSEL PINSEL1 
#define MOT_Y_PWM_PINSEL_VAL 1
#define MOT_Y_PWM_PINSEL_BIT 10
#define MOT_Y_PWM_ENA PWMPCR_ENA5
#define MOT_Y_PWM_LATCH PWMLER_LATCH5
#define MOT_Y_PWM_REG PWMMR5
#define MOT_Y_DIR_DIR_REG IO0DIR
#define MOT_Y_DIR_CLR_REG IO0CLR
#define MOT_Y_DIR_SET_REG IO0SET
#define MOT_Y_DIR_PIN 19

/* 5KHz */
#define MOT_CHOP_PERIOD SYS_TICS_OF_USEC(1000)

#endif /* PT_ANT_MOTORS_H */
