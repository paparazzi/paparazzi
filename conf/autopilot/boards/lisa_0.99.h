#ifndef CONFIG_LISA_V1_0_H
#define CONFIG_LISA_V1_0_H


#define AHB_CLK 72000000

/* Lisa uses an external clock instead of a crystal */
#define HSE_TYPE_EXT_CLK

/* Onboard LEDs */
#define LED_1_BANK 
#define LED_STP08



/* Default IMU b2 sensors connection */
#ifndef IMU_OVERRIDE_CHANNELS
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#define IMU_ACCEL_X_CHAN 5
#define IMU_ACCEL_Y_CHAN 3
#define IMU_ACCEL_Z_CHAN 4
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif /* not IMU_OVERRIDE_CHANNELS */

// FIXME, this is just to make it compile
#define POWER_SWITCH_LED 5



#endif /* CONFIG_LISA_V1_0_H */
