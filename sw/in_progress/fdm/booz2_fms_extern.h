#ifndef BOOZ2_FMS_EXTERN_H
#define BOOZ2_FMS_EXTERN_H

#define FMS_H_MODE_RATE     0
#define FMS_H_MODE_ATTITUDE 1
#define FMS_H_MODE_SPEED    2
#define FMS_H_MODE_POS      3

#define FMS_ATTITUDE_OF_DEG(_d) ((int32_t)((_d)*0.0000546))

#define FMS_V_MODE_DIRECT    0 
#define FMS_V_MODE_CLIMB     1 
#define FMS_V_MODE_HEIGHT    2 

//#define BOOZ2_SEND_IVY_FMS_COMMAND


#endif /* BOOZ2_FMS_EXTERN_H */
