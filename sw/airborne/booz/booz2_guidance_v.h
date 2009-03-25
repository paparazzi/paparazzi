#ifndef BOOZ2_GUIDANCE_V
#define BOOZ2_GUIDANCE_V

#include "std.h"

#include "booz2_guidance_v_ref.h"
#include "booz2_guidance_v_adpt.h"

#define BOOZ2_GUIDANCE_V_MODE_KILL      0
#define BOOZ2_GUIDANCE_V_MODE_RC_DIRECT 1
#define BOOZ2_GUIDANCE_V_MODE_RC_CLIMB  2
#define BOOZ2_GUIDANCE_V_MODE_CLIMB     3
#define BOOZ2_GUIDANCE_V_MODE_HOVER     4
#define BOOZ2_GUIDANCE_V_MODE_NAV       5

extern uint8_t booz2_guidance_v_mode;

extern int32_t booz2_guidance_v_z_sp;
extern int32_t booz2_guidance_v_zd_sp;
extern int32_t booz2_guidance_v_z_ref;
extern int32_t booz2_guidance_v_zd_ref;
extern int32_t booz2_guidance_v_zdd_ref;
extern int32_t booz2_guidance_v_z_sum_err;
extern int32_t booz2_guidance_v_ff_cmd;
extern int32_t booz2_guidance_v_fb_cmd;
extern int32_t booz2_guidance_v_delta_t;

extern int32_t booz2_guidance_v_kp;
extern int32_t booz2_guidance_v_kd;

extern void booz2_guidance_v_init(void);
extern void booz2_guidance_v_read_rc(void);
extern void booz2_guidance_v_mode_changed(uint8_t new_mode);
extern void booz2_guidance_v_notify_in_flight(bool_t in_flight);
extern void booz2_guidance_v_run(bool_t in_flight);

#endif /* BOOZ2_GUIDANCE_V */
