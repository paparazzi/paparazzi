/* rover_steering_guidance.c */

#define AUTOPILOT_CORE_GUIDANCE_C

#include "firmwares/rover/guidance/rover_guidance_steering.h"
#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"
#include "state.h"

void rover_guidance_steering_init(void)
{
  // from code generation
  autopilot_core_guidance_init();
}

void rover_guidance_steering_periodic(void)
{
  // from code generation
  autopilot_core_guidance_periodic_task();
}

