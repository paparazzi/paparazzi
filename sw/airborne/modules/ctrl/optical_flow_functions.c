
#include "optical_flow_functions.h"

#ifndef OFH_MAXBANK
#define OFH_MAXBANK 10.0
#endif

#ifndef LP_CONST
#define LP_CONST 0.5 // Backup value
#endif

/**
 * Determine and set the desired angle for constant flow control
 * @param[out] desired angle
 * @param[in] dt: time difference since last update
 * @param[in] *of_hover_ctrl: OpticalFlowHoverControl structure
 */
float PID_flow_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl)
{
	float des_angle = 0;

	// update the controller errors:
	float lp_factor = dt / LP_CONST;
	Bound(lp_factor, 0.f, 1.f);

	// maintain the controller errors:
	of_hover_ctrl->errors.sum_err += of_hover_ctrl->errors.err;
	of_hover_ctrl->errors.d_err += (((of_hover_ctrl->errors.err - of_hover_ctrl->errors.previous_err) / dt) - of_hover_ctrl->errors.d_err) * lp_factor;
	of_hover_ctrl->errors.previous_err = of_hover_ctrl->errors.err;

	// compute the desired angle
	des_angle = Max(-OFH_MAXBANK,Min(of_hover_ctrl->PID.P * of_hover_ctrl->errors.err + of_hover_ctrl->PID.I * of_hover_ctrl->errors.sum_err + of_hover_ctrl->PID.D * of_hover_ctrl->errors.d_err,OFH_MAXBANK));

	return des_angle;
}
