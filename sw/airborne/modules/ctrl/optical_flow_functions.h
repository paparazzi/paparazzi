
#ifndef OPTICAL_FLOW_FUNCTIONS_H_
#define OPTICAL_FLOW_FUNCTIONS_H_

#include "std.h"

struct GainsPID {
	float P;             	       ///< P-gain for control
	float I;					   ///< I-gain for control
	float D;					   ///< D-gain for control
};

struct Errors {
	float err;                     ///< Current tracking error
	float previous_err;            ///< Previous tracking error
	float sum_err;                 ///< integration of the error for I-gain
	float d_err;                   ///< difference of error for the D-gain
};

struct NominalValues {
	float thrust;		           ///< nominal thrust around which the PID-control operates
	float phi;					   ///< nominal phi value of a stable hover
	float theta;				   ///< nominal theta value of a stable hover
};

struct OpticalFlowHoverControl {
	struct GainsPID PID;		   ///< The struct with the PID gains
	struct Errors errors;		   ///< The struct with the erros

	float ramp;		   	      	   ///< The ramp pused is increased with per dt

	float reduction_factor;        ///< Reduce the gain by this factor when oscillating

	float setpoint;     		   ///< setpoint for constant divergence/flow
	float cov_setpoint;            ///< for adaptive gain control, setpoint of the covariance (oscillations)
};

struct OpticalFlowHover {
	float divergence;              ///< Divergence estimate
	float flowX;			       ///< Flow estimate in X direction
	float flowY;			       ///< Flow estimate in Y direction
};

extern float PID_flow_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl);

#endif /* OPTICAL_FLOW_FUNCTIONS_H_ */
