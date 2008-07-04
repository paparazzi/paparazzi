#include "booz_sensors_model.h"

#include BSM_PARAMS
#include "booz_sensors_model_utils.h"
#include "booz_flight_model.h"

bool_t booz_sensors_model_mag_available() {
  if (bsm.mag_available) {
    bsm.mag_available = FALSE;
    return TRUE;
  }
  return FALSE;
}


void booz_sensors_model_mag_init( double time ) {

  bsm.mag = v_get(AXIS_NB);
  bsm.mag->ve[AXIS_X] = 0.;
  bsm.mag->ve[AXIS_Y] = 0.;
  bsm.mag->ve[AXIS_Z] = 0.;
  bsm.mag_resolution = BSM_MAG_RESOLUTION;

  bsm.mag_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.mag_sensitivity);
  bsm.mag_sensitivity->me[AXIS_X][AXIS_X] = BSM_MAG_SENSITIVITY_XX;
  bsm.mag_sensitivity->me[AXIS_Y][AXIS_Y] = BSM_MAG_SENSITIVITY_YY;
  bsm.mag_sensitivity->me[AXIS_Z][AXIS_Z] = BSM_MAG_SENSITIVITY_ZZ;

  bsm.mag_neutral = v_get(AXIS_NB);
  bsm.mag_neutral->ve[AXIS_X] = BSM_MAG_NEUTRAL_X;
  bsm.mag_neutral->ve[AXIS_Y] = BSM_MAG_NEUTRAL_Y;
  bsm.mag_neutral->ve[AXIS_Z] = BSM_MAG_NEUTRAL_Z;

  bsm.mag_noise_std_dev = v_get(AXIS_NB);
  bsm.mag_noise_std_dev->ve[AXIS_X] = BSM_MAG_NOISE_STD_DEV_X;
  bsm.mag_noise_std_dev->ve[AXIS_Y] = BSM_MAG_NOISE_STD_DEV_Y;
  bsm.mag_noise_std_dev->ve[AXIS_Z] = BSM_MAG_NOISE_STD_DEV_Z;

  bsm.mag_next_update = time;
  bsm.mag_available = FALSE;

}

void booz_sensors_model_mag_run( double time, MAT* dcm ) {
  if (time < bsm.mag_next_update)
    return;
  
  /* rotate h to body frame */
  static VEC *h_body = VNULL;
  h_body = v_resize(h_body, AXIS_NB);
  h_body = mv_mlt(dcm, bfm.h_earth, h_body);
  
  bsm.mag = mv_mlt(bsm.mag_sensitivity, h_body, bsm.mag); 
  bsm.mag = v_add(bsm.mag, bsm.mag_neutral, bsm.mag); 

  /* compute mag error readings */  
  static VEC *mag_error = VNULL;
  mag_error = v_resize(mag_error, AXIS_NB);
  mag_error = v_zero(mag_error);
  /* add a gaussian noise */
  mag_error = v_add_gaussian_noise(mag_error, bsm.mag_noise_std_dev, mag_error);
  
  mag_error->ve[AXIS_X] = mag_error->ve[AXIS_X] * bsm.mag_sensitivity->me[AXIS_X][AXIS_X];
  mag_error->ve[AXIS_Y] = mag_error->ve[AXIS_Y] * bsm.mag_sensitivity->me[AXIS_Y][AXIS_Y];
  mag_error->ve[AXIS_Z] = mag_error->ve[AXIS_Z] * bsm.mag_sensitivity->me[AXIS_Z][AXIS_Z];

  /* add error */
  bsm.mag =  v_add(bsm.mag, mag_error, bsm.mag); 

  //  printf("h body %f %f %f\n", h_body->ve[AXIS_X], h_body->ve[AXIS_Y], h_body->ve[AXIS_Z]);
  //  printf("mag %f %f %f\n", bsm.mag->ve[AXIS_X], bsm.mag->ve[AXIS_Y], bsm.mag->ve[AXIS_Z]);
  /* round signal to account for adc discretisation */  
  RoundSensor(bsm.mag);

  bsm.mag_next_update += BSM_MAG_DT;
  bsm.mag_available = TRUE;
}

