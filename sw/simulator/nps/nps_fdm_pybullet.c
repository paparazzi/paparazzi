/*
 * Copyright (C) 2021 Fabien-B <fabien-b@github.com> 
 *
 * This file is part of paparazzi. See LICENCE file.
 */

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>
#include <Python.h>

#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "state.h"

#define _WIDEN(x)  L ## x
#define WIDEN(x)   _WIDEN(x)

#ifndef PYTHON_EXEC
#define PYTHON_EXEC "python3"
#endif
MESSAGE("PyBullet using" VALUE(PYTHON_EXEC))

#ifndef PYBULLET_GUI
#define PYBULLET_GUI TRUE
#endif

#ifdef NPS_ACTUATORS_ORDER
  int actuators_order[ACTUATORS_NB] = NPS_ACTUATORS_ORDER;
#else
  #error "[PyBullet] missing NPS_ACTUATORS_ORDER define!"
#endif

#ifndef NPS_PYBULLET_MODULE
#define NPS_PYBULLET_MODULE "simple_quad_sim"
MESSAGE("NPS_PYBULLET_MODULE not defined, take 'simple_quad_sim' as default value.")
#endif


#ifndef NPS_PYBULLET_URDF
#define NPS_PYBULLET_URDF "robobee.urdf"
MESSAGE("NPS_PYBULLET_URDF not defined, take 'robobee.urdf' as default value.")
#endif


// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

//rotation from pybullet to pprz
struct DoubleQuat quat_to_pprz = {
  1/sqrt(2),
  0,
  0,
  1/sqrt(2),
};

// python object declarations
PyObject *fdm_module = NULL;
PyObject *bullet_fdm = NULL;

// static python related function declaration
static void py_check_status(PyConfig* config, PyStatus* status);
static void py_check(bool exit_on_error, int line_nb);
static void python_init(double dt);

// Static functions declaration
static void init_ltp(void);

static void get_pos(PyObject* ppos);
static void get_vel(PyObject* pvel);
static void get_acc(PyObject* pacc);

static void get_orient(PyObject* porient);
static void get_ang_vel(PyObject* pang_vel);
static void get_ang_acc(PyObject* pang_acc);

void nps_fdm_init(double dt)
{
  python_init(dt);

  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = dt;
  fdm.time = dt;

  fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  fdm.ltpprz_to_body_eulers.psi = 0.0;
  init_ltp();

  // run a first step to initialize all fdm fields
  double dummy_commands[] = {1, 2, 3, 4};
  nps_fdm_run_step(false, dummy_commands, 4);
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{
  // TODO create a np.array instead ?
  PyObject* pcmd = PyList_New(commands_nb);

  for(int i=0; i<commands_nb; i++) {
    int j = actuators_order[i];
    PyList_SetItem(pcmd, i, PyFloat_FromDouble(commands[j]));
  }

  if(commands[0] < 0) {
    return;
  }

  PyObject* ret = PyObject_CallMethod(bullet_fdm, "step", "(O)", pcmd);
  py_check(true, __LINE__);

  // borrowed references
  PyObject* ppos = PyDict_GetItemString(ret, "pos");
  PyObject* pvel = PyDict_GetItemString(ret, "vel");
  PyObject* pacc = PyDict_GetItemString(ret, "accel");
  
  PyObject* pquat = PyDict_GetItemString(ret, "quat");
  PyObject* pang_v = PyDict_GetItemString(ret, "ang_v");
  PyObject* pang_acc = PyDict_GetItemString(ret, "ang_accel");
  //PyObject* prpy = PyDict_GetItemString(ret, "rpy");
  py_check(true, __LINE__);


  get_pos(ppos);
  get_orient(pquat);
  get_vel(pvel);
  get_acc(pacc);
  get_ang_vel(pang_v);
  get_ang_acc(pang_acc);

  Py_XDECREF(pcmd);
  Py_XDECREF(ret);
}

static void get_pos(PyObject* ppos) {
    // get position from pybullet, in ENU, in the local frame
  struct EnuCoor_d enu_pos;
  enu_pos.x = PyFloat_AsDouble(PyTuple_GetItem(ppos, 0));
  enu_pos.y = PyFloat_AsDouble(PyTuple_GetItem(ppos, 1));
  enu_pos.z = PyFloat_AsDouble(PyTuple_GetItem(ppos, 2));
  py_check(true, __LINE__);

  /***********    positions     *************/
  VECT3_NED_OF_ENU(fdm.ltpprz_pos, enu_pos)
  ecef_of_enu_point_d(&fdm.ecef_pos, &ltpdef, &enu_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  fdm.hmsl = -fdm.ltpprz_pos.z;  //FIXME
  fdm.agl = fdm.hmsl;  //FIXME
}

static void get_vel(PyObject* pvel) {
    // get velocity from pybullet, in ENU wrt local frame (fixed/world)
  struct EnuCoor_d enu_vel;
  enu_vel.x = PyFloat_AsDouble(PyTuple_GetItem(pvel, 0));
  enu_vel.y = PyFloat_AsDouble(PyTuple_GetItem(pvel, 1));
  enu_vel.z = PyFloat_AsDouble(PyTuple_GetItem(pvel, 2));
  py_check(true, __LINE__);

    /***********     velocities     *************/
  /** velocity in LTP frame, wrt ECEF frame (NedCoor_d) */
  VECT3_NED_OF_ENU(fdm.ltp_ecef_vel, enu_vel)
  /** velocity in ECEF frame, wrt ECEF frame */
  ecef_of_enu_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &enu_vel);
  // /** velocity in body frame, wrt ECEF frame */
  // struct DoubleVect3 body_ecef_vel;   /* aka UVW */
  // /** velocity in ltppprz frame, wrt ECEF frame */
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel;
}

static void get_acc(PyObject* pacc) {

  struct EnuCoor_d enu_accel = {
    PyFloat_AsDouble(PyTuple_GetItem(pacc, 0)),
    PyFloat_AsDouble(PyTuple_GetItem(pacc, 1)),
    PyFloat_AsDouble(PyTuple_GetItem(pacc, 2)),
  };

  /** accel in ltppprz frame, wrt ECEF frame */
  VECT3_NED_OF_ENU(fdm.ltpprz_ecef_accel, enu_accel)
/** acceleration in LTP frame, wrt ECEF frame */
  VECT3_COPY(fdm.ltp_ecef_accel, fdm.ltpprz_ecef_accel)
  
  /** acceleration in ECEF frame, wrt ECEF frame */
  ecef_of_enu_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &enu_accel);

  /** acceleration in body frame as measured by an accelerometer (incl. gravity) */
  struct DoubleVect3 tmp = {
    fdm.ltp_ecef_accel.x,
    fdm.ltp_ecef_accel.y,
    fdm.ltp_ecef_accel.z - 10,
  };
  double_quat_vmult(&fdm.body_accel, &fdm.ltp_to_body_quat, &tmp);

  // /** acceleration in body frame, wrt ECEF frame */
  // fdm.body_ecef_accel;
  // /** acceleration in body frame, wrt ECI inertial frame */
  // fdm.body_inertial_accel;


}

static void get_orient(PyObject* pquat) {
    // /* attitude */

  struct DoubleQuat enu_quat = {
    PyFloat_AsDouble(PyTuple_GetItem(pquat, 3)),
    PyFloat_AsDouble(PyTuple_GetItem(pquat, 1)),
    PyFloat_AsDouble(PyTuple_GetItem(pquat, 0)),
    -PyFloat_AsDouble(PyTuple_GetItem(pquat, 2)),
  };

  double_quat_comp(&fdm.ltpprz_to_body_quat, &enu_quat, &quat_to_pprz);
  double_eulers_of_quat(&fdm.ltpprz_to_body_eulers, &fdm.ltpprz_to_body_quat);
  QUAT_COPY(fdm.ltp_to_body_quat, fdm.ltpprz_to_body_quat);
  EULERS_COPY(fdm.ltp_to_body_eulers, fdm.ltpprz_to_body_eulers);
}

static void get_ang_vel(PyObject* pang_vel) {


  struct DoubleVect3 sim_rates, pprz_rates;
  sim_rates.x = PyFloat_AsDouble(PyTuple_GetItem(pang_vel, 1));
  sim_rates.y = PyFloat_AsDouble(PyTuple_GetItem(pang_vel, 0));
  sim_rates.z = -PyFloat_AsDouble(PyTuple_GetItem(pang_vel, 2));

  double_quat_vmult(&pprz_rates, &fdm.ltp_to_body_quat, &sim_rates);

  fdm.body_inertial_rotvel.p = pprz_rates.x;
  fdm.body_inertial_rotvel.q = pprz_rates.y;
  fdm.body_inertial_rotvel.r = pprz_rates.z;
  fdm.body_ecef_rotvel = fdm.body_inertial_rotvel;


}

static void get_ang_acc(PyObject* pang_acc) {

  struct DoubleVect3 sim_rotaccel, pprz_rotaccel;
  sim_rotaccel.x = PyFloat_AsDouble(PyTuple_GetItem(pang_acc, 1));
  sim_rotaccel.y = PyFloat_AsDouble(PyTuple_GetItem(pang_acc, 0));
  sim_rotaccel.z = -PyFloat_AsDouble(PyTuple_GetItem(pang_acc, 2));

  double_quat_vmult(&pprz_rotaccel, &fdm.ltp_to_body_quat, &sim_rotaccel);

  fdm.body_inertial_rotaccel.p = pprz_rotaccel.x;
  fdm.body_inertial_rotaccel.q = pprz_rotaccel.y;
  fdm.body_inertial_rotaccel.r = pprz_rotaccel.z;
  fdm.body_ecef_rotaccel = fdm.body_inertial_rotaccel;
}



/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);
  llh_nav0.alt = NAV_ALT0 / 1000.0;

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;


// TODO vérifier tout ça
  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.; // accel data are already with the correct format

#ifdef AHRS_H_X
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file.")
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
#endif

}


/*****************************************************/
// Atmosphere function (we don't need that features) //
void nps_fdm_set_wind(double speed __attribute__((unused)),
                      double dir __attribute__((unused)))
{
}

void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)),
                          double wind_east __attribute__((unused)),
                          double wind_down __attribute__((unused)))
{
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

static void python_init(double dt) {
  PyStatus status;
  PyConfig config;
  PyConfig_InitPythonConfig(&config);

  // needed for the prints
  config.configure_c_stdio = 1;
  config.buffered_stdio = 0;

  status = PyConfig_SetString(&config, &config.program_name, WIDEN(PYTHON_EXEC));
  py_check_status(&config, &status);
  status = Py_InitializeFromConfig(&config);
  py_check_status(&config, &status);

  // add path to sys.path
  PyObject *sys_path = PySys_GetObject("path");
  PyObject *module_path = PyUnicode_FromString( PAPARAZZI_SRC "/sw/simulator/nps/pybullet");
  PyList_Append(sys_path, module_path);
  Py_DECREF(module_path);

  fdm_module = PyImport_ImportModule(NPS_PYBULLET_MODULE);
  py_check(true, __LINE__);

  PyObject* bullet_fdm_class = PyObject_GetAttrString(fdm_module, "BulletFDM");
  py_check(true, __LINE__);
  
  PyObject* fdm_ctor_args = Py_BuildValue("(d)", dt);
  PyObject* fdm_ctor_kwargs = Py_BuildValue("{siss}", "GUI", PYBULLET_GUI, "urdf", NPS_PYBULLET_URDF);
  bullet_fdm = PyObject_Call(bullet_fdm_class, fdm_ctor_args, fdm_ctor_kwargs);
  
  py_check(true, __LINE__);
  Py_DECREF(fdm_ctor_args);
  Py_DECREF(bullet_fdm_class);
}

static void py_check_status(PyConfig* config, PyStatus* status) {
   if (PyStatus_Exception(*status)) {
    printf("error\n");
    PyConfig_Clear(config);
    Py_ExitStatusException(*status);
    exit(1);
   }
}


static void py_check(bool exit_on_error, int line_nb) {
  if(PyErr_Occurred()) {
    printf("Error at line %d\n", line_nb);
    PyErr_Print();
    if(exit_on_error) {
      exit(1);
    }
  }
}
