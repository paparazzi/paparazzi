cdef extern from "stdint.h":
    ctypedef unsigned long int uintptr_t


import numpy as np
cimport numpy as np

cimport pprz_algebra_float_c
from algebra_float import FloatQuat, FloatEulers, FloatRates


cdef extern from "stabilization/stabilization_attitude_ref_quat_float.h":
    void stabilization_attitude_ref_init()
    void stabilization_attitude_ref_update()
    void stabilization_attitude_ref_enter()
    void stabilization_attitude_ref_schedule(int idx)

    pprz_algebra_float_c.FloatEulers stab_att_sp_euler
    pprz_algebra_float_c.FloatQuat   stab_att_sp_quat
    pprz_algebra_float_c.FloatEulers stab_att_ref_euler
    pprz_algebra_float_c.FloatQuat   stab_att_ref_quat
    pprz_algebra_float_c.FloatRates  stab_att_ref_rate
    pprz_algebra_float_c.FloatRates  stab_att_ref_accel


sp_euler = FloatEulers(<uintptr_t> &stab_att_sp_euler)
sp_quat = FloatQuat(<uintptr_t> &stab_att_sp_quat)
ref_euler = FloatEulers(<uintptr_t> &stab_att_ref_euler)
ref_quat = FloatQuat(<uintptr_t> &stab_att_ref_quat)
ref_rate = FloatRates(<uintptr_t> &stab_att_ref_rate)
ref_accel = FloatRates(<uintptr_t> &stab_att_ref_accel)


def init():
    stabilization_attitude_ref_init()

def enter():
    stabilization_attitude_ref_enter()

def update():
    stabilization_attitude_ref_update()

"""
# test some different set methods
def set_sp_euler(**kwds):
    global stab_att_sp_euler
    cdef pprz_algebra_float_c.FloatEulers c_sp = kwds
    stab_att_sp_euler = c_sp

def set_sp_euler(phi=0.0, theta=0.0, psi=0.0):
    global sp_euler
    sp_euler.phi = phi
    sp_euler.theta = theta
    sp_euler.psi = psi

def set_sp_quat(**kwds):
    global stab_att_sp_quat
    cdef pprz_algebra_float_c.FloatQuat c_sp = kwds
    stab_att_sp_quat = c_sp
"""
