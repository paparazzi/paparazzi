cdef extern from "stdint.h":
    ctypedef unsigned long int uintptr_t


cimport pprz_algebra_int_c
from algebra_int import Int32Quat, Int32Eulers, Int32Rates


cdef extern from "stabilization/stabilization_attitude_ref_quat_int.h":
    void stabilization_attitude_ref_init()
    void stabilization_attitude_ref_update()
    void stabilization_attitude_ref_enter()

    pprz_algebra_int_c.Int32Eulers stab_att_sp_euler
    pprz_algebra_int_c.Int32Quat   stab_att_sp_quat
    pprz_algebra_int_c.Int32Eulers stab_att_ref_euler
    pprz_algebra_int_c.Int32Quat   stab_att_ref_quat
    pprz_algebra_int_c.Int32Rates  stab_att_ref_rate
    pprz_algebra_int_c.Int32Rates  stab_att_ref_accel


sp_euler = Int32Eulers(<uintptr_t> &stab_att_sp_euler)
sp_quat = Int32Quat(<uintptr_t> &stab_att_sp_quat)
ref_euler = Int32Eulers(<uintptr_t> &stab_att_ref_euler)
ref_quat = Int32Quat(<uintptr_t> &stab_att_ref_quat)
ref_rate = Int32Rates(<uintptr_t> &stab_att_ref_rate)
ref_accel = Int32Rates(<uintptr_t> &stab_att_ref_accel)


def init():
    stabilization_attitude_ref_init()

def enter():
    stabilization_attitude_ref_enter()

def update():
    stabilization_attitude_ref_update()
