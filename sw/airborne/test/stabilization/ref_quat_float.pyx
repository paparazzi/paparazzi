cimport ref_quat_float
from pprz_algebra_float_c cimport FloatQuat
import numpy as np

cdef class RefQuatFloat:
    cdef ref_quat_float.AttRefQuatFloat ref
    cdef FloatQuat sp_quat

    def __cinit__(self):
        ref_quat_float.attitude_ref_quat_float_init(&self.ref)
        self.setpoint = [1.0, 0, 0, 0]

    cpdef update(self, dt, setpoint=None):
        if setpoint is not None:
            self.setpoint = setpoint
        ref_quat_float.attitude_ref_quat_float_update(&self.ref, &self.sp_quat, dt)

    property setpoint:
        def __get__(self):
            return np.array([self.sp_quat.qi, self.sp_quat.qx, self.sp_quat.qy, self.sp_quat.qz])
        def __set__(self, sp):
            self.sp_quat.qi = sp[0]
            self.sp_quat.qx = sp[1]
            self.sp_quat.qy = sp[2]
            self.sp_quat.qz = sp[3]

    property eulers:
        def __get__(self):
            return np.array([self.ref.euler.phi, self.ref.euler.theta, self.ref.euler.psi])
