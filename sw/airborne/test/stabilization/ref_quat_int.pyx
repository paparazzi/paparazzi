cimport ref_quat_int
from pprz_algebra_int_c cimport Int32Quat
import numpy as np

cdef class RefQuatInt:
    cdef ref_quat_int.AttRefQuatInt ref
    cdef Int32Quat sp_quat

    def __cinit__(self):
        ref_quat_int.attitude_ref_quat_int_init(&self.ref)
        self.setpoint = [1.0, 0, 0, 0]

    cpdef update(self, dt, setpoint=None):
        if setpoint is not None:
            self.setpoint = setpoint
        ref_quat_int.attitude_ref_quat_int_update(&self.ref, &self.sp_quat, dt)
        #self.c_update(&self.sp_quat, dt)

    property setpoint:
        def __get__(self):
            quat = np.array([self.sp_quat.qi, self.sp_quat.qx, self.sp_quat.qy, self.sp_quat.qz], dtype='d')
            return quat / (1 << 15)
        def __set__(self, sp):
            self.sp_quat.qi = sp[0] * (1 << 15)
            self.sp_quat.qx = sp[1] * (1 << 15)
            self.sp_quat.qy = sp[2] * (1 << 15)
            self.sp_quat.qz = sp[3] * (1 << 15)

    property eulers:
        def __get__(self):
            euler = np.array([self.ref.euler.phi, self.ref.euler.theta, self.ref.euler.psi], dtype='d')
            return euler / (1 << 20)
