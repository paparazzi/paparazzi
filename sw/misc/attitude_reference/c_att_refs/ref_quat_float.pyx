cimport ref_quat_float
from pprz_algebra_float_c cimport FloatQuat, float_eulers_of_quat, float_quat_of_eulers
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

    cdef set_omega(self, omega):
        ref_quat_float.attitude_ref_quat_float_set_omega_p(&self.ref, omega[0])
        ref_quat_float.attitude_ref_quat_float_set_omega_q(&self.ref, omega[1])
        ref_quat_float.attitude_ref_quat_float_set_omega_r(&self.ref, omega[2])

    cdef set_zeta(self, zeta):
        self.ref.model[0].zeta.p = zeta[0]
        self.ref.model[0].zeta.q = zeta[1]
        self.ref.model[0].zeta.r = zeta[2]

    property setpoint:
        def __get__(self):
            return np.array([self.sp_quat.qi, self.sp_quat.qx, self.sp_quat.qy, self.sp_quat.qz])
        def __set__(self, sp):
            self.sp_quat.qi = sp[0]
            self.sp_quat.qx = sp[1]
            self.sp_quat.qy = sp[2]
            self.sp_quat.qz = sp[3]

    property euler:
        def __get__(self):
            return np.array([self.ref.euler.phi, self.ref.euler.theta, self.ref.euler.psi])
        def __set__(self, euler):
            self.ref.euler.phi = euler[0]
            self.ref.euler.theta = euler[1]
            self.ref.euler.psi = euler[2]
            float_quat_of_eulers(&self.ref.quat, &self.ref.euler)

    property quat:
        def __get__(self):
            return np.array([self.ref.quat.qi, self.ref.quat.qx, self.ref.quat.qy, self.ref.quat.qz], dtype='d')
        def __set__(self, quat):
            self.ref.quat.qi = quat[0]
            self.ref.quat.qx = quat[1]
            self.ref.quat.qy = quat[2]
            self.ref.quat.qz = quat[3]
            float_eulers_of_quat(&self.ref.euler, &self.ref.quat)

    property rate:
        def __get__(self):
            return np.array([self.ref.rate.p, self.ref.rate.q, self.ref.rate.r])

    property accel:
        def __get__(self):
            return np.array([self.ref.accel.p, self.ref.accel.q, self.ref.accel.r])

    property zeta:
        def __get__(self):
            return np.array([self.ref.model[0].zeta.p, self.ref.model[0].zeta.q, self.ref.model[0].zeta.r])
        def __set__(self, zeta):
            if type(zeta) == float:
                self.set_zeta(zeta * np.ones(3))
            else:
                self.set_zeta(zeta)

    property omega:
        def __get__(self):
            return np.array([self.ref.model[0].omega.p, self.ref.model[0].omega.q, self.ref.model[0].omega.r])
        def __set__(self, omega):
            if type(omega) == float:
                self.set_omega(omega * np.ones(3))
            else:
                self.set_omega(omega)

    property sat_vel:
        def __get__(self):
            return np.array([self.ref.saturation.max_rate.p, self.ref.saturation.max_rate.q, self.ref.saturation.max_rate.r])
        def __set__(self, vel):
            if type(vel) == float:
                vel = vel * np.ones(3)
            self.ref.saturation.max_rate.p = vel[0]
            self.ref.saturation.max_rate.q = vel[1]
            self.ref.saturation.max_rate.r = vel[2]

    property sat_accel:
        def __get__(self):
            return np.array([self.ref.saturation.max_accel.p, self.ref.saturation.max_accel.q, self.ref.saturation.max_accel.r])
        def __set__(self, accel):
            if type(accel) == float:
                accel = accel * np.ones(3)
            self.ref.saturation.max_accel.p = accel[0]
            self.ref.saturation.max_accel.q = accel[1]
            self.ref.saturation.max_accel.r = accel[2]
