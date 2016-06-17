cimport ref_quat_int
from pprz_algebra_int_c cimport Int32Quat, Int32Eulers, int32_quat_of_eulers, int32_eulers_of_quat
import numpy as np

REF_ACCEL_FRAC = 12
REF_RATE_FRAC = 16
INT32_ANGLE_FRAC = 12
INT32_QUAT_FRAC = 15

cdef class RefQuatInt:
    cdef ref_quat_int.AttRefQuatInt ref
    cdef Int32Quat sp_quat

    def __cinit__(self):
        ref_quat_int.attitude_ref_quat_int_init(&self.ref)
        self.setpoint = [1.0, 0, 0, 0]

    cpdef Int32Quat quat_bfp_of_real(self, quat):
        cdef Int32Quat quat_i
        quat_i.qi = quat[0] * (1 << INT32_QUAT_FRAC)
        quat_i.qx = quat[1] * (1 << INT32_QUAT_FRAC)
        quat_i.qy = quat[2] * (1 << INT32_QUAT_FRAC)
        quat_i.qz = quat[3] * (1 << INT32_QUAT_FRAC)
        return quat_i

    cpdef Int32Eulers euler_bfp_of_real(self, euler):
        cdef Int32Eulers euler_i
        euler_i.phi = euler[0] * (1 << INT32_ANGLE_FRAC)
        euler_i.theta = euler[1] * (1 << INT32_ANGLE_FRAC)
        euler_i.psi = euler[2] * (1 << INT32_ANGLE_FRAC)
        return euler_i

    cpdef update(self, dt, setpoint=None):
        if setpoint is not None:
            self.setpoint = setpoint
        ref_quat_int.attitude_ref_quat_int_update(&self.ref, &self.sp_quat, dt)

    cdef set_omega(self, omega):
        cdef FloatRates c_omega
        c_omega.p = omega[0]
        c_omega.q = omega[1]
        c_omega.r = omega[2]
        ref_quat_int.attitude_ref_quat_int_set_omega(&self.ref, &c_omega)

    cdef set_zeta(self, zeta):
        cdef FloatRates c_zeta
        c_zeta.p = zeta[0]
        c_zeta.q = zeta[1]
        c_zeta.r = zeta[2]
        ref_quat_int.attitude_ref_quat_int_set_zeta(&self.ref, &c_zeta)

    property setpoint:
        def __get__(self):
            quat = np.array([self.sp_quat.qi, self.sp_quat.qx, self.sp_quat.qy, self.sp_quat.qz], dtype='d')
            return quat / (1 << INT32_QUAT_FRAC)
        def __set__(self, sp):
            self.sp_quat = self.quat_bfp_of_real(sp)

    property euler:
        def __get__(self):
            euler = np.array([self.ref.euler.phi, self.ref.euler.theta, self.ref.euler.psi], dtype='d')
            return euler / (1 << INT32_ANGLE_FRAC)
        def __set__(self, euler):
            self.ref.euler = self.euler_bfp_of_real(euler)
            int32_quat_of_eulers(&self.ref.quat, &self.ref.euler)

    property quat:
        def __get__(self):
            quat = np.array([self.ref.quat.qi, self.ref.quat.qx, self.ref.quat.qy, self.ref.quat.qz], dtype='d')
            return quat / (1 << INT32_QUAT_FRAC)
        def __set__(self, quat):
            self.ref.quat = self.quat_bfp_of_real(quat)
            int32_eulers_of_quat(&self.ref.euler, &self.ref.quat)

    property rate:
        def __get__(self):
            return np.array([self.ref.rate.p, self.ref.rate.q, self.ref.rate.r], dtype='d') / (1 << REF_RATE_FRAC)

    property accel:
        def __get__(self):
            return np.array([self.ref.accel.p, self.ref.accel.q, self.ref.accel.r], dtype='d') / (1 << REF_ACCEL_FRAC)

    property zeta:
        def __get__(self):
            return np.array([self.ref.model.zeta.p, self.ref.model.zeta.q, self.ref.model.zeta.r])
        def __set__(self, zeta):
            if type(zeta) == float:
                self.set_zeta(zeta * np.ones(3))
            else:
                self.set_zeta(zeta)

    property omega:
        def __get__(self):
            return np.array([self.ref.model.omega.p, self.ref.model.omega.q, self.ref.model.omega.r])
        def __set__(self, omega):
            if type(omega) == float:
                self.set_omega(omega * np.ones(3))
            else:
                self.set_omega(omega)

    property sat_vel:
        def __get__(self):
            max_rate = [self.ref.saturation.max_rate.p, self.ref.saturation.max_rate.q, self.ref.saturation.max_rate.r]
            return np.array(max_rate, dtype='d') / (1 << REF_RATE_FRAC)
        def __set__(self, vel):
            if type(vel) == float:
                vel = vel * np.ones(3)
            ref_quat_int.attitude_ref_quat_int_set_max_p(&self.ref, vel[0])
            ref_quat_int.attitude_ref_quat_int_set_max_q(&self.ref, vel[1])
            ref_quat_int.attitude_ref_quat_int_set_max_r(&self.ref, vel[2])

    property sat_accel:
        def __get__(self):
            max_accel = [self.ref.saturation.max_accel.p, self.ref.saturation.max_accel.q, self.ref.saturation.max_accel.r]
            return np.array(max_accel, dtype='d') / (1 << REF_ACCEL_FRAC)
        def __set__(self, accel):
            if type(accel) == float:
                accel = accel * np.ones(3)
            ref_quat_int.attitude_ref_quat_int_set_max_pdot(&self.ref, accel[0])
            ref_quat_int.attitude_ref_quat_int_set_max_qdot(&self.ref, accel[1])
            ref_quat_int.attitude_ref_quat_int_set_max_rdot(&self.ref, accel[2])
