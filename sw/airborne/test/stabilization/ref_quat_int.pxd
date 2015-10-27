from libc.stdint cimport int32_t

from pprz_algebra_int_c cimport Int32Quat, Int32Eulers, Int32Rates
from pprz_algebra_float_c cimport FloatRates

cdef extern from "stabilization/stabilization_attitude_ref_quat_int.h":
    struct FloatRefModel:
        FloatRates omega
        FloatRates zeta

    struct AttRefQuatInt:
        Int32Eulers euler
        Int32Quat   quat
        Int32Rates  rate
        Int32Rates  accel
        FloatRefModel  model

    void attitude_ref_quat_int_init(AttRefQuatInt *ref)
    void attitude_ref_quat_int_enter(AttRefQuatInt *ref, int32_t psi)
    void attitude_ref_quat_int_update(AttRefQuatInt *ref, Int32Quat *sp_quat, float dt)

    void attitude_ref_quat_int_set_omega(AttRefQuatInt *ref, FloatRates *omega)
    void attitude_ref_quat_int_set_zeta(AttRefQuatInt *ref, FloatRates *zeta)
