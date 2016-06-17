from libc.stdint cimport int32_t

from pprz_algebra_int_c cimport Int32Quat, Int32Eulers, Int32Rates
from pprz_algebra_float_c cimport FloatRates

cdef extern from "stabilization/stabilization_attitude_ref_quat_int.h":
    struct IntRefModel:
        FloatRates omega
        FloatRates zeta

    struct Int32RefSat:
        Int32Rates max_rate
        Int32Rates max_accel

    struct AttRefQuatInt:
        Int32Eulers euler
        Int32Quat   quat
        Int32Rates  rate
        Int32Rates  accel
        IntRefModel  model
        Int32RefSat saturation

    void attitude_ref_quat_int_init(AttRefQuatInt *ref)
    void attitude_ref_quat_int_enter(AttRefQuatInt *ref, int32_t psi)
    void attitude_ref_quat_int_update(AttRefQuatInt *ref, Int32Quat *sp_quat, float dt)

    void attitude_ref_quat_int_set_omega(AttRefQuatInt *ref, FloatRates *omega)
    void attitude_ref_quat_int_set_zeta(AttRefQuatInt *ref, FloatRates *zeta)

    void attitude_ref_quat_int_set_max_p(AttRefQuatInt *ref, float max_p);
    void attitude_ref_quat_int_set_max_q(AttRefQuatInt *ref, float max_q);
    void attitude_ref_quat_int_set_max_r(AttRefQuatInt *ref, float max_r);
    void attitude_ref_quat_int_set_max_pdot(AttRefQuatInt *ref, float max_pdot);
    void attitude_ref_quat_int_set_max_qdot(AttRefQuatInt *ref, float max_qdot);
    void attitude_ref_quat_int_set_max_rdot(AttRefQuatInt *ref, float max_rdot);
