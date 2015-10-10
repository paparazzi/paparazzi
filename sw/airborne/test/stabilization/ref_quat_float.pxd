from pprz_algebra_float_c cimport FloatQuat, FloatEulers, FloatRates

cdef extern from "stabilization/stabilization_attitude_ref_quat_float.h":
    #struct FloatRefModel:
    #    FloatRates omega
    #    FloatRates zeta

    struct AttRefQuatFloat:
        FloatEulers euler
        FloatQuat   quat
        FloatRates  rate
        FloatRates  accel

    void attitude_ref_quat_float_init(AttRefQuatFloat *ref)
    void attitude_ref_quat_float_enter(AttRefQuatFloat *ref, float psi)
    void attitude_ref_quat_float_update(AttRefQuatFloat *ref, FloatQuat *sp_quat, float dt)

    void attitude_ref_quat_float_set_omega_p(AttRefQuatFloat *ref, float omega)
    void attitude_ref_quat_float_set_omega_q(AttRefQuatFloat *ref, float omega)
    void attitude_ref_quat_float_set_omega_r(AttRefQuatFloat *ref, float omega)


#cdef class RefQuatFloat:
#    cdef AttRefQuatFloat ref
