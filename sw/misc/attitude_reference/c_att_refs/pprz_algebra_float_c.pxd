cdef extern from "math/pprz_algebra_float.h":

    struct FloatVect2:
        float x
        float y

    struct FloatVect3:
        float x
        float y
        float z

    struct FloatEulers:
        float phi
        float theta
        float psi

    struct FloatQuat:
        float qi
        float qx
        float qy
        float qz

    struct FloatRates:
        float p
        float q
        float r

    struct FloatRMat:
        float m[3*3]

    void float_eulers_of_quat(FloatEulers *e, FloatQuat *q)
    void float_quat_of_eulers(FloatQuat *q, FloatEulers *e)
