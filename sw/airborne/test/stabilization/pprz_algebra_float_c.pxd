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
