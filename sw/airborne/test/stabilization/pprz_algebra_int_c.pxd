from libc.stdint cimport uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t

cdef extern from "math/pprz_algebra_int.h":

    struct Int32Vect2:
        int32_t x
        int32_t y

    struct Int32Vect3:
        int32_t x
        int32_t y
        int32_t z

    struct Int32Eulers:
        int32_t phi
        int32_t theta
        int32_t psi

    struct Int32Quat:
        int32_t qi
        int32_t qx
        int32_t qy
        int32_t qz

    struct Int32Rates:
        int32_t p
        int32_t q
        int32_t r

    struct Int32RMat:
        int32_t m[3*3]

cdef extern from "math/pprz_trig_int.h":
    int16_t pprz_trig_int[]
