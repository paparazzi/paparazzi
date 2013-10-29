cimport pprz_algebra_float_c

cdef class FloatEulers:
    cdef pprz_algebra_float_c.FloatEulers *ptr

cdef class FloatQuat:
    cdef pprz_algebra_float_c.FloatQuat *ptr

cdef class FloatRates:
    cdef pprz_algebra_float_c.FloatRates *ptr
