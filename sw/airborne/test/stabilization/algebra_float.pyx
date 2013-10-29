cdef extern from "stdint.h":
    ctypedef unsigned long int uintptr_t

cimport pprz_algebra_float_c
cimport algebra_float
cimport numpy as np
import numpy as np


cdef class FloatRates:
    # already declared in pxd
    #cdef pprz_algebra_float_c.FloatRates *ptr

    def __cinit__(self, uintptr_t p):
        self.ptr = <pprz_algebra_float_c.FloatRates*> p

    property array:
        def __get__(self):
            return np.asarray([self.ptr[0].p, self.ptr[0].q, self.ptr[0].r])
        def __set__(self, np.ndarray a):
            self.ptr[0].p = a[0]
            self.ptr[0].q = a[1]
            self.ptr[0].r = a[2]

    property p:
        def __get__(self):
            return self.ptr[0].p
        def __set__(self, v):
            self.ptr[0].p = v

    property q:
        def __get__(self):
            return self.ptr[0].q
        def __set__(self, v):
            self.ptr[0].q = v

    property r:
        def __get__(self):
            return self.ptr[0].r
        def __set__(self, v):
            self.ptr[0].r = v


cdef class FloatEulers:
    # already declared in pxd
    #cdef pprz_algebra_float_c.FloatEulers *ptr

    def __cinit__(self, uintptr_t p):
        self.ptr = <pprz_algebra_float_c.FloatEulers*> p

    property array:
        def __get__(self):
            return np.asarray([self.ptr[0].phi, self.ptr[0].theta,
                               self.ptr[0].psi])
        def __set__(self, np.ndarray a):
            self.ptr[0].phi = a[0]
            self.ptr[0].theta = a[1]
            self.ptr[0].psi = a[2]

    property phi:
        def __get__(self):
            return self.ptr[0].phi
        def __set__(self, v):
            self.ptr[0].phi = v

    property theta:
        def __get__(self):
            return self.ptr[0].theta
        def __set__(self, v):
            self.ptr[0].theta = v

    property psi:
        def __get__(self):
            return self.ptr[0].psi
        def __set__(self, v):
            self.ptr[0].psi = v


cdef class FloatQuat:
    # already declared in pxd
    #cdef pprz_algebra_float_c.FloatQuat *ptr

    def __cinit__(self, uintptr_t p):
        self.ptr = <pprz_algebra_float_c.FloatQuat*> p

    property array:
        def __get__(self):
            return np.asarray([self.ptr[0].qi, self.ptr[0].qx,
                               self.ptr[0].qy, self.ptr[0].qz])
        def __set__(self, np.ndarray a):
            self.ptr[0].qi = a[0]
            self.ptr[0].qx = a[1]
            self.ptr[0].qy = a[2]
            self.ptr[0].qz = a[3]

    property qi:
        def __get__(self):
            return self.ptr[0].qi
        def __set__(self, v):
            self.ptr[0].qi = v

    property qx:
        def __get__(self):
            return self.ptr[0].qx
        def __set__(self, v):
            self.ptr[0].qx = v

    property qy:
        def __get__(self):
            return self.ptr[0].qy
        def __set__(self, v):
            self.ptr[0].qy = v

    property qz:
        def __get__(self):
            return self.ptr[0].qz
        def __set__(self, v):
            self.ptr[0].qz = v
