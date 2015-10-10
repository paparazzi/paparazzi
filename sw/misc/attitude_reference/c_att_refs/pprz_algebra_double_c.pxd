cdef extern from "math/pprz_algebra_double.h":

    struct DoubleVect2
        double x
        double y

    struct DoubleVect3
        double x
        double y
        double z

    struct DoubleEulers:
        double phi
        double theta
        double psi

    struct DoubleQuat:
        double qi
        double qx
        double qy
        double qz

    struct DoubleRates:
        double p
        double q
        double r

    struct DoubleRMat:
        double m[3*3]
