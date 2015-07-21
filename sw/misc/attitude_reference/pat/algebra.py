import math
import numpy as np
import numpy.linalg

# euler angles
e_phi = 0
e_theta = 1
e_psi = 2
e_size = 3

# rotational velocities
r_p = 0
r_q = 1
r_r = 2
r_size = 3

# quaternions
q_i = 0
q_x = 1
q_y = 2
q_z = 3
q_size = 4

# vectors
v_x = 0
v_y = 1
v_z = 2
v_size = 3


def rmat_of_euler(e):
    cphi = math.cos(e[e_phi])
    sphi = math.sin(e[e_phi])
    ctheta = math.cos(e[e_theta])
    stheta = math.sin(e[e_theta])
    cpsi = math.cos(e[e_psi])
    spsi = math.sin(e[e_psi])
    return np.array([[ctheta * cpsi, ctheta * spsi, -stheta],
                     [sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi, sphi * ctheta],
                     [cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta]])


def euler_of_quat(q):
    dcm00 = 1.0 - 2. * (q[q_y] ** 2 + q[q_z] ** 2)
    dcm01 = 2. * (q[q_x] * q[q_y] + q[q_i] * q[q_z])
    dcm02 = 2. * (q[q_x] * q[q_z] - q[q_i] * q[q_y])
    dcm12 = 2. * (q[q_y] * q[q_z] + q[q_i] * q[q_x])
    dcm22 = 1.0 - 2. * (q[q_x] ** 2 + q[q_y] ** 2)
    return [math.atan2(dcm12, dcm22),
            -math.asin(dcm02),
            math.atan2(dcm01, dcm00)]


def euler_of_quat_v(q):
    nr, nc = q.shape
    eu = np.zeros((nr, e_size))
    for i in range(0, nr):
        eu[i] = euler_of_quat(q[i])
    return eu


#
# bad (earth relative)
#


def rmat_of_euler_1(e):
    cphi = math.cos(e[e_phi])
    sphi = math.sin(e[e_phi])
    ctheta = math.cos(e[e_theta])
    stheta = math.sin(e[e_theta])
    cpsi = math.cos(e[e_psi])
    spsi = math.sin(e[e_psi])
    return np.array([[ctheta * cpsi, -ctheta * spsi, -stheta],
                     [-sphi * stheta * cpsi + cphi * spsi, sphi * stheta * spsi + cphi * cpsi, -sphi * ctheta],
                     [cphi * stheta * cpsi + sphi * spsi, -cphi * stheta * spsi + sphi * cpsi, cphi * ctheta]])


def rvel_of_eulerd_1(eu, eud):
    sph = math.sin(eu[e_phi])
    cph = math.cos(eu[e_phi])
    sth = math.sin(eu[e_theta])
    cth = math.cos(eu[e_theta])
    a = 2 * (cph ** 2) - 1
    return [eud[e_phi] - sth * eud[e_psi],
            cph / a * eud[e_theta] - sph * cth / a * eud[e_psi],
            -sph / a * eud[e_theta] + cph * cth / a * eud[e_psi]]


def euler_derivatives_1(eu, om):
    sph = math.sin(eu[e_phi])
    cph = math.cos(eu[e_phi])
    tth = math.tan(eu[e_theta])
    cth = math.cos(eu[e_theta])
    return [om[r_p] + sph * tth * om[r_q] + cph * tth * om[r_r],
            cph * om[r_q] + sph * om[r_r],
            sph / cth * om[r_q] + cph / cth * om[r_r]]


#
# good (body relative)
#
def rvel_of_eulerd(eu, eud):
    sph = math.sin(eu[e_phi])
    cph = math.cos(eu[e_phi])
    sth = math.sin(eu[e_theta])
    cth = math.cos(eu[e_theta])
    return [eud[e_phi] - sth * eud[e_psi],
            cph * eud[e_theta] + sph * cth * eud[e_psi],
            -sph * eud[e_theta] + cph * cth * eud[e_psi]]


def euler_derivatives(eu, om):
    sph = math.sin(eu[e_phi])
    cph = math.cos(eu[e_phi])
    tth = math.tan(eu[e_theta])
    cth = math.cos(eu[e_theta])

    return [om[r_p] + sph * tth * om[r_q] + cph * tth * om[r_r],
            cph * om[r_q] - sph * om[r_r],
            sph / cth * om[r_q] + cph / cth * om[r_r]]


#
# Quaternions
#

def quat_zero():
    return np.array([1., 0., 0., 0.])


def quat_of_euler(e):
    s_phi2 = math.sin(e[e_phi] / 2)
    c_phi2 = math.cos(e[e_phi] / 2)
    s_theta2 = math.sin(e[e_theta] / 2)
    c_theta2 = math.cos(e[e_theta] / 2)
    s_psi2 = math.sin(e[e_psi] / 2)
    c_psi2 = math.cos(e[e_psi] / 2)

    return [c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2,
            -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2,
            c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2,
            c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2]


def quat_of_axis_angle(u, a):
    sa2 = math.sin(a / 2.)
    ca2 = math.cos(a / 2.)
    return [ca2, sa2 * u[0], sa2 * u[1], sa2 * u[2]]


def quat_comp(a2b, b2c):
    return [
        a2b[q_i] * b2c[q_i] - a2b[q_x] * b2c[q_x] - a2b[q_y] * b2c[q_y] - a2b[q_z] * b2c[q_z],
        a2b[q_i] * b2c[q_x] + a2b[q_x] * b2c[q_i] + a2b[q_y] * b2c[q_z] - a2b[q_z] * b2c[q_y],
        a2b[q_i] * b2c[q_y] - a2b[q_x] * b2c[q_z] + a2b[q_y] * b2c[q_i] + a2b[q_z] * b2c[q_x],
        a2b[q_i] * b2c[q_z] + a2b[q_x] * b2c[q_y] - a2b[q_y] * b2c[q_x] + a2b[q_z] * b2c[q_i]]


def quat_inv_comp(a2b, a2c):
    return [
        a2b[q_i] * a2c[q_i] + a2b[q_x] * a2c[q_x] + a2b[q_y] * a2c[q_y] + a2b[q_z] * a2c[q_z],
        a2b[q_i] * a2c[q_x] - a2b[q_x] * a2c[q_i] - a2b[q_y] * a2c[q_z] + a2b[q_z] * a2c[q_y],
        a2b[q_i] * a2c[q_y] + a2b[q_x] * a2c[q_z] - a2b[q_y] * a2c[q_i] - a2b[q_z] * a2c[q_x],
        a2b[q_i] * a2c[q_z] - a2b[q_x] * a2c[q_y] + a2b[q_y] * a2c[q_x] - a2b[q_z] * a2c[q_i]]


def quat_derivative(q, om):
    return [
        0.5 * (-om[r_p] * q[q_x] - om[r_q] * q[q_y] - om[r_r] * q[q_z]),
        0.5 * (om[r_p] * q[q_i] + om[r_r] * q[q_y] - om[r_q] * q[q_z]),
        0.5 * (om[r_q] * q[q_i] - om[r_r] * q[q_x] + om[r_p] * q[q_z]),
        0.5 * (om[r_r] * q[q_i] + om[r_q] * q[q_x] - om[r_p] * q[q_y])
    ]


def quat_wrap_shortest(q):
    if q[q_i] < 0:
        return [-q[q_i], -q[q_x], -q[q_y], -q[q_z]]
    else:
        return q


def quat_mormalize(q):
    nq = np.linalg.norm(q)
    return np.array(1. / nq * q)


def quat_vmul(q, v):
    qi2 = q[q_i] * q[q_i]
    qiqx = q[q_i] * q[q_x]
    qiqy = q[q_i] * q[q_y]
    qiqz = q[q_i] * q[q_z]
    qx2 = q[q_x] * q[q_x]
    qxqy = q[q_x] * q[q_y]
    qxqz = q[q_x] * q[q_z]
    qy2 = q[q_y] * q[q_y]
    qyqz = q[q_y] * q[q_z]
    qz2 = q[q_z] * q[q_z]
    m00 = qi2 + qx2 - qy2 - qz2
    m01 = 2 * (qxqy + qiqz)
    m02 = 2 * (qxqz - qiqy)
    m10 = 2 * (qxqy - qiqz)
    m11 = qi2 - qx2 + qy2 - qz2
    m12 = 2 * (qyqz + qiqx)
    m20 = 2 * (qxqz + qiqy)
    m21 = 2 * (qyqz - qiqx)
    m22 = qi2 - qx2 - qy2 + qz2

    return [
        m00 * v[v_x] + m01 * v[v_y] + m02 * v[v_z],
        m10 * v[v_x] + m11 * v[v_y] + m12 * v[v_z],
        m20 * v[v_x] + m21 * v[v_y] + m22 * v[v_z]]


def quat_inv_vmul(q, v):
    qi2 = q[q_i] * q[q_i]
    qiqx = q[q_i] * q[q_x]
    qiqy = q[q_i] * q[q_y]
    qiqz = q[q_i] * q[q_z]
    qx2 = q[q_x] * q[q_x]
    qxqy = q[q_x] * q[q_y]
    qxqz = q[q_x] * q[q_z]
    qy2 = q[q_y] * q[q_y]
    qyqz = q[q_y] * q[q_z]
    qz2 = q[q_z] * q[q_z]
    m00 = qi2 + qx2 - qy2 - qz2
    m01 = 2 * (qxqy + qiqz)
    m02 = 2 * (qxqz - qiqy)
    m10 = 2 * (qxqy - qiqz)
    m11 = qi2 - qx2 + qy2 - qz2
    m12 = 2 * (qyqz + qiqx)
    m20 = 2 * (qxqz + qiqy)
    m21 = 2 * (qyqz - qiqx)
    m22 = qi2 - qx2 - qy2 + qz2

    return [
        m00 * v[v_x] + m10 * v[v_y] + m20 * v[v_z],
        m01 * v[v_x] + m11 * v[v_y] + m21 * v[v_z],
        m02 * v[v_x] + m12 * v[v_y] + m22 * v[v_z]]


def quat_integrate(q_im1, om_im1, dt):
    no = np.linalg.norm(om_im1)
    if no == 0.:
        codt2 = 1.
        sodt2o = 1.
    else:
        codt2 = math.cos(dt / 2. * no)
        sodt2o = math.sin(dt / 2. * no) / no

    p = -om_im1[0]
    q = -om_im1[1]
    r = -om_im1[2]
    # WTF wrong formula. inverted signs instead
    Phi = np.array([[codt2, p * sodt2o, q * sodt2o, r * sodt2o],
                    [-p * sodt2o, codt2, -r * sodt2o, q * sodt2o],
                    [-q * sodt2o, r * sodt2o, codt2, -p * sodt2o],
                    [-r * sodt2o, -q * sodt2o, p * sodt2o, codt2]])
    q_i = np.dot(Phi, q_im1)
    return q_i


def cross_product(v1, v2):
    return np.array([v1[v_y] * v2[v_z] - v1[v_z] * v2[v_y],
                     v1[v_z] * v2[v_x] - v1[v_x] * v2[v_z],
                     v1[v_x] * v2[v_y] - v1[v_y] * v2[v_x]])


def dot_product(v1, v2):
    return np.sum(v1 * v2)
