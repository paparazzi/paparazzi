#
# Copyright (C) 2014  Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
"""
This is a reference implementation of the control system to serve as specifications
  for the C version
"""

import math, logging, pdb
import numpy as np
from scipy import integrate

import pat.utils as pu
import pat.algebra as pa
import pat.control as pc

from c_att_refs.ref_quat_float import RefQuatFloat
from c_att_refs.ref_quat_int import RefQuatInt

LOG = logging.getLogger('control')
# LOG.setLevel(logging.ERROR)
LOG.setLevel(logging.DEBUG)


class AttitudeReference(object):
    """Sort of abstract base class"""
    def __init__(self, **kwargs):
        self.omega = kwargs.get('omega', 6.)
        self.xi = kwargs.get('xi', 0.8)
        self.t = kwargs.get('t0', 0.)
        for key, value in kwargs.iteritems():
            if hasattr(self, key):
                setattr(self, key, value)
        for p in ['omega', 'xi']:
            self.ensure_vect(p)

    def ensure_vect(self, p):
        if type(getattr(self, p)) == float:
            setattr(self, p, getattr(self, p) * np.ones(3))

    def set_param(self, p, v):
        if hasattr(self, p):
            #print("%s: setting param %s to %s" % (self.name, p, v))
            if type(v) == float:
                v = v * np.ones(3)
            setattr(self, p, v)

    def run(self, t, sp_euler=None, sp_quat=None):
        dt = t - self.t
        self.t = t
        if sp_euler is not None:
            self.update_euler(sp_euler, dt)
        elif sp_quat is not None:
            self.update_quat(sp_quat, dt)

    def update_quat(self, setpoint, dt):
        raise NotImplementedError("Should have implemented this")

    def set_euler(self, euler):
        raise NotImplementedError("Should have implemented this")


class att_ref_default(AttitudeReference):
    """Default second order attitude reference model, Python implementation"""
    name = 'Python default'

    def __init__(self, **kwargs):
        super(att_ref_default, self).__init__(**kwargs)
        self.euler, self.quat = np.zeros(3), pa.quat_zero()
        self.vel, self.accel = np.zeros(3), np.zeros(3)

    def set_quat(self, quat, vel=np.zeros(3), accel=np.zeros(3)):
        self.quat = quat
        self.vel = vel
        self.accel = accel
        self.euler = pa.euler_of_quat(self.quat)

    def set_euler(self, euler, vel=np.zeros(3), accel=np.zeros(3)):
        self.euler = euler
        self.vel = vel
        self.accel = accel
        self.quat = pa.quat_of_euler(self.euler)

    def update_quat(self, setpoint, dt):
        self.quat = pa.quat_integrate(self.quat, self.vel, dt)
        self.vel += dt * self.accel
        err_quat = pa.quat_wrap_shortest(pa.quat_inv_comp(setpoint, self.quat))
        self.accel = -2. * self.xi * self.omega * self.vel \
                     - 2. * self.omega * self.omega * err_quat[1:]
        return self.quat, self.vel, self.accel

    def update_euler(self, setpoint, dt):
        self.update_quat(pa.quat_of_euler(setpoint), dt)
        self.euler = pa.euler_of_quat(self.quat)


class att_ref_sat_naive(att_ref_default):
    """Attitude reference model with naive saturation"""
    name = 'Python Sat Naive'

    def __init__(self, **kwargs):
        super(att_ref_sat_naive, self).__init__(**kwargs)
        self.sat_vel = kwargs.get('sat_vel', pu.rad_of_deg(100))
        self.sat_accel = kwargs.get('sat_accel', pu.rad_of_deg(500))
        for p in ['sat_vel', 'sat_accel']:
            self.ensure_vect(p)

    def saturate_naive(self):
        self.accel = np.clip(self.accel, -self.sat_accel, self.sat_accel)
        for i in range(0, 3):
            if self.vel[i] >= self.sat_vel[i]:
                self.vel[i] = self.sat_vel[i]
                if self.accel[i] > 0:
                    self.accel[i] = 0
            elif self.vel[i] <= -self.sat_vel[i]:
                self.vel[i] = -self.sat_vel[i]
                if self.accel[i] < 0:
                    self.accel[i] = 0

    def update_quat(self, setpoint, dt):
        super(att_ref_sat_naive, self).update_quat(setpoint, dt)
        self.saturate_naive()
        return self.quat, self.vel, self.accel


class att_ref_sat_nested(att_ref_sat_naive):
    """Nested saturation"""
    name = 'Python Sat Nested'

    def __init__(self, **kwargs):
        super(att_ref_sat_nested, self).__init__(**kwargs)
        self._compute_auxiliary()

    def update_quat(self, sp, dt):
        err_quat = 2. * np.array(pa.quat_wrap_shortest(pa.quat_inv_comp(sp, self.quat)))
        self.e = [err_quat[1:], self.vel, self.accel]
        self.quat = pa.quat_integrate(self.quat, self.vel, dt)
        self.vel += dt * self.accel
        self.accel = np.zeros(3)
        self.accel = self.M[0] * np.clip(self.K[0] / self.CM[0] * self.e[0] + self.accel, -1., 1.)
        self.accel = self.M[1] * np.clip(self.K[1] / self.CM[1] * self.e[1] + self.accel, -1., 1.)
        # pdb.set_trace()
        return self.quat, self.vel, self.accel

    def set_param(self, p, v):
        super(att_ref_sat_nested, self).set_param(p, v)
        self._compute_auxiliary()

    def _compute_auxiliary(self):
        # self.K = pc.butterworth(2, self.omega[0])
        omega, xi = self.omega[0], self.xi[0]
        omega_d = omega * math.sqrt(1 - xi ** 2)
        LOG.debug('omega_d: {}'.format(omega_d))
        LOG.debug('-omega*xi: {}'.format(-omega * xi))
        coefs = -np.poly([complex(-omega * xi, omega_d), complex(-omega * xi, -omega_d)])[::-1]
        self.K = np.real(coefs[:-1])
        LOG.debug('sat_nested.__init__  omega: {} coefs:{} K:{} poles:{} '.format(self.omega[0], coefs, self.K, pc.poles(self.K)))
        self.sats = np.array([self.sat_vel, self.sat_accel])
        LOG.debug(' sats:\n{}'.format(self.sats))
        self.M = np.array(self.sats)
        self.M[0] *= self.K[1]
        self.M[0] /= np.prod(self.M[1:], axis=0)
        LOG.debug(' M:\n{}'.format(self.M))
        self.CM = np.cumprod(self.M[::-1], axis=0)[::-1]
        LOG.debug(' CM:\n{}\n'.format(self.CM))


def my_K(omega, xi=0.8):
    omega_d = omega * math.sqrt(1 - xi ** 2)
    p1 = complex(-omega * xi, omega_d)
    p2 = complex(-omega * xi, -omega_d)
    p3 = -1.5 * omega
    poles = [p1, p2, p3]
    coefs = -np.poly(poles)[::-1]
    return np.real(coefs[:-1])


class att_ref_sat_nested2(att_ref_sat_naive):
    """Nested saturation"""
    name = 'Python Sat Nested2'

    def __init__(self, **kwargs):
        super(att_ref_sat_nested2, self).__init__(**kwargs)
        self.sat_jerk = kwargs.get('sat_jerk', pu.rad_of_deg(3000))
        self.ensure_vect('sat_jerk')
        self.jerk = np.zeros(3)
        self._compute_auxiliary()

    def update_quat(self, sp, dt):
        err_quat = 2. * np.array(pa.quat_wrap_shortest(pa.quat_inv_comp(sp, self.quat)))
        self.e = [err_quat[1:], self.vel, self.accel]

        self.quat = pa.quat_integrate(self.quat, self.vel, dt)
        self.vel += dt * self.accel
        self.accel += dt * self.jerk
        self.jerk = np.zeros(3)
        self.jerk = self.M[0] * np.clip(self.K[0] / self.CM[0] * self.e[0] + self.jerk, -1., 1.)
        self.jerk = self.M[1] * np.clip(self.K[1] / self.CM[1] * self.e[1] + self.jerk, -1., 1.)
        self.jerk = self.M[2] * np.clip(self.K[2] / self.CM[2] * self.e[2] + self.jerk, -1., 1.)
        # pdb.set_trace()
        return self.quat, self.vel, self.accel

    def set_param(self, p, v):
        super(att_ref_sat_nested2, self).set_param(p, v)
        self._compute_auxiliary()

    def _compute_auxiliary(self):
        # self.K = pc.butterworth(3, self.omega[0])
        self.K = my_K(self.omega[0], self.xi[0])
        LOG.debug('sat_nested.__init__  omega: {} K:{} poles:{} '.format(self.omega[0], self.K, pc.poles(self.K)))
        self.sats = np.array([self.sat_vel, self.sat_accel, self.sat_jerk])
        LOG.debug(' sats:\n{}'.format(self.sats))
        self.M = np.array(self.sats)
        # pdb.set_trace()
        for i in range(0, 3):
            self.M[0:-1, i] *= self.K[1:]
            for j in range(0, 2):
                self.M[1 - j, i] /= np.prod(self.M[2 - j:, i])
        LOG.debug(' M:\n{}'.format(self.M))
        self.CM = np.cumprod(self.M[::-1], axis=0)[::-1]
        LOG.debug(' CM:\n{}\n'.format(self.CM))


class AttRefNative(AttitudeReference):
    """ Base class for native C implementations """
    def __init__(self, **kwargs):
        super(AttRefNative, self).__init__(**kwargs)

    def update_quat(self, setpoint, dt):
        self.setpoint = setpoint
        self.update(dt)

    def set_euler(self, euler):
        self.euler = euler

    @property
    def vel(self):
        """ alias for rate """
        return self.rate

    @property
    def xi(self):
        """ alias for zeta """
        return self.zeta

    @xi.setter
    def xi(self, value):
        self.zeta = value


class AttRefFloatNative(AttRefNative, RefQuatFloat):
    """ C implementation stabilization_attitude_ref_quat_float """
    name = 'Native Quat Float'

    def __init__(self, **kwargs):
        super(AttRefFloatNative, self).__init__(**kwargs)


class AttRefIntNative(AttRefNative, RefQuatInt):
    """ C implementation stabilization_attitude_ref_quat_int """
    name = 'Native Quat Int'

    def __init__(self, **kwargs):
        super(AttRefIntNative, self).__init__(**kwargs)


class att_ref_analytic_disc(att_ref_default):
    """ Scalar discrete time second order LTI"""

    def __init__(self, axis=0):
        super(att_ref_analytic_disc, self).__init__()
        self.name = 'Scalar Analytic discrete time'
        self.axis = axis

    def update_euler(self, setpoint, dt):
        omega, xi = self.omega[self.axis], self.xi[self.axis]
        omega_d = omega * math.sqrt(1 - xi ** 2)
        tan_phi = xi / math.sqrt(1 - xi ** 2)
        omd_dt = omega_d * dt
        s_omd_dt = math.sin(omd_dt)
        c_omd_dt = math.cos(omd_dt)
        a1 = tan_phi * s_omd_dt + c_omd_dt
        a2 = 1. / omega_d * s_omd_dt
        a3 = -omega / math.sqrt(1 - xi ** 2) * s_omd_dt
        a4 = -tan_phi * s_omd_dt + c_omd_dt
        emxiomdt = math.exp(-xi * omega * dt)
        A = emxiomdt * np.array([[a1, a2], [a3, a4]])
        B = np.array([[-emxiomdt * (tan_phi * s_omd_dt + c_omd_dt) + 1],
                      [omega / math.sqrt(1 - xi ** 2) * emxiomdt * s_omd_dt]])
        Xi = np.array([[self.euler[self.axis]], [self.vel[self.axis]]])
        Xi1 = np.dot(A, Xi) + np.dot(B, setpoint[self.axis])
        self.euler[self.axis], self.vel[self.axis] = Xi1


class att_ref_analytic_cont(att_ref_default):
    """ Scalar continuous time second order LTI"""

    def __init__(self, axis=0):
        super(att_ref_analytic_cont, self).__init__(**kwargs)
        self.name = 'Scalar Analytic continuous time'
        self.axis = axis

    def update_euler(self, setpoint, dt):
        omega, xi = self.omega[self.axis], self.xi[self.axis]

        def dyn_cont(X, t, U):
            Ac = np.array([[0, 1], [-omega ** 2, -2. * xi * omega]])
            Bc = np.array([[0], [omega ** 2]])
            return np.dot(Ac, X) + np.dot(Bc, U)

        Xi = np.array([self.euler[self.axis], self.vel[self.axis]])
        foo, Xi1 = integrate.odeint(dyn_cont, Xi, [0., dt], args=([setpoint[self.axis]],))
        self.euler[self.axis], self.vel[self.axis] = Xi1
        Xi1d = dyn_cont(Xi1, 0., [setpoint[self.axis]])
        self.accel[self.axis] = Xi1d[1]
