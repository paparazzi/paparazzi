#!/usr/bin/env python

from __future__ import print_function, division

import numpy as np
import string


class MotorMixing(object):
    """Calculate motor mixing coefficients from a rotor configuration"""

    # ClockWise and CounterClockWise rotation directions
    CW = -0.1
    CCW = 0.1

    def __init__(self):
        self.rotors = []

    def add_rotor(self, x, y, r):
        """Add a rotor with x and y coordinates (body frame) and rotation direction"""
        self.rotors.append([x, y, r])

    def calc_rotor_matrix(self, rotors):
        """ convert a list of rotors to input matrix for coefficient calculation

        arguments:
        rotors -- list of rotors, each rotor as [x,y,r]

        Rotor positions in body frame (x forward, y right) with rotation direction r (negative is CW, positive CCW)

        """
        m = np.asarray(rotors)
        # "convert" positions to moments around axis
        m[:, [0, 1, 2]] = m[:, [1, 0, 2]]
        m[:, 0] *= -1
        return m.T

    def calc_coeffs(self, input_matrix=None, scale=None):
        if input_matrix is None:
            input_matrix = self.calc_rotor_matrix(self.rotors)
        # Moore-Penrose pseudoinverse of input matrix (A)
        B = np.linalg.pinv(np.asarray(input_matrix))
        # normalize roll/pitch to the largest of both
        # normalize yaw to 0.5
        # and transpose
        rp_max = np.fabs(B[:, 0:2]).max()
        y_max = 2 * np.fabs(B[:, 2]).max()
        n = np.array([rp_max, rp_max, y_max])
        B_nt = (B / n).T
        if scale is None:
            return B_nt
        elif isinstance(scale, int):
            return np.around(scale * B_nt).astype(int)
        else:
            return np.around(scale * B_nt, 3)

    def print_xml(self, coeffs=None, scale=256):
        """calculate and print defines with mixing coefficients ready to paste to the airframe file"""
        if coeffs is None:
            coeffs = self.calc_coeffs(scale=scale)

        def fmt(x):
            if isinstance(x, int):
                return '{:>4d}'.format(x)
            else:
                return '{:>4f}'.format(x)

        print('<section name="MIXING" prefix="MOTOR_MIXING_">')
        print('  <define name="NB_MOTOR"    value="{}"/>'.format(coeffs.shape[1]))
        print('  <define name="SCALE"       value="{}"/>'.format(scale))
        rows = ['ROLL_COEF"   ', 'PITCH_COEF"  ', 'YAW_COEF"    ']
        for i, r in enumerate(rows):
            print('  <define name="' + r + 'value="{' + string.join([fmt(c) for c in coeffs[i]], ', ') + '}"/>')
        print('  <define name="THRUST_COEF" value="{' + string.join([fmt(scale)] * coeffs.shape[1], ', ') + '}"/>')
        print('</section>')


if __name__ == '__main__':
    """Example for hexa in X configuration"""
    mm = MotorMixing()
    mm.add_rotor(0.87, -0.5, mm.CW)
    mm.add_rotor(0.87, 0.5, mm.CCW)
    mm.add_rotor(0, 1, mm.CW)
    mm.add_rotor(-0.87, 0.5, mm.CCW)
    mm.add_rotor(-0.87, -0.5, mm.CW)
    mm.add_rotor(0, -1, mm.CCW)

    print("Example for hexa in X configuration:\n")
    mm.print_xml()
