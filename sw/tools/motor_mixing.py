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

    def clear_rotors(self):
        self.rotors = []

    def add_rotor(self, x, y, d):
        """Add a rotor with x and y coordinates (body frame) and rotation direction"""
        self.rotors.append([x, y, d])

    def add_rotors(self, nb, offset=0.0):
        """ Add multiple rotors, placed symmetrically with optional angle offset.
        Adds rotors in clockwise order, starting with CW rotation direction (and then alternating).
        """
        direction = self.CW
        for i in range(0, nb):
            angle = i * 2 * np.pi / nb + offset
            # adding in CW direction (negative mathematical angle)
            self.add_rotor(np.cos(angle), np.sin(angle), direction)
            direction *= -1

    def calc_rotor_matrix(self, rotors):
        """ convert a list of rotors to input matrix for coefficient calculation

        arguments:
        rotors -- list of rotors, each rotor as [x,y,d]

        Rotor positions in body frame (x forward, y right) with rotation direction d (negative is CW, positive CCW)

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
        #print(B)
        # normalize inputs in xy plane to distance of 1.0 to center
        xy = input_matrix[0:2, :]
        xy_normalized = xy / np.linalg.norm(xy, axis=0)
        # maximum distance to either x or y axis (effective lever arm for that axis)
        max_lever = xy_normalized.max()
        # normalize roll/pitch to the largest lever arm of both
        rp_max = np.fabs(B[:, 0:2]).max() / max_lever
        # normalize yaw to 0.5
        y_max = 2 * np.fabs(B[:, 2]).max()
        n = np.array([rp_max, rp_max, y_max])
        # normalize and transpose
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

    def print_rotors(self, header=None):
        if header is not None:
            print(header)
        for r in self.rotors:
            print("x={: .3f}, y={: .3f}, d={}".format(r[0], r[1], "CW" if r[2] < 0 else "CCW"))


if __name__ == '__main__':
    """Print some example motor mixing configurations"""

    mm = MotorMixing()

    print("\nExample for quad in + configuration:")
    mm.clear_rotors()
    mm.add_rotors(4)
    mm.print_xml()

    print("\nExample for quad in x configuration:")
    mm.clear_rotors()
    # first rotor is front left
    mm.add_rotors(4, np.radians(-45))
    mm.print_xml()

    print("\nExample for hexa in + configuration:")
    mm.clear_rotors()
    mm.add_rotors(6)
    mm.print_xml()

    mm.clear_rotors()
    print("\nExample for hexa in x configuration:")
    mm.add_rotors(6, np.radians(-30))
    mm.print_xml()

    mm.clear_rotors()
    print("\nExample for octo in x configuration:")
    mm.add_rotors(8, np.radians(-22.5))
    mm.print_xml()

    print("\nExample for hexa in slight V configuration:")
    mm.clear_rotors()
    mm.add_rotor(-0.35, 0.17, mm.CW)
    mm.add_rotor(-0.35, -0.17, mm.CCW)
    mm.add_rotor(0, 0.25, mm.CCW)
    mm.add_rotor(0, -0.25, mm.CW)
    mm.add_rotor(0.35, 0.33, mm.CW)
    mm.add_rotor(0.35, -0.33, mm.CCW)
    mm.print_rotors("rotor positions in body frame:")
    mm.print_xml()
