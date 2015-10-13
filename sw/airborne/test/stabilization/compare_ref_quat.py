#! /usr/bin/env python

from __future__ import division, print_function, absolute_import

import numpy as np
import matplotlib.pyplot as plt

from ref_quat_float import RefQuatFloat
from ref_quat_int import RefQuatInt

steps = 512 * 2
ref_float_res = np.zeros((steps, 3))
ref_int_res = np.zeros((steps, 3))

ref_float = RefQuatFloat()
ref_int = RefQuatInt()

q_sp = np.array([0.92387956, 0.38268346, 0., 0.])
ref_float.setpoint = q_sp
ref_int.setpoint = q_sp
#print(ref_int.setpoint)

dt = 1/512

for i in range(0, steps):
    ref_float.update(dt)
    ref_float_res[i, :] = ref_float.eulers
    ref_int.update(dt)
    ref_int_res[i, :] = ref_int.eulers


plt.figure(1)
plt.subplot(311)
plt.title("reference in euler angles")
plt.plot(np.degrees(ref_float_res[:, 0]), 'g')
plt.plot(np.degrees(ref_int_res[:, 0]), 'r')
plt.ylabel("phi [deg]")

plt.subplot(312)
plt.plot(np.degrees(ref_float_res[:, 1]), 'g')
plt.plot(np.degrees(ref_int_res[:, 1]), 'r')
plt.ylabel("theta [deg]")

plt.subplot(313)
plt.plot(np.degrees(ref_float_res[:, 2]), 'g')
plt.plot(np.degrees(ref_int_res[:, 2]), 'r')
plt.ylabel("psi [deg]")

plt.show()
