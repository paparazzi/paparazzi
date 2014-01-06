#! /usr/bin/env python

from __future__ import division, print_function, absolute_import

import numpy as np
import matplotlib.pyplot as plt

import ref_quat_float
import ref_quat_int

steps = 512 * 2
ref_eul_res = np.zeros((steps, 3))
ref_quat_res = np.zeros((steps, 3))

ref_quat_float.init()
ref_quat_int.init()

# reset psi and update_ref_quat_from_eulers
ref_quat_float.enter()
ref_quat_int.enter()

q_sp = np.array([0.92387956, 0.38268346, 0., 0.])
ref_quat_float.sp_quat.array = q_sp
ref_quat_int.sp_quat.array = q_sp * (1 << 15)

for i in range(0, steps):
    ref_quat_float.update()
    ref_eul_res[i, :] = ref_quat_float.ref_euler.array
    ref_quat_int.update()
    ref_quat_res[i, :] = ref_quat_int.ref_euler.array / (1 << 20)


plt.figure(1)
plt.subplot(311)
plt.title("reference in euler angles")
plt.plot(np.degrees(ref_eul_res[:, 0]), 'g')
plt.plot(np.degrees(ref_quat_res[:, 0]), 'r')
plt.ylabel("phi [deg]")

plt.subplot(312)
plt.plot(np.degrees(ref_eul_res[:, 1]), 'g')
plt.plot(np.degrees(ref_quat_res[:, 1]), 'r')
plt.ylabel("theta [deg]")

plt.subplot(313)
plt.plot(np.degrees(ref_eul_res[:, 2]), 'g')
plt.plot(np.degrees(ref_quat_res[:, 2]), 'r')
plt.ylabel("psi [deg]")

plt.show()
