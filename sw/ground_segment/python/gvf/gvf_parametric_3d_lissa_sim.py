# Simulator for the gvf_parametric algorithm in Paparazzi (fixed-wing).
# Here you can check the demanded climbing and heading rates for your aircraft
# and the expected trajectories as well.

import numpy as np
from scipy import linalg as la

import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D

# Happy pdf for a happy submission without complains in paperplaza, arxiv, etc
font = {'size'   : 20}

matplotlib.rc('font', **font)

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

# Simulation parameters
tf = 500
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 1

# Data log
X_h = np.zeros((time.size, 4))
theta_h = np.zeros((time.size, 1))
e1_h = np.zeros((time.size, 1))
e2_h = np.zeros((time.size, 1))
e3_h = np.zeros((time.size, 1))
u_theta_h = np.zeros((time.size, 1))
u_z_h = np.zeros((time.size, 1))
u_w_h = np.zeros((time.size, 1))

# Initial conditions
X = np.array([[100], [-53], [40], [0]])
X_dot = np.array([[0], [0], [0], [0]])
theta = np.pi/4

X_h[0, :] = X.transpose()
theta_h[0] = theta

# Desired trajectory
xo = 0
yo = 0
zo = 100
cx = 150
cy = 150
cz = -10
deltax = 0
deltay = np.pi/2
deltaz = 0
wx = 1
wy = 1
wz = 1

alpha = np.pi/4

# Controller
L = 1e-1
beta = 1e-2
k1 = 1e-3
k2 = 1e-3
k3 = 1e-3
ktheta = 0.5

# Vehicle
s = 15

for t in time[:-1]:

    x = X[0][0]
    y = X[1][0]
    z = X[2][0]
    w = X[3][0]

    wb = w*beta

    #f
    nrf1 = cx*np.cos(wx*wb + deltax)
    nrf2 = cy*np.cos(wy*wb + deltay)
    f3 = cz*np.cos(wz*wb + deltaz) + zo

    nrf1d = -wx*cx*np.sin(wx*wb + deltax)
    nrf2d = -wy*cy*np.sin(wy*wb + deltay)
    f3d = -wz*cz*np.sin(wz*wb + deltaz)

    nrf1dd = -wx*wx*cx*np.cos(wx*wb + deltax)
    nrf2dd = -wy*wy*cy*np.cos(wy*wb + deltay)
    f3dd = -wz*wz*cz*np.cos(wz*wb + deltaz)

    f1 = np.cos(alpha)*nrf1 - np.sin(alpha)*nrf2 + xo
    f2 = np.sin(alpha)*nrf1 + np.cos(alpha)*nrf2 + yo

    f1d = np.cos(alpha)*nrf1d - np.sin(alpha)*nrf2d
    f2d = np.sin(alpha)*nrf1d + np.cos(alpha)*nrf2d

    f1dd = np.cos(alpha)*nrf1dd - np.sin(alpha)*nrf2dd
    f2dd = np.sin(alpha)*nrf1dd + np.cos(alpha)*nrf2dd

    #phi
    phi1 = L*(x - f1)
    phi2 = L*(y - f2)
    phi3 = L*(z - f3)

    #Chi, J
    Chi = L*np.array([[-f1d*L*L*beta -k1*phi1],
                      [-f2d*L*L*beta -k2*phi2],
                      [-f3d*L*L*beta -k3*phi3],
                      [-L*L + beta*(k1*phi1*f1d + k2*phi2*f2d + k3*phi3*f3d)]])

    j44 = beta*beta*(k1*(phi1*f1dd-L*f1d*f1d) + k2*(phi2*f2dd-L*f2d*f2d) + k3*(phi3*f3dd-L*f3d*f3d))
    J = L*np.array([[-k1*L,        0,      0, -(beta*L)*(beta*L*f1dd-k1*f1d)],
                   [     0,    -k2*L,      0, -(beta*L)*(beta*L*f2dd-k2*f2d)],
                   [     0,      0,    -k3*L, -(beta*L)*(beta*L*f3dd-k3*f3d)],
                   [beta*L*k1*f1d, beta*L*k2*f2d, beta*L*k3*f3d,         j44]])

    #G, Fp, Gp
    G = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,0,0],
                  [0,0,0,0]])

    Fp = np.array([[0, -1, 0, 0],
                   [1,  0, 0, 0]])

    Gp = np.array([[0, -1, 0, 0],
                   [1,  0, 0, 0],
                   [0,  0, 0, 0],
                   [0,  0, 0, 0]])

    h = np.array([[np.cos(theta)],[np.sin(theta)]])
    ht = h.transpose()

    Chit = Chi.transpose()
    Chinorm = np.sqrt(Chi.transpose().dot(Chi))[0][0]
    Chih = Chi / Chinorm

    u_theta = (-(1/(Chit.dot(G).dot(Chi))*Chit.dot(Gp).dot(np.eye(4) - Chih.dot(Chih.transpose())).dot(J).dot(X_dot)) - ktheta*ht.dot(Fp).dot(Chi) / np.sqrt(Chit.dot(G).dot(Chi)))[0][0]

    u_zeta = Chi[2][0]*s / np.sqrt(Chi[0][0]*Chi[0][0] + Chi[1][0]*Chi[1][0])
    u_w    = Chi[3][0]*s / np.sqrt(Chi[0][0]*Chi[0][0] + Chi[1][0]*Chi[1][0])

    # Euler integration
    theta = theta + u_theta*dt
    X_dot = np.array([[s*np.cos(theta)],[s*np.sin(theta)], [u_zeta], [u_w]])
    X = X + X_dot*dt


    # Log
    X_h[it, :] = X.transpose()
    theta_h[it] = theta
    e1_h[it] = phi1
    e2_h[it] = phi2
    e3_h[it] = phi3
    u_theta_h[it] = u_theta
    u_z_h[it] = u_zeta
    u_w_h[it] = u_w

    it = it + 1

# Plots

fig3d = plt.figure()
ax3d = fig3d.gca(projection='3d')
ax3d.plot(X_h[:,0], X_h[:,1], X_h[:,2])

f, (axe1,axe2,axe3) = plt.subplots(3,1)
axe1.plot(time[1:], e1_h[1:])
axe2.plot(time[1:], e2_h[1:])
axe3.plot(time[1:], e3_h[1:])
axe3.set_xlabel("Time [s]")
axe1.set_xlabel("error X")
axe2.set_xlabel("error Y")
axe3.set_xlabel("error Z")

f, (axut,axuz,axuw) = plt.subplots(3,1)
axut.plot(time[1:], u_theta_h[1:])
axuz.plot(time[1:], u_z_h[1:])
axuw.plot(time[1:], u_w_h[1:])
axuw.set_xlabel("Time [s]")
axut.set_ylabel("Heading rate [rad/s]")
axuz.set_ylabel("Climbing rate [m/s]")
axuw.set_ylabel("Virual coord rate")

plt.show()

