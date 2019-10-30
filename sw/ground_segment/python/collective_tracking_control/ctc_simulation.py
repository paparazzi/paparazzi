from scipy.integrate import odeint
import numpy as np

def odeintz(func, z0, t, **kwargs):
    """An odeint-like function for complex valued differential equations."""

    # Disallow Jacobian-related arguments.
    _unsupported_odeint_args = ['Dfun', 'col_deriv', 'ml', 'mu']
    bad_args = [arg for arg in kwargs if arg in _unsupported_odeint_args]
    if len(bad_args) > 0:
        raise ValueError("The odeint argument %r is not supported by "
                         "odeintz." % (bad_args[0],))

    # Make sure z0 is a numpy array of type np.complex128.
    z0 = np.array(z0, dtype=np.complex128, ndmin=1)

    def realfunc(x, t, *args):
        z = x.view(np.complex128)
        dzdt = func(z, t, *args)
        # func might return a python list, so convert its return
        # value to an array with type np.complex128, and then return
        # a np.float64 view of that array.
        return np.asarray(dzdt, dtype=np.complex128).view(np.float64)

    result = odeint(realfunc, z0.view(np.float64), t, **kwargs)

    if kwargs.get('full_output', False):
        z = result[0].view(np.complex128)
        infodict = result[1]
        return z, infodict
    else:
        z = result.view(np.complex128)
        return z

def xfunc(x, t, gammas, target_vel, omegas, vels):

    r_1, theta_1, r_2, theta_2, r_3, theta_3, r_ref, target_pos = x

    centroid_pos = (r_1+r_2+r_3)/3
    position_error = target_pos - r_ref
    distance_error = np.absolute(position_error)
    if distance_error < 0.1:
        distance_error = 0.1

    w_rho = (1 - np.exp(-0.01*distance_error)) / distance_error

    v_ref = target_vel + w_rho*(target_pos - centroid_pos)

    centroid_vel = (vels[0]*np.exp(theta_1*1j) + vels[1]*np.exp(theta_2*1j) + vels[2]*np.exp(theta_3*1j))/3

    u_1_vel = -gammas[0]*np.real(np.conj(centroid_vel - v_ref)*1j*vels[0]*np.exp(theta_1*1j))
    u_2_vel = -gammas[0]*np.real(np.conj(centroid_vel - v_ref)*1j*vels[1]*np.exp(theta_2*1j))
    u_3_vel = -gammas[0]*np.real(np.conj(centroid_vel - v_ref)*1j*vels[2]*np.exp(theta_3*1j))

    u_1_spacing = omegas[0]*(1 + gammas[1]*np.real(np.conj(r_1 - r_ref)*vels[0]*np.exp(theta_1*1j)))
    u_2_spacing = omegas[0]*(1 + gammas[1]*np.real(np.conj(r_2 - r_ref)*vels[1]*np.exp(theta_2*1j)))
    u_3_spacing = omegas[0]*(1 + gammas[1]*np.real(np.conj(r_3 - r_ref)*vels[2]*np.exp(theta_3*1j)))

    u_1 = u_1_vel + u_1_spacing
    u_2 = u_2_vel + u_2_spacing
    u_3 = u_3_vel + u_3_spacing

    dX1 = vels[0]*np.exp(theta_1*1j)
    dX2 = u_1

    dX3 = vels[1]*np.exp(theta_2*1j)
    dX4 = u_2

    dX5 = vels[2]*np.exp(theta_3*1j)
    dX6 = u_3

    dX7 = v_ref

    dX8 = target_vel

    return [dX1, dX2, dX3, dX4, dX5, dX6, dX7, dX8]


if __name__ == "__main__":

    gammas = np.array([0.001, 0.001])
    omegas = np.array([0.25, 0.25, 0.25])
    vels = np.array([16.1, 10.5, 12.5])
    target_vel = 0.0 + 0.0j
    X0 = np.array([100+21j, 0, 100+25j, 0.3, 120+40j, 0.1, 0+0j, 0+140j])
    t = np.linspace(0, 450, 1500)

    X, infodict = odeintz(xfunc, X0, t, args=(gammas,target_vel,omegas,vels), full_output=True)

    import matplotlib.pyplot as plt

    plt.clf()
    plt.plot(X[:,0].real, X[:,0].imag, label='agent 1')
    plt.plot(X[:,2].real, X[:,2].imag, label='agent 2')
    plt.plot(X[:,4].real, X[:,4].imag, label='agent 3')
    plt.plot(X[:,6].real, X[:,6].imag, label='ref')
    plt.plot(X[:,7].real, X[:,7].imag, label='target')
    plt.plot((X[:,0].real + X[:,2].real + X[:,4].real)/3, (X[:,0].imag + X[:,2].imag + X[:,4].imag)/3, label='centroid')
    plt.legend(loc='best')
    plt.axis("equal")
    plt.plot(X[-1,0].real, X[-1,0].imag, 'x')
    plt.plot(X[-1,2].real, X[-1,2].imag, 'x')
    plt.plot(X[-1,4].real, X[-1,4].imag, 'x')

    plt.show()
