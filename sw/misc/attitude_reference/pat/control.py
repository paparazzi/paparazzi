from __future__ import division

import math
import numpy as np


class LinRef:
    """ Linear Reference Model (with first order integration)"""

    def __init__(self, K):
        """K: coefficients of the caracteristic polynomial, in ascending powers order,
              highesr order ommited (normalized to -1)"""
        self.K = K
        self.order = len(K)
        self.X = np.zeros(self.order + 1)

    def run(self, dt, sp):
        self.X[:self.order] += self.X[1:self.order + 1] * dt
        e = np.array(self.X[:self.order])
        e[0] -= sp
        self.X[self.order] = np.sum(e * self.K)
        return self.X


class SatRef:
    """ Nested Saturations Reference Model (with first order integration)"""

    def __init__(self, K, sats):
        """
        K: coefficients of the caracteristic polynomial, in ascending power order,
           highest power ommited (normalized to -1)
        sats: saturations for each order in ascending order
        """
        self.K = K
        self.M = np.array(sats)
        self.M[0:-1] *= K[1:]
        for i in range(0, len(self.M) - 1):
            self.M[len(self.M) - 2 - i] /= np.prod(self.M[len(self.M) - 1 - i:])
        self.CM = np.cumprod(self.M[::-1])[::-1]
        print 'M', self.M, 'CM', self.CM
        self.order = len(K)
        self.X = np.zeros(self.order + 1)

    def run(self, dt, sp):
        self.X[:self.order] += self.X[1:self.order + 1] * dt
        e = np.array(self.X[:self.order])
        e[0] -= sp
        self.X[self.order] = 0
        for i in range(0, self.order):
            self.X[self.order] = self.M[i] * np.clip(self.K[i] / self.CM[i] * e[i] + self.X[self.order], -1., 1.)
        return self.X


class SatRefNaiveSecOrder:
    """ Second Order (brutally) Saturated Reference Model"""

    def __init__(self, K, sats):
        """
        K: coefficients of the caracteristic polynomial, in ascending power order,
           highest power ommited (normalized to -1)
        sats: saturations for each order in ascending order
        """
        self.K = K
        self.sats = sats
        self.order = len(K)
        self.X = np.zeros(self.order + 1)

    def run(self, dt, sp):
        self.X[:self.order] += self.X[1:self.order + 1] * dt
        e = np.array(self.X[:self.order])
        e[0] -= sp
        self.X[self.order] = np.sum(e * self.K)
        self.X[self.order] = np.clip(self.X[self.order], -self.sats[1], self.sats[1])  # saturate accel
        if self.X[self.order - 1] >= self.sats[0]:  # saturate vel, trim accel
            self.X[self.order - 1] = self.sats[0]
            if self.X[self.order] > 0:
                self.X[self.order] = 0
        elif self.X[self.order - 1] <= -self.sats[0]:
            self.X[self.order - 1] = -self.sats[0]
            if self.X[self.order] < 0:
                self.X[self.order] = 0
        return self.X


def butterworth(n, omega=1):
    """
    returns the coefficients of a butterworth polynomial of order n in ascending powers order
    see: http://en.wikipedia.org/wiki/Butterworth_filter
    """
    poles = [omega * np.exp(complex(0, 2 * k + n - 1) * math.pi / 2 / n) for k in range(1, n + 1)]
    coefs = -np.poly(poles)[::-1]
    return np.real(coefs[:-1])


def bessel(n, omega=1):
    """
    returns the coefficients of a bessel polynomial of order n in ascending powers order
    see http://en.wikipedia.org/wiki/Bessel_filter
    """
    coefs = [-math.factorial(2 * n - k) / (math.pow(2, n - k) * math.factorial(k) * math.factorial(n - k)) for k in
             range(0, n)]
    return coefs[::-1]


def poles(K):
    """ returns the roots of a polynomial whose coefficients are provided in ascending order,
        highest power coefficients normalized to -1
    """
    p = np.zeros(len(K) + 1)
    p[0] = -1
    p[1:] = K[::-1]
    return np.roots(p)
