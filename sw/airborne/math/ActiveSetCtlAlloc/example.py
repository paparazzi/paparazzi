# https://github.com/chcomin/ctypes-numpy-example/tree/master/simplest
import numpy as np
import ctypes as ct
funcs = ct.CDLL("./bin/libas.so")

AS_N_V = 6
AS_N_U = 20
AS_N_C = AS_N_U + AS_N_V

n_v = 4
n_u = 4
theta = ct.c_double(0.002)
cond_bound = ct.c_double(1e8)

B = np.random.random((AS_N_V, AS_N_U))
Wv = np.random.random(AS_N_V)
Wu = np.random.random(AS_N_U)
up = np.random.random(AS_N_U)
dv = np.random.random(AS_N_V)

A = np.zeros((AS_N_C, AS_N_U))
b = np.zeros(AS_N_C)
gamma = ct.c_double()

c_doublep = ct.POINTER(ct.c_double)
c_intp = ct.POINTER(ct.c_int)
c_int8p = ct.POINTER(ct.c_int8)

funcs.setupWLS_A.restype = None
funcs.setupWLS_A(
    B.ctypes.data_as(c_doublep),
    Wv.ctypes.data_as(c_doublep),
    Wu.ctypes.data_as(c_doublep),
    n_v, n_u,
    theta,
    cond_bound,
    A.ctypes.data_as(c_doublep),
    ct.byref(gamma),
    )

funcs.setupWLS_b.restype = None
funcs.setupWLS_b(
    dv.ctypes.data_as(c_doublep),
    up.ctypes.data_as(c_doublep),
    Wv.ctypes.data_as(c_doublep),
    Wu.ctypes.data_as(c_doublep),
    n_v, n_u,
    gamma,
    b.ctypes.data_as(c_doublep),
    )

umin = np.zeros(AS_N_U)
umax = np.ones(AS_N_U)
us = np.zeros(AS_N_U)
Ws = np.zeros(AS_N_U, dtype=np.int8)
imax = 10
iter = ct.c_int()
n_free = ct.c_int()
costs = np.zeros((15), )

activeSetAlgo = ct.CFUNCTYPE(
    ct.c_int,
    c_doublep,
    c_doublep,
    c_doublep,
    c_doublep,
    c_doublep,
    c_int8p,
    ct.c_int,
    ct.c_int,
    ct.c_int,
    c_intp,
    c_intp,
    c_doublep,
)

AS_QR_NAIVE = 0
AS_QR = 1
AS_CHOL = 2
AS_CG = 3

funcs.solveActiveSet.restype = ct.c_void_p # to get memory address correct
solveActiveSet = activeSetAlgo(funcs.solveActiveSet(AS_QR))
res = solveActiveSet(
    A.ctypes.data_as(c_doublep),
    b.ctypes.data_as(c_doublep),
    umin.ctypes.data_as(c_doublep),
    umax.ctypes.data_as(c_doublep),
    us.ctypes.data_as(c_doublep),
    Ws.ctypes.data_as(c_int8p),
    imax,
    n_u, n_v,
    ct.byref(iter), ct.byref(n_free),
    costs.ctypes.data_as(c_doublep),
)
# 
# print("hey")
# 
# print(f'Return: {res}')
# print(f'us    : {us[:n_u]}')