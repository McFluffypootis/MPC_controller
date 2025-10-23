import pytest
from mpc import *

import numpy as np

# System parameters
l = 1.0  # length
m = 1.0  # mass
b = 0.1  # damping

# Define controller
Q = np.diag([0, 0, 0])  # weight on theta, omega and z
R = np.array([[0]])  # weight on control effort
QT = np.diag([0, 0, 0])


def test_different_refs():
    T = 30
    dt = 0.1

    theta0 = 0
    omega0 = 0.0
    z0 = 0.0
    x0 = np.array([theta0, omega0, z0])

    referense_test_cases = [np.pi * 1 / 4, np.pi * 1 / 2, np.pi * 3 / 4, np.pi]

    for ref in referense_test_cases:
        theta_ref = theta_ref4  # to easily test different reference values
        pendulum = Pendulum(l, m, b, Q, R, QT, dt, T, x0, theta_ref)
        xs, u = pendulum.pendulum_sim()
        time = pendulum.time

        assert xs != None
        assert u != None
