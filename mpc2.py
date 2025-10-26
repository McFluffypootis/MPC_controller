# %%
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import sys


class mpc_controller:
    def __init__(self, model, Q, R, QT, ulim, ylim, MT, mt, horizon=10):

        self.model = model
        self.Q = Q
        self.R = R
        self.QT = QT
        self.horizon = horizon

        self.ulim = np.tile((-ulim, ulim), (self.horizon - 1, 1))
        self.ylim = np.tile(
            ((-ylim[0, 0], ylim[0, 0]), (-ylim[0, 1], ylim[0, 1])),
            (self.horizon - 1, 1),
        )
        self.bounds = np.vstack(
            (
                np.tile(self.ylim, (self.horizon - 1, 1)),
                np.tile(self.ulim, (self.horizon - 1, 1)),
            )
        )

        self.MT = MT
        self.mt = mt

    def control_action(self):
        u_horz = self.compute_optimal_horizon()
        return u_horz[:, 0]

    def cost_function(self, v):
       

        # TODO determine the order of the vaiables of the otimization vector
        # that is are they ordered as [x1_0 x2_0 u_0 x1_1 x2_1 u_1 ...] 
        # or [x1_0 x1_0 ... x2_0 x2_1 ... u1 u2 ...]

        #then it is simply to insert them as x  and u below (same for constraints etc)
        x = v[

        cost = 0
        for t in range(self.horizon):
            # reference error
            e = self.model.x_ref - x[:, t]

            # tracking cost
            cost += e.T @ self.Q @ e

            # control action cost
            cost += u.T[:, t] @ self.R @ u[:, t]

        # terimal cost
        cost += (
            self.model.x_ref
            - x[:, self.horizon] @ self.QT @ self.model.x_ref
            - x[:, self.horizon]
        )

        return cost

    def dynamic_constraints(self, x):
        """Returns a matrix (state_dim x horizon_length) with values showing how much
        we diverge from the dynamical system.
        Basically enforce x(t+1) = A @ x + B @ u
        """

        x_constraint = self.model.get_current_state().T
        _, B, _, _ = self.model.get_system()

        n, _ = B.shape

        constraint_violation = np.zeros(n * (self.horizon - 1))

        for t in range(0, self.horizon - 1, 2):
            # simulate a step
            x_constraint = self.model.pendulum_dynamics(x_constraint, x[t + n])
            # calculate and save how far off the optimizer guess is

            constraint_violation[t : t + n] = x_constraint.T[0] - x[t : t + n]

        return constraint_violation

    def input_constraints(self, x_optimizer, u_optimizer):
        """Returns a matrix (state_dim x horizon_length - 1) with values showing how much
        we diverge from the dynamical system. Note that length = horizon - 1 since inputs are between states
        Basically enforce u < ulim + s
        """

        _, B, _, _ = self.model.get_system()
        _, m = B.shape

        constraint_violation = np.zeros((m, self.horizon - 1))

        for t in range(self.horizon - 1):
            constraint_violation

    def compute_optimal_horizon(self):
        # in this method the optimization is done
        # x0 is updated every iteration in pendulum_sim

        constraints = (
            {"type": "eq", "fun": self.dynamic_constraints},
            # {"type": "ineq", "fun": self.inequality_constraints},
        )

        bounds = np.vstack((self.ylim, self.ulim))

        init = np.array(
            [
                self.model.get_current_state()[0],
                self.model.get_current_state()[1],
                0,
            ]
        )

        init = np.zeros(3 * (self.horizon - 1))

        res = sp.optimize.minimize(
            self.cost_function,
            x0=init,
            method="SLSQP",
            constraints=constraints,
            bounds=bounds,
        )

        if not res.success:
            raise Exception(f"Failed to converge to solution")
        return res.value


# %%
class Pendulum:
    def __init__(self, l, m, b, x_init, dt=0.01, theta_ref=0.0):

        # pendulum Paramters
        self.g = 9.81
        self.l = l
        self.m = m
        self.b = b

        self.x_current = x_init
        self.dt = dt
        self.theta_ref = theta_ref
        self.x_ref = np.array([[theta_ref, 0.0]]).T

        self.A = self.linearlize_pendulum(np.pi / 2)
        self.B = np.array([[0, 1 / (self.m * self.l**2)]]).T
        self.C = np.eye(2)
        self.D = np.zeros(1)

    def get_current_state(self):
        return self.x_current

    def get_system(self):
        return self.A, self.B, self.C, self.D

    def linearlize_pendulum(self, theta):

        A = np.array(
            [
                [0, 1, 0],
                [
                    -(self.g / self.l) * np.cos(theta),
                    self.b / (self.m * self.l**2),
                    0,
                ],
                [1, 0, 0],
            ]
        )

        return A

    def pendulum_dynamics(self, x, u):
        # give the dynamics of the system: dtheta, domega and dz
        theta, omega = x[[0]], x[[1]]

        dtheta = omega
        domega = (
            -(self.g / self.l) * np.sin(theta)
            - (self.b / (self.m * self.l**2)) * omega
            + u / (self.m * self.l**2)
        )

        return np.array([dtheta, domega])

    def pendulum_step(self, u):
        f = self.pendulum_dynamics(self.get_current_state(), u)
        # this method gives x_i+1 = x_i + dt*f
        x = self.get_current_state()
        theta, omega = x[0], x[1]
        dtheta, domega, dz = f[0], f[1], f[2]
        theta_next = theta + self.dt * dtheta
        omega_next = omega + self.dt * domega

        x_next = np.array([theta_next, omega_next]).T

        return x_next

    def simulate_pendulum_with_controller(self, x0, T, controller):
        # loop with the mpc algorithm
        time = np.arange(0, T, dt)
        timesteps = len(time)

        x_traj = np.zeros((timesteps, 3))  # is a column vector
        x_traj[0, :] = x0
        self.x_current = x0

        u_traj = np.zeros((timesteps, 1))

        for i in range(0, timesteps - 1):

            self.A = self.linearlize_pendulum(self.x_current[0])

            # get optimal control for first state
            u = controller.control_action()

            u_traj[i, :] = u  # use first optimal control value

            # get the dynamics using that value
            x_traj[i + 1, :] = self.pendulum_step(u)  # take a step to get next state

            self.x_current = x_traj[i + 1, :]

        return x_traj, u_traj


if __name__ == "__main__":
    # System parameters
    l = 1.0  # length
    m = 1.0  # mass
    b = 0.1  # damping

    # Initial state
    theta0 = np.pi / 2.0
    omega0 = 0.0
    z0 = theta0
    x0 = np.array([theta0, omega0, z0])

    # Simulator parameters
    T = 30
    dt = 0.1
    theta_ref = 0  # to easily test different reference values
    pendulum = Pendulum(l, m, b, x0, dt, theta_ref)

    # Define controller
    Q = np.diag([1, 1, 1])  # weight on theta, omega and z
    R = np.array([[0.1]])  # weight on control effort
    QT = np.diag([0.1, 0.1, 0.1])

    # state limits
    theta_lim = np.inf
    omega_lim = 10
    z_lim = np.inf
    xlim = np.array([[theta_lim, omega_lim]])

    # input limits
    ulim = 10

    # terminal limits
    MT = np.eye(3)
    mt = np.array([[10, 10, 10]]).T

    mpc = mpc_controller(pendulum, Q, R, QT, ulim, xlim, MT, mt, horizon=20)

    xs, u = pendulum.simulate_pendulum_with_controller(x0, T, mpc)
    time = np.arange(0, T, dt)

    # Plot results
    theta = xs[:, 0]
    omega = xs[:, 1]
    z = xs[:, 2]

    ## Plotting stuff
    plt.figure(figsize=(10, 10))

    plt.subplot(4, 1, 1)
    plt.plot(time, theta)
    plt.axhline(
        theta_ref,
        color="r",
        linestyle="--",
        label=f"Target θ = {np.round(theta_ref,1)}",
    )
    plt.ylabel("Theta (rad)")
    plt.legend()
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.plot(time, omega, color="orange")
    plt.ylabel("Omega (rad/s)")
    plt.legend()
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.plot(time, z, label="Integral of error", color="green")
    plt.ylabel("Integral of Error")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid()

    plt.subplot(4, 1, 4)
    plt.plot(time, u, color="k")
    plt.ylabel("Control action")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid()

    plt.suptitle("MPC for Pendulum Stabilization at θ = π/4")
    plt.tight_layout()
    plt.show()
