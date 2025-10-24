# %%
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
import sys


class mpc_controller:
    def __init__(self, model, Q, R, QT, ulim, ylim, MT, mt, horizon=10):

        self.model = model
        self.Q = Q
        self.R = R
        self.QT = QT
        self.ulim = ulim
        self.ylim = ylim
        self.MT = MT
        self.mt = mt
        self.horizon = horizon

    def control_action(self):
        x_horz, u_horz = self.compute_optimal_horizon()
        return u_horz[:, 0]

    def compute_optimal_horizon(self):
        # in this method the optimization is done
        # x0 is updated every iteration in pendulum_sim

        x0 = self.model.get_current_state().T
        A, B, C, _ = self.model.get_system()

        # setup free varialbes
        n, m = B.shape
        x = cp.Variable((n, self.horizon + 1))
        u = cp.Variable((m, self.horizon))

        cost = 0
        constr = [x[:, [0]] == x0]

        for t in range(self.horizon):
            cost += cp.quad_form(x[:, t], self.Q) + cp.quad_form(u[:, t], self.R)
            # constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]
            # constr += [-self.ulim <= u[:, [t]], u[:, [t]] <= self.ulim]
            # constr += [
            #     -self.ylim <= C @ x[:, [t]],
            #     C @ x[:, [t]] <= self.ylim,
            # ]

        # constr += [self.MT @ x[:, [self.horizon]] <= self.mt]
        # cost += cp.quad_form(x[:, self.horizon], self.QT)
        problem = cp.Problem(cp.Minimize(cost), constr)

        if problem.solve(verbose=True) == False or u.value == None:
            raise Exception(
                f"Failed to converge to solution with values {x.value}, {u.value}"
            )

        return x.value, u.value


# %%
class Pendulum:
    def __init__(self, l, m, b, x_init, dt=0.01, theta_ref=0):

        # Pendelum Paramters
        self.g = 9.81
        self.l = l
        self.m = m
        self.b = b

        self.x_current = x_init
        self.dt = dt
        self.theta_ref = theta_ref

        self.A = self.linearlize_pendulum(np.pi / 2, 0)
        self.B = np.array([[0, 1 / (self.m * self.l**2), 0]]).T
        self.C = np.eye(3)
        self.D = np.zeros(1)

    def get_current_state(self):
        return self.x_current

    def get_system(self):
        return self.A, self.B, self.C, self.D

    def linearlize_pendulum(self, theta, omega):

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
        dz = theta - self.theta_ref  # error integration

        return np.array([dtheta, domega, dz])

    def pendulum_step(self, u):
        f = self.pendulum_dynamics(self.get_current_state(), u)
        # this method gives x_i+1 = x_i + dt*f
        x = self.get_current_state()
        theta, omega, z = x[0], x[1], x[2]
        dtheta, domega, dz = f[0], f[1], f[2]
        theta_next = theta + self.dt * dtheta
        omega_next = omega + self.dt * domega
        z_next = z + self.dt * dz

        x_next = np.array([theta_next, omega_next, z_next]).T

        return x_next

    def simulate_pendelum_with_controller(self, x0, T, controller):
        # loop with the mpc algorithm
        time = np.arange(0, T, dt)
        timesteps = len(time)

        x_traj = np.zeros((timesteps, 3))  # is a column vector
        x_traj[0, :] = x0
        self.x_current = x0

        u_traj = np.zeros((timesteps, 1))

        for i in range(0, timesteps - 1):

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
    theta0 = np.pi
    omega0 = 0.0
    z0 = 0.0
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
    theta_lim = 2 * np.pi
    omega_lim = 5
    z_lim = np.pi
    xlim = np.array([[theta_lim, omega_lim, z_lim]]).T

    # input limits
    ulim = np.array([[20]])

    # terminal limits
    MT = np.eye(3)
    mt = np.diag([[0.1, 0.1, 0.1]]).T

    mpc = mpc_controller(pendulum, Q, R, QT, ulim, xlim, MT, mt, horizon=20)

    xs, u = pendulum.simulate_pendelum_with_controller(x0, T, mpc)
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
