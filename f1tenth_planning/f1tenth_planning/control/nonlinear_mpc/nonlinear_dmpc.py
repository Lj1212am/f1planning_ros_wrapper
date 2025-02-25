"""
NMPC waypoint tracker using CasADi. On init, takes in model equation. 
"""
from dataclasses import dataclass, field
import numpy as np
from f1tenth_planning.utils.utils import nearest_point, intersect_point
from f1tenth_gym.envs.track import Track
import casadi as ca

@dataclass
class mpc_config:
    NXK: int = 7  # length of kinematic state vector: z = [x, y, delta, v_x, yaw, yaw_rate, beta]
    NU: int = 2  # length of input vector: u = = [steering speed, acceleration]
    TK: int = 5  # finite time horizon length kinematic
    Rk: list = field(
        default_factory=lambda: np.diag([0.1, 0.1])
    )  # input cost matrix, penalty for inputs - [steering_speed, accel]
    Rd: list = field(
        default_factory=lambda: np.diag([0.1, 0.1])
    )  # input difference cost matrix, penalty for change of inputs - [steering_speed, accel]
    Qk: list = field(
        default_factory=lambda: np.diag([5.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta]
    Qf: list = field(
        default_factory=lambda: np.diag([5.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    )  # final state error matrix, penalty  for the final state constraints: [x, y, delta, v, yaw, yaw-rate, beta]
    N_IND_SEARCH: int = 20  # Search index number
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.03  # dist step [m] kinematic
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad]
    MIN_DSTEER: float = -3.2  # maximum steering speed [rad/s]
    MAX_DSTEER: float = 3.2  # maximum steering speed [rad/s]
    MAX_SPEED: float = 20.0  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 9.51  # maximum acceleration [m/ss]
    MIN_ACCEL: float = -9.51  # minimum acceleration [m/ss]

    # Vehicle parameters
    MU: float = 1.1 # friction coefficient
    C_SF: float = 5.3507 # front cornering stiffness
    C_SR: float = 5.3507 # rear cornering stiffness
    LF: float = 0.2735 # distance from center of gravity to front axle
    LR: float = 0.2585 # distance from center of gravity to rear axle
    H: float = 0.1825 # height of center of gravity
    M: float = 15.32 # mass of vehicle
    I: float = 0.64332 # moment of inertia

class NMPCPlanner:
    """
    NMPC Controller, uses CasADi to solve the nonlinear MPC problem using whatever model is passed in.

    All vehicle pose used by the planner should be in the map frame.

    Args:
        track (f1tenth_gym_ros:Track): track object, contains the reference raceline
        config (mpc_config, optional): MPC configuration object, contains MPC costs and constraints
    """

    def __init__(
        self,
        track: Track,
        config: mpc_config = mpc_config(),
        debug=False,
    ):
        # [x, y, delta, v_x, yaw, yaw_rate, beta]
        self.waypoints = [
            track.raceline.xs,
            track.raceline.ys,
            np.zeros_like(track.raceline.xs),
            track.raceline.vxs,
            track.raceline.yaws,
            np.zeros_like(track.raceline.xs),
            np.zeros_like(track.raceline.xs),
        ]
        self.config = config
        self.oa = None
        self.odelta_v = None
        self.ox = None
        self.oy = None
        self.x_sol = None
        self.u_sol = None
        self.ref_path = None
        self.ref_point = None
        self.debug = debug
        self.mpc_prob_init()
        self.max_reacquire = 20.0

        self.drawn_waypoints = []
        self.waypoint_render = None
        self.local_plan_render = None
        self.mpc_render = None

    def _get_current_waypoint(self, lookahead_distance, position):
        """
        Finds the current waypoint on the look ahead circle intersection

        Args:
            lookahead_distance (float): lookahead distance to find next point to track
            position (numpy.ndarray (2, )): current position of the vehicle (x, y)

        Returns:
            current_waypoint (numpy.ndarray (3, )): selected waypoint (x, y, velocity), None if no point is found
        """

        waypoints = np.array(self.waypoints).T
        nearest_p, nearest_dist, t, i = nearest_point(position, waypoints[:, 0:2])
        if nearest_dist < lookahead_distance:
            self.lookahead_point, self.current_index, t2 = intersect_point(
                position,
                lookahead_distance,
                waypoints[:, 0:2],
                np.float32(i + t),
                wrap=True,
            )
            if self.current_index is None:
                return None
            current_waypoint = waypoints[self.current_index, :]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        else:
            return None
        
    def render_waypoints(self, e):
        """
        update waypoints being drawn by EnvRenderer
        """
        points = np.array(self.waypoints).T[:, :2]
        if self.waypoint_render is None:
            self.waypoint_render = e.render_closed_lines(
                points, color=(128, 0, 0), size=1
            )
        else:
            self.waypoint_render.setData(points)

    def render_local_plan(self, e):
        """
        update waypoints being drawn by EnvRenderer
        """
        if self.ref_path is not None:
            points = self.ref_path[:2].T
            if self.local_plan_render is None:
                self.local_plan_render = e.render_closed_lines(
                    points, color=(0, 128, 0), size=2
                )
            else:
                self.local_plan_render.setData(points)

    def render_mpc_sol(self, e):
        """
        Callback to render the lookahead point.
        """
        if self.ox is not None and self.oy is not None:
            points = np.array([self.ox, self.oy]).T
            if self.mpc_render is None:
                self.mpc_render = e.render_lines(points, color=(0, 0, 128), size=2)
            else:
                self.mpc_render.setData(points)

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Heading
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state["pose_x"], state["pose_y"]]), np.array([cx, cy]).T)

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]

        ref_traj[3, 0] = sp[ind]
        ref_traj[4, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state["linear_vel_x"]) * self.config.DTK
        dind = travel / self.config.dlk
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[3, :] = sp[ind_list]
        cyaw[cyaw - state["pose_theta"] > 4.5] = np.abs(
            cyaw[cyaw - state["pose_theta"] > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state["pose_theta"] < -4.5] = np.abs(
            cyaw[cyaw - state["pose_theta"] < -4.5] + (2 * np.pi)
        )
        ref_traj[4, :] = cyaw[ind_list]

        return ref_traj
    
    def mpc_prob_init(self):
        self.opti = ca.Opti()

        # matrix containing all states over all time steps +1 (each column is a state vector)
        # print(self.config.NXK, self.config.TK + 1)
        self.X = self.opti.variable(self.config.NXK, self.config.TK + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        self.U = self.opti.variable(self.config.NU, self.config.TK)

        # coloumn vector for storing initial state and target state, and friction coefficient (mu)
        self.P = self.opti.parameter(self.config.NXK + 1, self.config.TK + 1)

        # state weights matrix converted from config Qk
        Q = ca.diagcat(*np.diag(self.config.Qk))

        # controls weights matrix
        R = ca.diagcat(*np.diag(self.config.Rk))

        # ---- dynamic constraints --------
        def f(state, u, p):
            # params
            mu = p[-1]
            # controls
            a = u[0]
            delta_v = u[1]
            # states
            x = state[0]
            y = state[1]
            delta = state[2]
            v = state[3]
            yaw = state[4]
            yaw_rate = state[5]
            beta = state[6]
            
            # set gravity constant
            g = 9.81  # [m/s^2]

            # mu = P[0, -1] # friction coefficient
            C_Sf = self.config.C_SF
            C_Sr = self.config.C_SR
            lf = self.config.LF
            lr = self.config.LR
            h = self.config.H
            m = self.config.M
            I = self.config.I

            # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
            # ---- dynamic constraints --------
            fsteer = lambda delta, vdelta: vdelta # ideal, continuous time steering-speed
            facc = lambda speed, along: along # ideal, continuous time acceleration
            
            v_s = 1.0
            v_b = 0.1
            v_min = v_s/2

            # weights for mixing both models
            w_std = 0.5 * (ca.tanh((v - v_s)/v_b) + 1)
            w_ks = 1 - w_std

            dyaw_slow = v * ca.cos(beta) * ca.tan(delta) / (lr + lf)
            d_beta_slow = (lr * delta_v) / ((lr + lf) * ca.cos(delta) ** 2 * (1 + (ca.tan(delta) ** 2 * lr / (lr + lf)) ** 2))
            dyaw_rate_slow = 1 / (lr + lf) * (a * ca.cos(beta) * ca.tan(delta) -
                                v * ca.sin(beta) * ca.tan(delta) * d_beta_slow  +
                                v * ca.cos(beta) * delta_v / (ca.cos(delta) ** 2))
            
            dyaw_fast = yaw_rate                # dyaw/dt = yaw_rate
            dyaw_rate_fast = -mu * m / (v * I * (lr + lf)) * (
                                            lf ** 2 * C_Sf * (g * lr - a * h) + lr ** 2 * C_Sr * (g * lf + a * h)) * yaw_rate \
                                + mu * m / (I * (lr + lf)) * (lr * C_Sr * (g * lf + a * h) - lf * C_Sf * (g * lr - a * h)) * beta \
                                + mu * m / (I * (lr + lf)) * lf * C_Sf * (g * lr - a * h) * delta # dyaw_rate/dt = RHS
            d_beta_fast = (mu / (v ** 2 * (lr + lf)) * (C_Sr * (g * lf + a * h) * lr - C_Sf * (g * lr - a * h) * lf) - 1) * yaw_rate \
                                - mu / (v * (lr + lf)) * (C_Sr * (g * lf + a * h) + C_Sf * (g * lr - a * h)) * beta \
                                + mu / (v * (lr + lf)) * (C_Sf * (g * lr - a * h)) * delta    # dbeta/dt = RHS
            
            # Mixed model using weights
            # For a switched model, check the commented code below
            RHS = ca.vertcat(
                                v * ca.cos(yaw + beta),  # dx/dt = v * cos(yaw + beta)
                                v * ca.sin(yaw + beta),  # dy/dt = v * sin(yaw + beta)
                                delta_v,                 # d(delta)/dt = delta_v
                                a,                       # dv/dt = a
                                w_std * dyaw_fast + w_ks * dyaw_slow, # dyaw/dt = f(x,u)
                                w_std * dyaw_rate_fast + w_ks * dyaw_rate_slow, # dyaw_rate/dt = f(x,u)
                                w_std * d_beta_fast + w_ks * d_beta_slow # dbeta/dt = f(x,u)
                            ) # dx/dt = f(x,u)

            return RHS

        cost_fn = 0  # cost function

        # initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.P[:-1, 0])

        # runge kutta
        for k in range(self.config.TK):
            st = self.X[:, k]
            con = self.U[:, k]
            p = self.P[:, k + 1]
            cost_fn = cost_fn + (st - p[:-1]).T @ Q @ (st - p[:-1]) + con.T @ R @ con

            st_next = self.X[:, k + 1]
            k1 = f(st, con, p)
            k2 = f(st + self.config.DTK / 2 * k1, con, p)
            k3 = f(st + self.config.DTK / 2 * k2, con, p)
            k4 = f(st + self.config.DTK * k3, con, p)
            st_next_RK4 = st + (self.config.DTK / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.opti.subject_to(st_next == st_next_RK4)

        self.opti.minimize(cost_fn)

        # control constraints
        self.opti.subject_to(self.U[0, :] > self.config.MIN_ACCEL)
        self.opti.subject_to(self.U[0, :] < self.config.MAX_ACCEL)
        self.opti.subject_to(self.U[1, :] > self.config.MIN_DSTEER)
        self.opti.subject_to(self.U[1, :] < self.config.MAX_DSTEER)

        # state constraints
        self.opti.subject_to(self.X[2, :] > self.config.MIN_STEER)
        self.opti.subject_to(self.X[2, :] < self.config.MAX_STEER)
        # solver
        jit_options = {"flags": ["-O3"], "verbose": True, "compiler":"ccache gcc", "temp_suffix":False}
        ipopt_opts = {
            "ipopt": {
                "print_level": 1,
                "max_iter": 4000,
                "acceptable_tol": 1e-4,
                "acceptable_obj_change_tol": 1e-3,
                "warm_start_init_point": "yes",
                "linear_solver": "mumps",
                # "hessian_approximation": "limited-memory",
            },
            "print_time": 0,
            "jit": True, 
            "compiler": "shell",
            "jit_options": jit_options,
            "jit_temp_suffix": False,
        }
        self.opti.solver("ipopt", ipopt_opts)
        return
    
    def mpc_prob_solve(self, reference_traj, x0, mu):
        # [x, y, delta, v_x, yaw, yaw_rate, beta]
        curr_state = ca.vertcat(
            x0["pose_x"],
            x0["pose_y"],
            x0["delta"],
            x0["linear_vel_x"],
            x0["pose_theta"],
            x0["ang_vel_z"],
            x0["beta"]
        )
        if self.x_sol is not None:
            self.opti.set_initial(
                self.X,
                ca.horzcat(
                    curr_state,
                    self.x_sol[:, 1:]
                )
            )
        else:
            self.opti.set_initial(self.X, ca.horzcat(curr_state, reference_traj[:, :-1]))

        self.opti.set_value(
            self.P,
            ca.horzcat(
                ca.vertcat(curr_state, 0.0),
                ca.vertcat(reference_traj[:, 1:], np.repeat(mu, self.config.TK ).reshape(1,-1))
            ),
        )

        # solve
        sol = self.opti.solve()

        # extract solution
        self.u_sol = sol.value(self.U)
        self.x_sol = sol.value(self.X)

        self.oa = self.u_sol[0, :].flatten()
        self.odelta_v = self.u_sol[1, :].flatten()
        self.odelta = self.x_sol[2, :].flatten()

        self.ox = self.x_sol[0, :].flatten()
        self.oy = self.x_sol[1, :].flatten()
        return self.oa[0], self.odelta_v[0]

    def plan(self, current_state, mu=None):
        """
        Plan a trajectory using the NMPC controller.

        Args:
            current_state (f1tenth_gym_ros.msg.State): current state of the vehicle

        Returns:
            (float, float): steering angle and acceleration
        """
        if mu is None:
            mu = self.config.MU

        if self.waypoints is None:
            raise ValueError(
                "Please set waypoints to track during planner instantiation or when calling plan()"
            )

        if current_state["linear_vel_x"] < 0.1:
            return self.config.MAX_ACCEL, 0.0

        # calculate the reference trajectory
        self.ref_path = self.calc_ref_trajectory(
            current_state, self.waypoints[0], self.waypoints[1], self.waypoints[2], self.waypoints[3]
        )

        # solve the NMPC problem
        oa, odelta_v = self.mpc_prob_solve(self.ref_path, current_state, mu)

        return oa, odelta_v
    