#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MPC + CBF Optimizer Node (Python)

Provides the service `/mpc_cbf_optimize` used by the Nav2 controller plugin.

- Solves a short-horizon MPC problem with Control Barrier Function (CBF) constraints
- Supports rotated elliptical obstacles
- Works with both obstacle array fields or Pose[] arrays in the service request
- Returns the optimal linear and angular velocity (v, w) for the first step
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# Adjust this import to match your package name
from nav2_mpc_cbf_controller.srv import MpcCbfOptimize

try:
    import casadi as ca
except Exception as e:
    raise RuntimeError("CasADi is not installed. Run `pip install casadi`.") from e


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle (rotation about z-axis)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class MpcCbfOptimizerNode(Node):
    """
    ROS2 node that provides the `/mpc_cbf_optimize` service.

    - Builds and solves an MPC problem with CBF constraints (rotated ellipses).
    - Can read obstacle data from array fields or from Pose[] messages.
    """

    def __init__(self) -> None:
        super().__init__('mpc_cbf_optimizer')

        # ===== Node parameters =====
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('horizon', 20)
        self.declare_parameter('gamma_cbf', 7.0)
        self.declare_parameter('w_u', 0.1)           # smoothness weight
        self.declare_parameter('alpha_w', 0.01)         # w weight
        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('allow_reverse', True)
        # Used only if obstacle data are provided as Pose[]
        self.declare_parameter('default_obs_a', 0.5)
        self.declare_parameter('default_obs_b', 0.5)

        # Read parameters
        self.dt = float(self.get_parameter('dt').value)
        self.N = int(self.get_parameter('horizon').value)
        self.gamma = float(self.get_parameter('gamma_cbf').value)
        self.wu = float(self.get_parameter('w_u').value)
        self.alpha_w = float(self.get_parameter('alpha_w').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.default_a = float(self.get_parameter('default_obs_a').value)
        self.default_b = float(self.get_parameter('default_obs_b').value)

        # Create the optimization service
        self.srv = self.create_service(
            MpcCbfOptimize,
            'mpc_cbf_optimize',
            self.optimize_callback
        )

        self.get_logger().info(
            f"[mpc_cbf_optimizer] Ready. dt={self.dt}, N={self.N}, "
            f"gamma={self.gamma}, v_max={self.v_max}, w_max={self.w_max}, "
            f"allow_reverse={self.allow_reverse}"
        )

    # ---------- Obstacle parsing helpers ----------

    def _extract_obstacles_from_arrays(self, request) -> List[Tuple[float, float, float, float, float]]:
        """
        Read obstacles from array fields:
          obs_x[], obs_y[], obs_a[], obs_b[], obs_theta[]
        Returns: list of (x, y, a, b, theta)
        """
        try:
            ox = list(request.obs_x)
            oy = list(request.obs_y)
            oa = list(request.obs_a)
            ob = list(request.obs_b)
            ot = list(request.obs_theta)
        except Exception:
            return []

        n = min(len(ox), len(oy), len(oa), len(ob), len(ot))
        obstacles = []
        for i in range(n):
            obstacles.append((float(ox[i]), float(oy[i]), float(oa[i]), float(ob[i]), float(ot[i])))
        return obstacles

    # def _extract_obstacles_from_poses(self, request) -> List[Tuple[float, float, float, float, float]]:
    #     """
    #     Read obstacles from a Pose[] field named 'obstacles'.
    #     Uses default a/b from node parameters.
    #     """
    #     obstacles = []
    #     try:
    #         poses = list(request.obstacles)
    #     except Exception:
    #         return obstacles

    #     for p in poses:
    #         x = float(p.position.x)
    #         y = float(p.position.y)
    #         qx = float(p.orientation.x)
    #         qy = float(p.orientation.y)
    #         qz = float(p.orientation.z)
    #         qw = float(p.orientation.w)
    #         theta = yaw_from_quaternion(qx, qy, qz, qw)
    #         obstacles.append((x, y, self.default_a, self.default_b, theta))
    #     return obstacles

    def _extract_obstacles_from_poses(self, request) -> List[Tuple[float, float, float, float, float]]:
        """
        Read obstacles from a Pose[] field named 'obstacles'.
        Uses default a/b from node parameters.
        """
        obstacles = []
        try:
            poses = list(request.obstacles)
        except Exception:
            return obstacles

        for p in poses:
            x = float(p.position.x)
            y = float(p.position.y)
            oa = float(p.orientation.x)
            ob = float(p.orientation.y)
            theta = float(p.orientation.z)
            obstacles.append((x, y, oa, ob, theta))
        return obstacles

    def _collect_obstacles(self, request) -> List[Tuple[float, float, float, float, float]]:
        """
        Try both array and Pose[] obstacle formats.
        Priority: array → Pose[].
        """
        arr = self._extract_obstacles_from_arrays(request)
        if arr:
            return arr
        return self._extract_obstacles_from_poses(request)

    # ---------- MPC + CBF optimization ----------

    def optimize_callback(self, request, response):
        """
        Solve the MPC optimization problem with CBF constraints.

        Input fields (required):
          - request.current_pose (geometry_msgs/Pose)
          - request.goal_pose (geometry_msgs/Pose)

        Optional fields:
          - request.obs_* arrays or request.obstacles[]
          - request.dt, request.horizon, etc. (if defined in .srv)
        """
        # Allow per-request overrides
        dt = getattr(request, 'dt', self.dt) or self.dt
        N = int(getattr(request, 'horizon', self.N) or self.N)
        gamma = getattr(request, 'gamma_cbf', self.gamma) or self.gamma
        v_max = getattr(request, 'v_max', self.v_max) or self.v_max
        w_max = getattr(request, 'w_max', self.w_max) or self.w_max

        # Current pose
        cur = request.current_pose
        x0 = float(cur.pose.position.x)
        y0 = float(cur.pose.position.y)
        qx, qy, qz, qw = cur.pose.orientation.x, cur.pose.orientation.y, cur.pose.orientation.z, cur.pose.orientation.w

        th0 = yaw_from_quaternion(qx, qy, qz, qw)

        # Goal pose
        goal = request.goal_pose
        xg = float(goal.pose.position.x)
        yg = float(goal.pose.position.y)

        # --- Reference path ---
        ref_path = getattr(request, 'reference_path', None)
        if ref_path and hasattr(ref_path, 'poses') and len(ref_path.poses) > 0:
            local_ref = [pose.pose for pose in ref_path.poses[:5]]
        else:
            local_ref = [goal.pose] 

        # Obstacles as (x, y, a, b, theta)
        obstacles = self._collect_obstacles(request)

        # ====== Build CasADi optimization problem ======
        v = ca.SX.sym('v', N)  # linear velocity sequence
        w = ca.SX.sym('w', N)  # angular velocity sequence

        # State rollout
        x = x0
        y = y0
        th = th0

        cost = 0.0
        g_list = []

        # Control smoothing weight
        wu = self.wu

        for k in range(N):
            # --- Tracking cost ---
            ref_idx = min(k, len(local_ref) - 1)
            x_ref = local_ref[ref_idx].position.x
            y_ref = local_ref[ref_idx].position.y
            cost += (x - x_ref)**2 + (y - y_ref)**2

            # --- Control smoothing ---
            if k > 0:
                cost += wu * ((v[k] - v[k - 1])**2 + (w[k] - w[k - 1])**2)

            cost += self.alpha_w * (w[k]**2)

            # CBF constraints for each obstacle (rotated ellipse)
            for (ox, oy, oa, ob, ot) in obstacles:
                c = ca.cos(ot)
                s = ca.sin(ot)
                dx = x - ox
                dy = y - oy
                dxr = c * dx + s * dy
                dyr = -s * dx + c * dy

                inva2 = 1.0 / (oa * oa + 1e-9)
                invb2 = 1.0 / (ob * ob + 1e-9)

                h = 1.0 - (dxr * dxr) * inva2 - (dyr * dyr) * invb2

                # Derivatives dh/dx, dh/dy
                dh_dxr = -2.0 * dxr * inva2
                dh_dyr = -2.0 * dyr * invb2

                ddxr_dx, ddxr_dy = c, s
                ddyr_dx, ddyr_dy = -s, c

                dh_dx = dh_dxr * ddxr_dx + dh_dyr * ddyr_dx
                dh_dy = dh_dxr * ddxr_dy + dh_dyr * ddyr_dy

                # State derivative
                xdot = v[k] * ca.cos(th)
                ydot = v[k] * ca.sin(th)

                # Time derivative of h
                h_dot = dh_dx * xdot + dh_dy * ydot

                # CBF constraint: ḣ + γ * h ≥ 0
                g_list.append(h_dot + gamma * h)

            # Integrate dynamics (unicycle model)
            x = x + dt * (v[k] * ca.cos(th))
            y = y + dt * (v[k] * ca.sin(th))
            th = th + dt * w[k]

        # Decision vector and constraints
        opt_vars = ca.vertcat(v, w)
        g = ca.vertcat(*g_list) if len(g_list) > 0 else ca.SX([])

        # Bounds
        if self.allow_reverse:
            v_lb = [-v_max] * N
        else:
            v_lb = [0.0] * N
        v_ub = [v_max] * N
        w_lb = [-w_max] * N
        w_ub = [w_max] * N

        lbx = v_lb + w_lb
        ubx = v_ub + w_ub

        # Build and configure solver
        nlp = {'x': opt_vars, 'f': cost, 'g': g}
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 200,
            'ipopt.acceptable_tol': 1e-4,
            'ipopt.tol': 1e-4,
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # Initial guess
        x0_guess = [0.0] * (2 * N)

        # Constraints bounds (ḣ + γh ≥ 0)
        if g.shape[0] > 0:
            lbg = [0.0] * int(g.shape[0])
            ubg = [1e19] * int(g.shape[0])
        else:
            lbg = []
            ubg = []

        # Solve
        try:
            sol = solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x0_guess)
            u_opt = sol['x'].full().flatten().tolist()
            v0 = float(u_opt[0]) if len(u_opt) > 0 else 0.0
            w0 = float(u_opt[N]) if len(u_opt) > N else 0.0

            # Fill response
            response.v = v0
            response.w = w0
            if hasattr(response, 'success'):
                response.success = True
            if hasattr(response, 'message'):
                response.message = "OK"
            return response

        except Exception as e:
            self.get_logger().warn(f"Solver failed: {e}")
            response.v = 0.0
            response.w = 0.0
            if hasattr(response, 'success'):
                response.success = False
            if hasattr(response, 'message'):
                response.message = f"solver_error: {e}"
            return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcCbfOptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
