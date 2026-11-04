import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import casadi as ca
import math
import os
import time

from poker_interfaces.msg import ServoVelocityCommand, MotorFeedback
from poker_interfaces.action import MoveJoints, MovePose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory


class PokerController(Node):
    def __init__(self):
        super().__init__('poker_controller')

        # --- Robot Configuration ---
        self.n_joints = 6
        self.SERVO_ZERO = [1848, 1906, 500, 1859, 2050, 2013]
        self.SERVO_SIGNS = [1] * self.n_joints
        self.STEPS_PER_RAD = 4096.0 / (2.0 * np.pi)

        # --- Matrix LQR Design ---
        self.Ts = 0.01
        q_diag = np.array([10.0] * self.n_joints)
        r_diag = np.array([50.0] * self.n_joints)

        # Solve scalar DARE per joint (vectorised)
        disc = np.sqrt(q_diag**2 + (4.0 * q_diag * r_diag) / (self.Ts**2))
        P_diag = 0.5 * (q_diag + disc)

        # LQR gain per joint
        K_diag = (self.Ts * P_diag) / (r_diag + (self.Ts**2) * P_diag)
        self.K_diag = K_diag

        # --- Load Kinematics ---
        self.ik_func = None
        try:
            pkg_dir = get_package_share_directory('poker_control')
            model_dir = os.path.join(pkg_dir, 'models')
            self.ik_func = ca.Function.load(os.path.join(
                model_dir, 'inverse_kinematics.casadi'))
        except Exception as e:
            self.get_logger().error(f"Failed to load Kinematics: {e}")

        # --- State Management ---
        self.q_current_internal = np.zeros(self.n_joints)
        self.q_measured = np.zeros(self.n_joints)
        self.has_feedback = False
        self.last_feedback_time = None

        self.q_ref_traj = []
        self.v_ref_traj = []
        self.traj_index = 0
        self.tracking_active = False
        self.goal_reached_logged = False

        self.current_q_target = None
        self.current_duration = 0.0

        # Tracking variables for Action Feedback
        self.last_error = 0.0
        self.last_elapsed = 0.0

        # --- Interfaces ---
        self.sub_feedback = self.create_subscription(
            MotorFeedback, '/motor_feedback', self.feedback_callback, 10)

        self.pub_cmd = self.create_publisher(
            ServoVelocityCommand, '/servo_velocity_cmd', 10)

        self.pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)

        self.pub_ik_solution = self.create_publisher(
            Float64MultiArray, '/ik_solution', 10)

        self.joint_names = ['1', '2', '3', '4', '5', '6']

        # --- ACTION SERVERS ---
        # Give actions their own thread pool so they don't block motor feedback!
        self.action_cb_group = MutuallyExclusiveCallbackGroup()

        self.joints_action_server = ActionServer(
            self, MoveJoints, '/move_joints',
            execute_callback=self.execute_joints_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group)

        self.pose_action_server = ActionServer(
            self, MovePose, '/move_pose',
            execute_callback=self.execute_pose_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group)

        # Control loop runs on a fixed 100 Hz timer so the period matches the
        # Ts the LQR gain was tuned for, regardless of feedback bus rate.
        self.control_timer = self.create_timer(self.Ts, self.control_cb)

        # Wheel mode holds the last speed forever, so a stalled feedback
        # stream while tracking is dangerous: the error term freezes and the
        # last nonzero command keeps running. Stop if feedback goes quiet.
        self.feedback_watchdog = self.create_timer(
            0.1, self.feedback_watchdog_cb)

    # --- ACTION HANDLERS ---
    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        if self.tracking_active:
            self.get_logger().warn("Rejecting goal: Robot is currently moving!")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow the user to cancel a movement mid-flight."""
        self.get_logger().info("Received request to cancel movement.")
        return CancelResponse.ACCEPT

    def execute_joints_callback(self, goal_handle):
        """Handles the MoveJoints action lifecycle."""
        req = goal_handle.request
        q_target = np.array(req.joints)

        self.get_logger().info("Executing MoveJoints Action...")
        self.plan_move(q_target, req.duration)

        feedback_msg = MoveJoints.Feedback()

        # Loop while the controller tracks the trajectory
        while self.tracking_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop()  # Stops the controller
                self.get_logger().info('MoveJoints Goal Canceled.')
                return MoveJoints.Result(success=False, final_error=self.last_error)

            # Send live feedback to the client
            feedback_msg.current_error = self.last_error
            feedback_msg.elapsed_time = self.last_elapsed
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.05)

        goal_handle.succeed()
        result = MoveJoints.Result(success=True, final_error=self.last_error)
        return result

    def execute_pose_callback(self, goal_handle):
        """Handles the MovePose action lifecycle, including IK."""
        req = goal_handle.request

        if self.ik_func is None:
            self.get_logger().error("IK Function not loaded. Aborting action.")
            goal_handle.abort()
            return MovePose.Result(success=False, final_error=99.9)

        try:
            target_vec = [req.x, req.y, req.z, req.pitch, req.roll]
            res = self.ik_func(target_vec)
            q_target = np.array(res).flatten()

            ik_msg = Float64MultiArray()
            ik_msg.data = q_target.tolist()
            self.pub_ik_solution.publish(ik_msg)

            self.get_logger().info("Executing MovePose Action (IK Solved)...")
            self.plan_move(q_target, req.duration)

            feedback_msg = MovePose.Feedback()

            while self.tracking_active:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop()
                    return MovePose.Result(success=False, final_error=self.last_error)

                feedback_msg.current_error = self.last_error
                feedback_msg.elapsed_time = self.last_elapsed
                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)

            goal_handle.succeed()
            return MovePose.Result(success=True, final_error=self.last_error)

        except Exception as e:
            self.get_logger().error(f"IK Planning Failed: {e}")
            goal_handle.abort()
            return MovePose.Result(success=False, final_error=99.9)

    # --- CORE CONTROLLER LOGIC ---
    def feedback_callback(self, msg):
        # Drop telemetry packets the driver flagged as untrustworthy. A bus
        # read failure on any servo poisons multiple fields; using such a
        # packet would feed garbage into the LQR error term. The timer-driven
        # loop just sees slightly stale q_measured until the next valid
        # packet, and SimBridge / mock_arm_server always set
        # feedback_valid=True.
        if not msg.feedback_valid:
            return
        if len(msg.positions) < self.n_joints:
            return
        for i in range(self.n_joints):
            steps = msg.positions[i]
            if steps == -1:
                continue
            diff = steps - self.SERVO_ZERO[i]
            self.q_measured[i] = diff / \
                (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
        self.has_feedback = True
        self.last_feedback_time = self.get_clock().now()

    def control_cb(self):
        if self.tracking_active and self.current_q_target is not None:
            self.update_control(self.current_q_target, self.current_duration)

    def feedback_watchdog_cb(self):
        if not self.tracking_active:
            return
        now = self.get_clock().now()
        if self.last_feedback_time is None:
            # Open-loop velocity commands can't verify progress; stop rather
            # than spin blind.
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed > 0.5:
                self.get_logger().error(
                    "No feedback since move start - stopping!")
                self.stop()
            return
        stale = (now - self.last_feedback_time).nanoseconds / 1e9
        if stale > 0.5:
            self.get_logger().error(
                f"Feedback stale ({stale:.2f}s) - stopping!")
            self.stop()

    def plan_move(self, q_target, duration):
        self.current_q_target = q_target
        self.current_duration = duration
        self.start_time = self.get_clock().now()

        q_start = self.q_measured if self.has_feedback else self.q_current_internal
        self.generate_trajectory(q_start, q_target, duration)

        self.traj_index = 0
        self.tracking_active = True
        self.goal_reached_logged = False

    def generate_trajectory(self, q_start, q_end, duration):
        steps = int(duration / self.Ts)
        if steps <= 0:
            steps = 1
        t_vec = np.linspace(0, duration, steps)

        self.q_ref_traj = [np.zeros(self.n_joints) for _ in range(steps)]
        self.v_ref_traj = [np.zeros(self.n_joints) for _ in range(steps)]

        for j in range(self.n_joints):
            s, e = q_start[j], q_end[j]
            a0 = s
            a3 = 10 * (e - s) / (duration**3)
            a4 = -15 * (e - s) / (duration**4)
            a5 = 6 * (e - s) / (duration**5)

            q_j = a0 + a3*(t_vec**3) + a4*(t_vec**4) + a5*(t_vec**5)
            v_j = 3*a3*(t_vec**2) + 4*a4*(t_vec**3) + 5*a5*(t_vec**4)

            for k in range(steps):
                self.q_ref_traj[k][j] = q_j[k]
                self.v_ref_traj[k][j] = v_j[k]

    def publish_velocity(self, u_rad_s):
        """Convert a joint velocity vector (rad/s) to signed steps/s and send it."""
        msg = ServoVelocityCommand()
        msg.ids = [1, 2, 3, 4, 5, 6]
        msg.speeds = []
        for j in range(self.n_joints):
            u_steps = u_rad_s[j] * self.STEPS_PER_RAD * self.SERVO_SIGNS[j]
            # No software speed cap: the servo firmware saturates the command
            # at its own internal limit, and the LQR gain keeps u sane anyway.
            msg.speeds.append(int(round(u_steps)))
        self.pub_cmd.publish(msg)

    def stop(self):
        """Halt tracking and command zero velocity on every joint."""
        self.tracking_active = False
        self.publish_velocity(np.zeros(self.n_joints))

    def update_control(self, q_target, duration):
        if not self.tracking_active:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        self.traj_index = int(elapsed / self.Ts)

        if self.traj_index >= len(self.q_ref_traj):
            self.traj_index = len(self.q_ref_traj) - 1

        q_des = self.q_ref_traj[self.traj_index]
        v_des = self.v_ref_traj[self.traj_index]

        if self.traj_index + 1 < len(self.q_ref_traj):
            q_des_next = self.q_ref_traj[self.traj_index + 1]
        else:
            q_des_next = q_des

        max_pos_error = np.max(np.abs(q_target - self.q_measured))

        # Save state for the Action Server Feedback
        self.last_error = float(max_pos_error)
        self.last_elapsed = float(elapsed)

        error_threshold = 0.09  # ~5 degrees in radians

        if self.traj_index == len(self.q_ref_traj) - 1:
            if not self.goal_reached_logged:
                if (max_pos_error <= error_threshold):
                    self.get_logger().info(
                        f"✅ GOAL REACHED (Actual time: {elapsed:.2f}s, Final error: {max_pos_error:.3f} rad)")
                    self.goal_reached_logged = True
                    self.stop()
                    return
                elif elapsed >= (duration + 10):
                    self.get_logger().warn(
                        f"Goal timeout! Final error: {max_pos_error:.3f} rad")
                    self.goal_reached_logged = True
                    self.stop()
                    return

        if self.has_feedback:
            e = q_des - self.q_measured
        else:
            e = np.zeros(6)

        # Velocity command: feedforward trajectory speed + LQR feedback term.
        u_tilde = v_des + self.K_diag * e
        self.publish_velocity(u_tilde)

        current_rads = self.q_measured if self.has_feedback else self.q_current_internal
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = current_rads.tolist()
        self.pub_joint_states.publish(js_msg)

        if self.traj_index < len(self.q_ref_traj):
            self.q_current_internal = q_des


def main(args=None):
    rclpy.init(args=args)
    # Brief pause to allow DDS to deregister any stale endpoints from a previous
    # session before this node's action servers are advertised. Without this,
    # relaunching quickly can leave ghost servers visible to clients, causing
    # the "unexpected goal response" warning in the action client.
    time.sleep(1.0)
    node = PokerController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
