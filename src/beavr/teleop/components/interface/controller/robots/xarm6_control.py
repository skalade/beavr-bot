import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
from xarm import XArmAPI

from beavr.teleop.common.math.orientation import quat_positive, quat_to_axis_angle
from beavr.teleop.configs.constants import robots


class Robot(XArmAPI):
    """XArm6 robot wrapper with gripper support."""

    def __init__(self, ip="192.168.1.197", is_radian=True, simulation_mode=False):
        super(Robot, self).__init__(
            port=ip,
            is_radian=is_radian,
            is_tool_coord=False,
            enable_report=True,
            report_type="rich",
        )
        if simulation_mode and not self.is_simulation_robot:
            self.set_simulation_robot(on_off=True)
        self.ip = ip

        self._max_step_distance = 50.0
        self._last_command_time = 0
        self._command_interval = 1.0 / robots.VR_FREQ
        self._last_position = None

        # Pre-flight IK reject stats (rate-limited logging).
        self._ik_reject_count = 0
        self._ik_last_log_time = 0.0
        self._ik_log_interval_s = 2.0

    def _check_ik_reachable(self, pose_mm: np.ndarray) -> bool:
        """Pre-flight: ask the firmware to solve IK for ``pose_mm`` before sending it.

        ``pose_mm`` is the same 6-vector consumed by ``set_servo_cartesian_aa``:
        ``[x_mm, y_mm, z_mm, rx, ry, rz]`` where ``(rx, ry, rz)`` is an
        axis-angle (rotation vector) in radians. The xArm SDK's
        ``get_inverse_kinematics`` expects RPY, so we convert first.

        Returns ``True`` if the firmware reports a solution exists, ``False``
        otherwise. On any unexpected error we return ``True`` so we don't
        silently break teleop on an SDK API mismatch.
        """
        try:
            pos = pose_mm[:3]
            aa = pose_mm[3:]
            rpy = Rotation.from_rotvec(aa).as_euler("xyz", degrees=False)
            pose_rpy = np.concatenate([pos, rpy]).tolist()
            code, _joints = self.get_inverse_kinematics(pose_rpy, input_is_radian=True)
            return code == 0
        except Exception as e:
            print(f"IK pre-check raised, allowing pose through: {e}")
            return True

    def clear(self):
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=True)

    def set_mode_and_state(self, mode, state=0):
        self.set_mode(mode)
        time.sleep(0.2)
        self.set_state(state)
        time.sleep(0.3)
        return self.mode == mode and self.state == 2

    def reset(self):
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=True)

        print("Resetting xArm6 to home...")
        self.set_mode_and_state(0, 0)
        time.sleep(0.5)

        status = self.set_servo_angle(
            angle=robots.XARM6_HOME_JS,
            wait=True,
            is_radian=True,
            speed=math.radians(30),
        )
        if status != 0:
            print(f"Warning: Failed to home xArm6, status={status}")

        time.sleep(0.5)
        self.set_mode_and_state(1, 0)
        return status

    def get_arm_pose(self):
        status, home_pose = self.get_position_aa()
        home_affine = self.robot_pose_aa_to_affine(home_pose)
        return home_affine

    def get_arm_position(self):
        code, joint_state = self.get_servo_angle(is_radian=True, is_real=True)
        if code != 0:
            print(f"\033[93mWarning: Failed to get joint states, error code: {code}\033[0m")
            return None

        if isinstance(joint_state, (list, tuple, np.ndarray)) and len(joint_state) >= 6:
            return np.array(joint_state[:6], dtype=np.float32)
        else:
            print(f"\033[93mWarning: Unexpected joint state format: {joint_state}\033[0m")
            return None

    def get_arm_velocity(self):
        status, states = self.get_joint_states()
        if status != 0:
            print(f"\033[93mWarning: Failed to get joint states, error code: {status}\033[0m")
            return None
        velocities = np.array(states[1][:6], dtype=np.float32)
        return velocities

    def get_arm_torque(self):
        status, torques = self.get_joints_torque()
        if status != 0:
            print(f"\033[93mWarning: Failed to get joint torques, error code: {status}\033[0m")
            return None
        return np.array(torques[:6], dtype=np.float32)

    def get_arm_cartesian_coords(self):
        status, home_pose = self.get_position_aa()
        return home_pose

    def move_arm_joint(self, joint_angles):
        status = self.set_servo_angle(angle=joint_angles, wait=True, is_radian=True, mvacc=80, speed=10)
        if status != 0:
            print(f"\033[93mWarning: Failed to move robot, error code: {status}\033[0m")
        return status

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        try:
            current_time = time.time()

            if current_time - self._last_command_time < self._command_interval:
                return 0
            self._last_command_time = current_time

            if len(cartesian_pos) != 7:
                raise ValueError("Expected 7-D pose (x,y,z,qx,qy,qz,qw)")

            pos_m = np.asarray(cartesian_pos[0:3], dtype=np.float32)
            quat = np.asarray(cartesian_pos[3:7], dtype=np.float32)
            quat = quat_positive(quat)
            rotvec = quat_to_axis_angle(quat)
            aa = np.array([rotvec[0], -rotvec[2], rotvec[1]], dtype=np.float32)

            pose_mm = np.zeros(6, dtype=np.float32)
            pose_mm[0:3] = pos_m * robots.XARM_SCALE_FACTOR
            pose_mm[3:6] = aa

            if not self._check_ik_reachable(pose_mm):
                self._ik_reject_count += 1
                if current_time - self._ik_last_log_time >= self._ik_log_interval_s:
                    print(
                        f"IK pre-check rejected {self._ik_reject_count} pose(s) "
                        f"in last {self._ik_log_interval_s:.1f}s; latest skipped."
                    )
                    self._ik_last_log_time = current_time
                    self._ik_reject_count = 0
                return -1

            if self.mode != 1 or (self.state != 1 and self.state != 2):
                print(f"Robot not in correct mode/state. Current: Mode={self.mode}, State={self.state}")
                self.set_mode_and_state(1, 0)
                time.sleep(0.2)

            status = self.set_servo_cartesian_aa(
                pose_mm, wait=False, relative=False, mvacc=50, speed=10, is_radian=True
            )
            if status != 0:
                print(f"Servo cartesian command failed with status {status}")
            return status
        except Exception as e:
            print(f"Movement failed: {e}")
            return -1

    def home_arm(self):
        try:
            home_joints = np.array(robots.XARM6_HOME_JS, dtype=np.float32)
            self.set_mode_and_state(0, 0)
            status = self.set_servo_angle(angle=home_joints, wait=True, is_radian=True, mvacc=5, speed=1)
            return status
        except Exception as e:
            print(f"Error in home_arm: {e}")
            return -1

    def init_gripper(self):
        self.set_gripper_enable(True)
        self.set_gripper_mode(0)
        self.set_gripper_speed(5000)

    def set_gripper_pos(self, position):
        """Set gripper position. 0=closed, 850=open."""
        pos = max(0, min(850, int(position)))
        self.set_gripper_position(pos, wait=False)

    def get_gripper_pos(self):
        code, pos = self.get_gripper_position()
        if code != 0:
            return None
        return pos

    def robot_pose_aa_to_affine(self, pose_aa):
        rotation = Rotation.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3]) / robots.XARM_SCALE_FACTOR
        return np.block([[rotation, translation[:, np.newaxis]], [0, 0, 0, 1]])


class DexArmControl:
    """Controller for XArm6 with gripper."""

    def __init__(self, ip="192.168.1.197", simulation_mode=False):
        self.robot = Robot(ip, is_radian=True, simulation_mode=simulation_mode)
        self.robot.set_tcp_maxacc(50)
        self.robot.set_joint_maxacc(10)
        self.robot.set_tcp_jerk(100)
        self.robot.reset()
        self.robot.init_gripper()

        self._command_interval = 1.0 / robots.VR_FREQ
        self._last_command_time = 0

    def _init_xarm_control(self):
        return self.robot.reset()

    def get_arm_states(self):
        position = self.robot.get_arm_position()
        velocity = self.robot.get_arm_velocity()
        torque = self.robot.get_arm_torque()
        return {
            "joint_position": position,
            "joint_velocity": velocity,
            "joint_torque": torque,
            "timestamp": time.time(),
        }

    def get_arm_position(self):
        return self.robot.get_arm_position()

    def get_arm_velocity(self):
        return self.robot.get_arm_velocity()

    def get_arm_torque(self):
        return self.robot.get_arm_torque()

    def get_arm_cartesian_coords(self):
        return self.robot.get_arm_cartesian_coords()

    def get_cartesian_state(self):
        cartesian_state = self.robot.get_arm_cartesian_coords()
        return {
            "cartesian_position": np.array(cartesian_state, dtype=np.float32),
            "timestamp": time.time(),
        }

    def get_arm_pose(self):
        return self.robot.get_arm_pose()

    def move_arm_joint(self, joint_angles):
        return self.robot.move_arm_joint(joint_angles)

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        current_time = time.time()
        if current_time - self._last_command_time < self._command_interval:
            return 0
        self._last_command_time = current_time
        return self.robot.move_arm_cartesian(cartesian_pos, duration)

    def home_arm(self):
        return self.robot.home_arm()

    def set_gripper(self, position):
        self.robot.set_gripper_pos(position)

    def get_gripper(self):
        return self.robot.get_gripper_pos()
