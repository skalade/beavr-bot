import logging
import time

import numpy as np

from beavr.teleop.common.network.handshake import HandshakeCoordinator
from beavr.teleop.common.network.publisher import ZMQPublisherManager
from beavr.teleop.common.network.subscriber import ZMQSubscriber
from beavr.teleop.common.network.utils import cleanup_zmq_resources
from beavr.teleop.common.ops import Ops
from beavr.teleop.components.detector.detector_types import SessionCommand
from beavr.teleop.components.interface.controller.robots.xarm6_control import (
    DexArmControl,
)
from beavr.teleop.components.interface.interface_base import RobotWrapper
from beavr.teleop.components.interface.interface_types import (
    CartesianState,
    CommandedCartesianState,
)
from beavr.teleop.components.operator import GripperCommand
from beavr.teleop.components.operator.operator_types import CartesianTarget
from beavr.teleop.configs.constants import robots

logger = logging.getLogger(__name__)


class XArm6Robot(RobotWrapper):
    """XArm6 teleop interface with gripper support."""

    def __init__(
        self,
        host,
        endeff_subscribe_port,
        joint_subscribe_port,
        home_subscribe_port,
        reset_subscribe_port,
        teleoperation_state_port,
        robot_ip,
        gripper_subscribe_port,
        is_right_arm=True,
        endeff_publish_port: int = 10040,
        state_publish_port: int = 10046,
        simulation_mode: bool = False,
        **kwargs,
    ):
        if not endeff_publish_port:
            raise ValueError("XArm6Robot requires an 'endeff_publish_port'")
        if not state_publish_port:
            raise ValueError("XArm6Robot requires a 'state_publish_port'")

        if simulation_mode:
            logger.info("XArm6Robot: simulation mode enabled - skipping hardware initialization")
            self._controller = None
        else:
            self._controller = DexArmControl(ip=robot_ip)
        self._is_right_arm = is_right_arm
        self._data_frequency = robots.VR_FREQ

        self._cartesian_coords_subscriber = ZMQSubscriber(
            host=host,
            port=endeff_subscribe_port,
            topic="endeff_coords",
            message_type=CartesianTarget,
        )

        self._reset_subscriber = ZMQSubscriber(
            host=host,
            port=reset_subscribe_port,
            topic="reset",
            message_type=SessionCommand,
        )

        self._home_subscriber = ZMQSubscriber(
            host=host,
            port=home_subscribe_port,
            topic="home",
            message_type=SessionCommand,
        )

        self._arm_teleop_state_subscriber = Ops(
            arm_teleop_state_subscriber=ZMQSubscriber(
                host=host,
                port=teleoperation_state_port,
                topic="pause",
                message_type=SessionCommand,
            )
        )

        # Gripper command subscriber
        self._gripper_subscriber = ZMQSubscriber(
            host=host,
            port=gripper_subscribe_port,
            topic="gripper",
            message_type=GripperCommand,
        )

        self._subscribers = {
            "cartesian_coords": self._cartesian_coords_subscriber,
            "reset": self._reset_subscriber,
            "home": self._home_subscriber,
            "teleop_state": self._arm_teleop_state_subscriber.get_arm_teleop_state,
            "gripper": self._gripper_subscriber,
        }

        self._publisher_manager = ZMQPublisherManager.get_instance()
        self._publisher_host = host
        self._endeff_publish_port = endeff_publish_port
        self._state_publish_port = state_publish_port

        self._latest_cartesian_coords = None
        self._latest_joint_state = None
        self._latest_cartesian_state_timestamp = 0
        self._latest_joint_state_timestamp = 0
        self._is_recording_enabled = False
        self._latest_commanded_cartesian_position = None
        self._latest_commanded_cartesian_timestamp = 0.0
        self._latest_gripper_position = 850  # Start open (SDK units 0-850)
        self._latest_gripper_width_m = 0.085  # Continuous metres mirror

        self._handshake_coordinator = HandshakeCoordinator.get_instance()
        self._handshake_server_id = f"{self.name}_handshake"
        self._handshake_coordinator.start_server(
            subscriber_id=self._handshake_server_id,
            bind_host="*",
            port=robots.TELEOP_HANDSHAKE_PORT + (1 if self._is_right_arm else 2),
        )
        logger.info(f"Handshake server started for {self.name}")

        self._is_homed = False

    @property
    def name(self):
        return f"{robots.ROBOT_NAME_XARM6}_{'right' if self._is_right_arm else 'left'}"

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state,
            "operator_cartesian_states": self.get_cartesian_state_from_operator,
            "xarm_cartesian_states": self.get_robot_actual_cartesian_position,
            "commanded_cartesian_state": self.get_cartesian_commanded_position,
            "joint_angles_rad": self.get_joint_position,
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    def get_joint_state(self):
        if self._controller is None:
            return None
        arm_states = self._controller.get_arm_states()
        if arm_states is None or arm_states.get("joint_position") is None:
            return None
        return {
            "joint_position": list(np.array(arm_states["joint_position"], dtype=np.float32)),
            "timestamp": arm_states.get("timestamp", time.time()),
        }

    def get_joint_velocity(self):
        if self._controller is None:
            return None
        return self._controller.get_arm_velocity()

    def get_joint_torque(self):
        if self._controller is None:
            return None
        return self._controller.get_arm_torque()

    def get_cartesian_state(self):
        if self._controller is None:
            return None
        return self._controller.get_cartesian_state()

    def get_joint_position(self):
        if self._controller is None:
            return None
        arm_position = self._controller.get_arm_position()
        if arm_position is None:
            return None
        return list(np.array(arm_position, dtype=np.float32))

    def get_cartesian_position(self):
        if self._controller is None:
            return None
        return self._controller.get_arm_cartesian_coords()

    def reset(self):
        if self._controller is None:
            return None
        return self._controller._init_xarm_control()

    def get_teleop_state(self):
        return self._arm_teleop_state_subscriber.get_arm_teleop_state()

    def get_pose(self):
        if self._controller is None:
            return None
        return self._controller.get_arm_pose()

    def home(self):
        if self._controller is None:
            return None
        return self._controller.home_arm()

    def move(self, input_angles):
        if self._controller is None:
            return
        self._controller.move_arm_joint(input_angles)

    def move_coords(self, cartesian_coords, duration=3):
        if self._controller is None:
            return
        self._controller.move_arm_cartesian(cartesian_coords, duration=duration)

    def get_cartesian_state_from_operator(self):
        if self._latest_cartesian_coords is None:
            return None
        position = tuple(np.asarray(self._latest_cartesian_coords, dtype=np.float32).tolist())
        return CartesianState(position_m=position, timestamp_s=self._latest_cartesian_state_timestamp)

    def get_cartesian_commanded_position(self):
        if self._latest_commanded_cartesian_position is None:
            return None
        return CommandedCartesianState(
            commanded_cartesian_position=self._latest_commanded_cartesian_position.tolist()
            if isinstance(self._latest_commanded_cartesian_position, np.ndarray)
            else list(self._latest_commanded_cartesian_position),
            timestamp_s=self._latest_commanded_cartesian_timestamp,
        )

    def get_robot_actual_cartesian_position(self):
        cartesian_state = self.get_cartesian_position()
        if cartesian_state is None:
            return None
        position = tuple(np.asarray(cartesian_state, dtype=np.float32).tolist())
        return CartesianState(position_m=position, timestamp_s=time.time())

    def send_robot_pose(self):
        if self._controller is None:
            # In sim mode publish an identity pose so the operator's reset handshake completes
            h_matrix = tuple(tuple(float(x) for x in row) for row in np.eye(4))
            self._publisher_manager.publish(
                host=self._publisher_host,
                port=self._endeff_publish_port,
                topic="endeff_homo",
                data=CartesianState(
                    timestamp_s=time.time(),
                    h_matrix=h_matrix,
                ),
            )
            return
        pose_homo = self._controller.get_arm_pose()
        try:
            h_matrix = tuple(tuple(float(x) for x in row) for row in pose_homo)
            self._publisher_manager.publish(
                host=self._publisher_host,
                port=self._endeff_publish_port,
                topic="endeff_homo",
                data=CartesianState(
                    timestamp_s=time.time(),
                    h_matrix=h_matrix,
                ),
            )
        except Exception as e:
            logger.error(f"Failed to publish robot pose for {self.name}: {e}")

    def check_reset(self):
        reset_bool = self._reset_subscriber.recv_keypoints()
        return reset_bool is not None

    def check_home(self):
        home_bool = self._home_subscriber.recv_keypoints()
        if home_bool == robots.ARM_TELEOP_STOP:
            return True
        elif home_bool == robots.ARM_TELEOP_CONT:
            return False
        return False

    def _process_gripper(self):
        msg = self._gripper_subscriber.recv_keypoints()
        if msg is not None:
            width_m = float(np.clip(msg.width_m, 0.0, 0.085))
            self._latest_gripper_width_m = width_m
            # Convert normalized width (0.0-0.085m) to SDK units (0-850)
            gripper_pos = int(np.clip(width_m / 0.085 * 850, 0, 850))
            self._latest_gripper_position = gripper_pos
            if self._controller is not None:
                self._controller.set_gripper(gripper_pos)

    def stream(self):
        self.home()
        if self._controller is not None:
            assert self._controller.robot.set_mode_and_state(1, 0), "Failed to enter SERVO-READY"

        target_interval = 1.0 / self._data_frequency
        next_frame_time = time.time()
        _sim_log_interval = 5.0  # seconds between sim-mode status logs
        _sim_last_log = time.time()
        _sim_cmd_count = 0

        while True:
            current_time = time.time()

            if current_time >= next_frame_time:
                next_frame_time = current_time + target_interval

                if self.check_home() and not self._is_homed:
                    self.home()
                    self._is_homed = True
                    self.send_robot_pose()
                elif not self.check_home() and self._is_homed:
                    self._is_homed = False

                if self.check_reset():
                    self.send_robot_pose()

                if self.get_teleop_state() == robots.ARM_TELEOP_STOP:
                    continue

                msg = self._cartesian_coords_subscriber.recv_keypoints()
                if msg is not None:
                    self._latest_commanded_cartesian_position = np.concatenate(
                        [
                            np.asarray(msg.position_m, dtype=np.float32),
                            np.asarray(msg.orientation_xyzw, dtype=np.float32),
                        ]
                    )
                    self._latest_commanded_cartesian_timestamp = msg.timestamp_s
                    _sim_cmd_count += 1

                if self._latest_commanded_cartesian_position is not None:
                    self.move_coords(self._latest_commanded_cartesian_position)

                # Process gripper commands
                self._process_gripper()

                self.publish_current_state()

                # Sim-mode: periodic status log so the user can see data is flowing
                if self._controller is None and current_time - _sim_last_log >= _sim_log_interval:
                    pos = self._latest_commanded_cartesian_position
                    if pos is not None:
                        logger.info(
                            f"[sim] {self.name} received {_sim_cmd_count} cmds in last {_sim_log_interval:.0f}s | "
                            f"pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                            f"quat=({pos[3]:.3f}, {pos[4]:.3f}, {pos[5]:.3f}, {pos[6]:.3f})"
                        )
                    else:
                        logger.info(f"[sim] {self.name}: waiting for operator commands...")
                    _sim_last_log = current_time
                    _sim_cmd_count = 0

                sleep_time = max(0, next_frame_time - time.time())
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def publish_current_state(self):
        publish_time = time.time()
        joint_states = self.get_joint_state()
        operator_cart = self.get_cartesian_state_from_operator()
        robot_cart = self.get_robot_actual_cartesian_position()
        commanded_cart = self.get_cartesian_commanded_position()
        joint_angles_rad = self.get_joint_position()

        current_state_dict = {}
        if joint_states is not None:
            current_state_dict["joint_states"] = joint_states
        if operator_cart is not None:
            current_state_dict["operator_cartesian_states"] = operator_cart.to_dict()
        if robot_cart is not None:
            current_state_dict["xarm_cartesian_states"] = robot_cart.to_dict()
        if commanded_cart is not None:
            current_state_dict["commanded_cartesian_state"] = commanded_cart.to_dict()
        if joint_angles_rad is not None:
            current_state_dict["joint_angles_rad"] = joint_angles_rad

        current_state_dict["gripper_position"] = self._latest_gripper_position
        current_state_dict["gripper_width_m"] = self._latest_gripper_width_m
        current_state_dict["timestamp"] = publish_time

        self._publisher_manager.publish(
            host=self._publisher_host,
            port=self._state_publish_port,
            topic=self.name,
            data=current_state_dict,
        )

    def __del__(self):
        if hasattr(self, "_handshake_coordinator") and hasattr(self, "_handshake_server_id"):
            self._handshake_coordinator.stop_server(self._handshake_server_id)
        cleanup_zmq_resources()
