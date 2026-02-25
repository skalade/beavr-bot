"""Config for unimanual (right-hand) XArm6 robot with gripper."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

from beavr.teleop.common.configs.loader import Laterality, log_laterality_configuration
from beavr.teleop.components.interface.robots.xarm6_robot import XArm6Robot
from beavr.teleop.configs.constants import network, ports, robots
from beavr.teleop.configs.robots import TeleopRobotConfig
from beavr.teleop.configs.robots.shared_components import SharedComponentRegistry

logger = logging.getLogger(__name__)


@dataclass
class XArm6RobotCfg:
    host: str = network.HOST_ADDRESS
    robot_ip: str = network.RIGHT_XARM_IP
    is_right_arm: bool = True
    endeff_publish_port: int = ports.XARM6_ENDEFF_PUBLISH_PORT
    endeff_subscribe_port: int = ports.XARM6_ENDEFF_SUBSCRIBE_PORT
    joint_subscribe_port: int = ports.XARM6_JOINT_SUBSCRIBE_PORT
    reset_subscribe_port: int = ports.XARM6_RESET_SUBSCRIBE_PORT
    state_publish_port: int = ports.XARM6_STATE_PUBLISH_PORT
    home_subscribe_port: int = ports.XARM6_HOME_SUBSCRIBE_PORT
    teleoperation_state_port: int = ports.XARM6_TELEOPERATION_STATE_PORT
    gripper_subscribe_port: int = ports.XARM6_GRIPPER_PORT
    hand_side: str = robots.RIGHT
    recorder_config: dict[str, Any] = field(
        default_factory=lambda: {
            "robot_identifier": robots.ROBOT_IDENTIFIER_RIGHT_XARM6,
            "recorded_data": [
                robots.RECORDED_DATA_JOINT_STATES,
                robots.RECORDED_DATA_XARM_CARTESIAN_STATES,
                robots.RECORDED_DATA_COMMANDED_CARTESIAN_STATE,
                robots.RECORDED_DATA_JOINT_ANGLES_RAD,
            ],
        }
    )

    def build(self):
        return XArm6Robot(
            host=self.host,
            robot_ip=self.robot_ip,
            is_right_arm=self.is_right_arm,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            joint_subscribe_port=self.joint_subscribe_port,
            reset_subscribe_port=self.reset_subscribe_port,
            state_publish_port=self.state_publish_port,
            home_subscribe_port=self.home_subscribe_port,
            teleoperation_state_port=self.teleoperation_state_port,
            gripper_subscribe_port=self.gripper_subscribe_port,
        )


@dataclass
class XArm6OperatorCfg:
    host: str = network.HOST_ADDRESS
    transformed_keypoints_port: int = ports.KEYPOINT_TRANSFORM_PORT
    stream_configs: dict[str, Any] = field(
        default_factory=lambda: {
            "host": network.HOST_ADDRESS,
            "port": ports.CONTROL_STREAM_PORT,
        }
    )
    stream_oculus: bool = True
    endeff_publish_port: int = ports.XARM6_ENDEFF_SUBSCRIBE_PORT
    endeff_subscribe_port: int = ports.XARM6_ENDEFF_PUBLISH_PORT
    moving_average_limit: int = 3
    arm_resolution_port: int = ports.KEYPOINT_STREAM_PORT
    use_filter: bool = False
    teleoperation_state_port: int = ports.XARM6_TELEOPERATION_STATE_PORT
    gripper_publish_port: int = ports.XARM6_GRIPPER_PORT
    logging_config: dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False,
            "log_dir": "logs",
            "log_poses": True,
            "log_prefix": "xarm6",
        }
    )
    hand_side: str = robots.RIGHT

    def build(self):
        from beavr.teleop.components.operator.robots.xarm6_right import (
            XArm6RightOperator,
        )

        return XArm6RightOperator(
            host=self.host,
            transformed_keypoints_port=self.transformed_keypoints_port,
            stream_configs=self.stream_configs,
            stream_oculus=self.stream_oculus,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            moving_average_limit=self.moving_average_limit,
            gripper_publish_port=self.gripper_publish_port,
            arm_resolution_port=self.arm_resolution_port,
            use_filter=self.use_filter,
            teleoperation_state_port=self.teleoperation_state_port,
            logging_config=self.logging_config,
        )


@dataclass
@TeleopRobotConfig.register_subclass(robots.ROBOT_NAME_XARM6)
class XArm6Config:
    robot_name: str = robots.ROBOT_NAME_XARM6
    laterality: Laterality = Laterality.RIGHT

    detector: list = field(default_factory=list)
    transforms: list = field(default_factory=list)
    visualizers: list = field(default_factory=list)
    robots: list = field(default_factory=list)
    operators: list = field(default_factory=list)

    def __post_init__(self):
        log_laterality_configuration(self.laterality, robots.ROBOT_NAME_XARM6)
        self._configure_right()

    def _configure_right(self):
        self.detector = [
            SharedComponentRegistry.get_detector_config(
                hand_side=robots.RIGHT,
                host=network.HOST_ADDRESS,
            )
        ]

        self.transforms = [
            SharedComponentRegistry.get_transform_config(
                hand_side=robots.RIGHT,
                host=network.HOST_ADDRESS,
                keypoint_sub_port=ports.KEYPOINT_STREAM_PORT,
                moving_average_limit=3,
            )
        ]

        self.visualizers = []

        self.robots = [
            XArm6RobotCfg(
                host=network.HOST_ADDRESS,
                robot_ip=network.RIGHT_XARM_IP,
                is_right_arm=True,
                endeff_publish_port=ports.XARM6_ENDEFF_PUBLISH_PORT,
                endeff_subscribe_port=ports.XARM6_ENDEFF_SUBSCRIBE_PORT,
                joint_subscribe_port=ports.XARM6_JOINT_SUBSCRIBE_PORT,
                reset_subscribe_port=ports.XARM6_RESET_SUBSCRIBE_PORT,
                state_publish_port=ports.XARM6_STATE_PUBLISH_PORT,
                home_subscribe_port=ports.XARM6_HOME_SUBSCRIBE_PORT,
                teleoperation_state_port=ports.XARM6_TELEOPERATION_STATE_PORT,
                gripper_subscribe_port=ports.XARM6_GRIPPER_PORT,
                hand_side=robots.RIGHT,
                recorder_config={
                    "robot_identifier": robots.ROBOT_IDENTIFIER_RIGHT_XARM6,
                    "recorded_data": [
                        robots.RECORDED_DATA_JOINT_STATES,
                        robots.RECORDED_DATA_XARM_CARTESIAN_STATES,
                        robots.RECORDED_DATA_COMMANDED_CARTESIAN_STATE,
                        robots.RECORDED_DATA_JOINT_ANGLES_RAD,
                    ],
                },
            )
        ]

        self.operators = [
            XArm6OperatorCfg(
                host=network.HOST_ADDRESS,
                transformed_keypoints_port=ports.KEYPOINT_TRANSFORM_PORT,
                stream_configs={
                    "host": network.HOST_ADDRESS,
                    "port": ports.CONTROL_STREAM_PORT,
                },
                stream_oculus=True,
                endeff_publish_port=ports.XARM6_ENDEFF_SUBSCRIBE_PORT,
                endeff_subscribe_port=ports.XARM6_ENDEFF_PUBLISH_PORT,
                moving_average_limit=3,
                arm_resolution_port=ports.KEYPOINT_STREAM_PORT,
                use_filter=False,
                teleoperation_state_port=ports.XARM6_TELEOPERATION_STATE_PORT,
                gripper_publish_port=ports.XARM6_GRIPPER_PORT,
                hand_side=robots.RIGHT,
                logging_config={
                    "enabled": False,
                    "log_dir": "logs",
                    "log_poses": True,
                    "log_prefix": "xarm6_right",
                },
            )
        ]

    def build(self):
        return {
            "robot_name": self.robot_name,
            "detector": [detector.build() for detector in self.detector],
            "transforms": [item.build() for item in self.transforms],
            "visualizers": [item.build() for item in self.visualizers],
            "robots": [item.build() for item in self.robots],
            "operators": [item.build() for item in self.operators],
        }
