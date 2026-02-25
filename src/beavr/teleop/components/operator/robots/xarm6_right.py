from typing import Any, Dict, Optional

import numpy as np

from beavr.teleop.configs.constants import robots

from .xarm6_operator import XArm6Operator

# Same transform matrices as xArm7 right — these depend on physical mounting,
# not joint count.  Recalibrate if the arm is mounted differently.
H_R_V_RIGHT = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
H_T_V_RIGHT = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])


class XArm6RightOperator(XArm6Operator):
    """Right-arm XArm6 operator with gripper pinch control."""

    def __init__(
        self,
        host: str,
        transformed_keypoints_port: int,
        stream_configs: Dict[str, Any],
        stream_oculus: bool,
        endeff_publish_port: int,
        endeff_subscribe_port: int,
        moving_average_limit: int,
        gripper_publish_port: int,
        use_filter: bool = True,
        arm_resolution_port: Optional[int] = None,
        teleoperation_state_port: Optional[int] = None,
        logging_config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(
            operator_name="xarm6_right_operator",
            host=host,
            transformed_keypoints_port=transformed_keypoints_port,
            stream_configs=stream_configs,
            stream_oculus=stream_oculus,
            endeff_publish_port=endeff_publish_port,
            endeff_subscribe_port=endeff_subscribe_port,
            moving_average_limit=moving_average_limit,
            h_r_v=H_R_V_RIGHT,
            h_t_v=H_T_V_RIGHT,
            gripper_publish_port=gripper_publish_port,
            use_filter=use_filter,
            arm_resolution_port=arm_resolution_port,
            teleoperation_state_port=teleoperation_state_port,
            logging_config=logging_config,
            hand_side=robots.RIGHT,
        )
