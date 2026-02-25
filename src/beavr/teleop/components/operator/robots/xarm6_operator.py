import logging
import time
from typing import Any, Dict, Optional

import numpy as np

from beavr.teleop.common.network.utils import SerializationError
from beavr.teleop.components.operator import GripperCommand
from beavr.teleop.components.operator.robots.xarm7_operator import XArmOperator
from beavr.teleop.configs.constants import robots

logger = logging.getLogger(__name__)

# Thumb tip and index tip indices in Oculus 26-keypoint hand model
_THUMB_TIP_IDX = 5
_INDEX_TIP_IDX = 10

# Pinch distance thresholds (in transformed keypoint units)
_PINCH_CLOSE_THRESHOLD = 0.02
_PINCH_OPEN_THRESHOLD = 0.06


class XArm6Operator(XArmOperator):
    """XArmOperator extended with gripper control via VR pinch gesture.

    Detects thumb-to-index pinch distance from VR hand keypoints and
    publishes GripperCommand messages for the xArm6 gripper.
    """

    def __init__(
        self,
        operator_name: str,
        host: str,
        transformed_keypoints_port: int,
        stream_configs: Dict[str, Any],
        stream_oculus: bool,
        endeff_publish_port: int,
        endeff_subscribe_port: int,
        moving_average_limit: int,
        h_r_v: np.ndarray,
        h_t_v: np.ndarray,
        gripper_publish_port: int,
        use_filter: bool = True,
        arm_resolution_port: Optional[int] = None,
        teleoperation_state_port: Optional[int] = None,
        logging_config: Optional[Dict[str, Any]] = None,
        hand_side: str = robots.RIGHT,
    ):
        super().__init__(
            operator_name=operator_name,
            host=host,
            transformed_keypoints_port=transformed_keypoints_port,
            stream_configs=stream_configs,
            stream_oculus=stream_oculus,
            endeff_publish_port=endeff_publish_port,
            endeff_subscribe_port=endeff_subscribe_port,
            moving_average_limit=moving_average_limit,
            h_r_v=h_r_v,
            h_t_v=h_t_v,
            use_filter=use_filter,
            arm_resolution_port=arm_resolution_port,
            teleoperation_state_port=teleoperation_state_port,
            logging_config=logging_config,
            hand_side=hand_side,
        )
        self._gripper_publish_port = gripper_publish_port
        self._latest_pinch_distance: Optional[float] = None
        self._gripper_width_m = 0.085  # Start open (max ~85mm)

    # ------------------------------------------------------------------
    # Override _get_hand_frame to also extract pinch distance from keypoints
    # ------------------------------------------------------------------
    def _get_hand_frame(self) -> Optional[np.ndarray]:
        data = self._arm_transformed_keypoint_subscriber.recv_keypoints()

        if data is not None:
            try:
                if data.frame_vectors is not None:
                    frame_data = np.array(data.frame_vectors, dtype=np.float64).reshape(4, 3)
                    self.last_valid_hand_frame = frame_data

                # Extract pinch distance from keypoints
                if data.keypoints is not None:
                    kps = np.array(data.keypoints, dtype=np.float64).reshape(-1, 3)
                    if kps.shape[0] > max(_THUMB_TIP_IDX, _INDEX_TIP_IDX):
                        self._latest_pinch_distance = float(
                            np.linalg.norm(kps[_THUMB_TIP_IDX] - kps[_INDEX_TIP_IDX])
                        )

                if data.frame_vectors is not None:
                    return frame_data

            except Exception as e:
                logger.error(f"Error processing InputFrame data: {e}")

        if self.last_valid_hand_frame is not None:
            return self.last_valid_hand_frame

        return None

    # ------------------------------------------------------------------
    # Gripper mapping
    # ------------------------------------------------------------------
    def _compute_gripper_width(self) -> float:
        """Map pinch distance to gripper width in meters (0.0=closed, 0.085=open)."""
        if self._latest_pinch_distance is None:
            return self._gripper_width_m

        t = (self._latest_pinch_distance - _PINCH_CLOSE_THRESHOLD) / (
            _PINCH_OPEN_THRESHOLD - _PINCH_CLOSE_THRESHOLD
        )
        t = float(np.clip(t, 0.0, 1.0))
        self._gripper_width_m = t * 0.085
        return self._gripper_width_m

    def _publish_gripper_command(self):
        width = self._compute_gripper_width()
        cmd = GripperCommand(
            timestamp_s=time.time(),
            hand_side=self.hand_side,
            width_m=width,
        )
        try:
            self._publisher_manager.publish(
                host=self._publisher_host,
                port=self._gripper_publish_port,
                topic="gripper",
                data=cmd,
            )
        except (ConnectionError, SerializationError) as e:
            logger.error(f"Failed to publish gripper command: {e}")

    # ------------------------------------------------------------------
    # Override main retargeting to also publish gripper
    # ------------------------------------------------------------------
    def _apply_retargeted_angles(self):
        super()._apply_retargeted_angles()
        if self.arm_teleop_state == robots.ARM_TELEOP_CONT:
            self._publish_gripper_command()
