# XArm6 + Gripper Teleoperation Setup

Unimanual (right-hand) xArm6 with gripper, controlled via Meta Quest 3.

## Run

```bash
python -m beavr.teleop.main --robot_name=xarm6 --laterality=right
```

## What was added

### Constants
- `configs/constants/robots.py` — `ROBOT_NAME_XARM6`, `XARM6_HOME_JS` (6 joints), `ROBOT_IDENTIFIER_RIGHT_XARM6`
- `configs/constants/ports.py` — `XARM6_*` ports (10037–10050, 8189) to avoid collisions with xArm7

### Control layer
- `components/interface/controller/robots/xarm6_control.py`
  - `Robot(XArmAPI)` — same SDK as xArm7, joint count validation returns 6, uses `XARM6_HOME_JS`
  - Adds `init_gripper()`, `set_gripper_pos(0–850)`, `get_gripper_pos()`
  - `DexArmControl` wraps it, calls `init_gripper()` on startup

### Robot interface
- `components/interface/robots/xarm6_robot.py`
  - `XArm6Robot(RobotWrapper)` — mirrors XArm7Robot
  - Subscribes to `GripperCommand` on `XARM6_GRIPPER_PORT`
  - Each stream cycle: receives gripper commands, maps `width_m` (0–0.085m) to SDK units (0–850), sends to hardware
  - Publishes `gripper_position` in state dict

### Operator (retargeting + gripper)
- `components/operator/robots/xarm6_operator.py`
  - `XArm6Operator(XArmOperator)` — inherits all Cartesian retargeting from xArm7
  - Overrides `_get_hand_frame()` to also extract pinch distance (thumb tip idx 5, index tip idx 10)
  - After each retargeting cycle, publishes `GripperCommand` with width mapped from pinch distance
- `components/operator/robots/xarm6_right.py`
  - `XArm6RightOperator` — sets right-arm transform matrices, passes gripper port

### Config
- `configs/robots/xarm6_config.py`
  - `XArm6Config` registered as `"xarm6"` via `@TeleopRobotConfig.register_subclass`
  - Right-hand only, wires detector + transform + robot + operator

## What to calibrate

### Must change
- **Robot IP** — `configs/constants/network.py` → `RIGHT_XARM_IP` must match your xArm6's IP

### Likely need to change
- **Home joint state** — `configs/constants/robots.py` → `XARM6_HOME_JS`
  - Current value is derived from xArm7's first 6 joints
  - Set this to a safe neutral pose for your xArm6 + gripper setup
  - Test with xArm Studio first to find a good home pose, note the 6 joint angles in radians

### May need to tune
- **Transform matrices** — `components/operator/robots/xarm6_right.py` → `H_R_V_RIGHT`, `H_T_V_RIGHT`
  - These map VR hand tracking coordinates to robot base coordinates
  - Current values assume same mounting as xArm7 (arm in front of operator, base on table)
  - If your arm is mounted differently (rotated, on a different side), these need to change
  - They are 4x4 homogeneous rotation matrices (no translation component)

- **Pinch thresholds** — `components/operator/robots/xarm6_operator.py`
  - `_PINCH_CLOSE_THRESHOLD = 0.02` — pinch distance below this = fully closed
  - `_PINCH_OPEN_THRESHOLD = 0.06` — pinch distance above this = fully open
  - Linear interpolation between them
  - Units are in the transformed keypoint coordinate space (roughly meters)
  - If gripper feels too sensitive or too sluggish, adjust these

- **Gripper range** — `xarm6_control.py` → `set_gripper_pos()` clamps to 0–850
  - 0 = fully closed, 850 = fully open
  - This matches the UFACTORY xArm gripper default range
  - If using a different gripper, change the range in `set_gripper_pos()` and the width mapping in `xarm6_robot.py` → `_process_gripper()`

## Architecture

```
Quest 3 (hand tracking)
  → OculusVRHandDetector (26 keypoints via ZMQ)
  → TransformHandPositionCoords (VR frame → canonical frame)
  → XArm6RightOperator
      ├── Cartesian retargeting (frame_vectors → CartesianTarget)  [inherited from XArmOperator]
      └── Gripper pinch detection (keypoints → GripperCommand)     [added in XArm6Operator]
  → XArm6Robot
      ├── move_coords(CartesianTarget)  → xArm SDK set_servo_cartesian_aa
      └── _process_gripper(GripperCommand) → xArm SDK set_gripper_position
```

## Data flow (gripper)

1. `TransformHandPositionCoords` publishes `InputFrame` with both `frame_vectors` and `keypoints`
2. `XArm6Operator._get_hand_frame()` reads `InputFrame`, extracts frame for arm control AND computes pinch distance from `keypoints[5]` (thumb tip) and `keypoints[10]` (index tip)
3. `XArm6Operator._apply_retargeted_angles()` calls parent for arm, then publishes `GripperCommand` on `XARM6_GRIPPER_PORT` topic `"gripper"`
4. `XArm6Robot._process_gripper()` receives `GripperCommand`, converts `width_m` to SDK range, calls `set_gripper_position()`
