"""
Move xArm6 slowly to the teleop home position.
Run this BEFORE teleop.py to safely reach the starting pose.

Target: [0°, -30°, -30°, 0°, 20°, 0°]
"""

import math
import time

from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.231"

# Target joints in degrees — edit here if you change XARM6_HOME_JS
TARGET_DEG = [0.0, -27.0, -20.0, 0.0, 25.0, 0.0]
#TARGET_DEG = [0.0, -25.0, -1.0, 26.0, 0.0, 50.0]
TARGET_RAD = [math.radians(d) for d in TARGET_DEG]

# Speed in deg/s — keep low until you've confirmed the path is clear
SPEED_DEG_S = 10.0  # 10 deg/s, well under the 180 deg/s default
SPEED_RAD_S = math.radians(SPEED_DEG_S)

arm = XArmAPI(ROBOT_IP, is_radian=True)
arm.motion_enable(enable=True)
arm.clean_error()
arm.clean_warn()

print(f"Connected. Current joint angles (deg): "
      f"{[round(math.degrees(a), 1) for a in arm.angles]}")
print(f"Moving to {TARGET_DEG} deg at {SPEED_DEG_S} deg/s ...")
print("Press Ctrl+C to abort at any time.\n")

arm.set_mode(0)
arm.set_state(0)
time.sleep(0.5)

code = arm.set_servo_angle(
    angle=TARGET_RAD,
    speed=SPEED_RAD_S,
    mvacc=math.radians(10),   # 10 deg/s² acceleration — very gentle
    wait=True,
    is_radian=True,
)

if code == 0:
    print(f"\nDone. Final joint angles (deg): "
          f"{[round(math.degrees(a), 1) for a in arm.angles]}")
else:
    print(f"\nMotion failed with error code {code}. Check xArm Studio for details.")

arm.disconnect()
