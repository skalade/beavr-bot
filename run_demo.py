#!/usr/bin/env python3
"""
ONE-CLICK xArm6 + Quest 3 teleop demo launcher.

==============================================================
  EDIT ONLY THE TWO IPs BELOW, THEN RUN:   python run_demo.py
==============================================================
"""

HOST_IP = "10.114.156.114"   # <-- this PC's IP (type THIS into the Quest 3 app)
ARM_IP = "192.168.1.231"     # <-- the xArm6 controller IP

# ============================================================
#  Nothing below here needs editing.
# ============================================================
#
# What this does, in order:
#   1. Writes HOST_IP + ARM_IP into network.py and dev.yaml so every teleop
#      component agrees on the same addresses (both files matter: the robot
#      config bakes in the network.py constants, dev.yaml overrides the rest).
#   2. Pings the arm so you find a cabling/IP problem before anything moves.
#   3. Launches teleop. Teleop homes the arm to XARM6_HOME_JS on startup at a
#      slow, controlled speed, then waits for the Quest before it tracks.
#
# Stop anytime with Ctrl+C in this terminal -- the arm holds its last position.

import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))

# (file, {regex_with_two_groups: new_value}) -- the regex keeps the surrounding
# quotes/key as groups 1 and 2 and replaces only the value between them.
_PATCHES = {
    "src/beavr/teleop/configs/constants/network.py": {
        r'(HOST_ADDRESS\s*=\s*")[^"]+(")': HOST_IP,
        r'(RIGHT_XARM_IP\s*=\s*")[^"]+(")': ARM_IP,
        r'(XARM_RIGHT_IP\s*=\s*")[^"]+(")': ARM_IP,
    },
    "configs/environment/dev.yaml": {
        r'(host_address:\s*")[^"]+(")': HOST_IP,
        r'(right_xarm_ip:\s*")[^"]+(")': ARM_IP,
        r'(xarm_right_ip:\s*")[^"]+(")': ARM_IP,
    },
}


def _venv_python():
    """Always use the repo's own .venv (where beavr is installed), no matter
    which interpreter launched this script."""
    venv_py = os.path.join(REPO_ROOT, ".venv", "bin", "python")
    if not os.path.exists(venv_py):
        sys.exit(
            f"ERROR: expected project venv at {venv_py} but it is missing.\n"
            "Create it / install beavr there, or fix the .venv path."
        )
    return venv_py


def patch_ips():
    print(f"Setting HOST_IP={HOST_IP}  ARM_IP={ARM_IP}")
    for rel_path, patterns in _PATCHES.items():
        path = os.path.join(REPO_ROOT, rel_path)
        text = open(path).read()
        for pattern, value in patterns.items():
            text, n = re.subn(pattern, rf"\g<1>{value}\g<2>", text)
            if n == 0:
                print(f"  WARNING: no match for {pattern!r} in {rel_path}")
        open(path, "w").write(text)
        print(f"  patched {rel_path}")


def ping_arm():
    print(f"\nPinging arm at {ARM_IP} ...")
    ok = subprocess.run(
        ["ping", "-c", "1", "-W", "2", ARM_IP],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0
    if ok:
        print("  arm reachable.")
    else:
        print(f"  WARNING: no ping reply from {ARM_IP}. Check the ethernet cable / arm power.")
    return ok


def launch_teleop():
    print("\n" + "=" * 60)
    print("CLEAR THE AREA AROUND THE ARM.")
    print("On startup the arm homes itself (slow). Then put on the Quest 3,")
    print(f"open the beavr app, and enter this PC's IP:  {HOST_IP}")
    print("Wait for 'TELEOP RESET COMPLETE' before moving your hand.")
    print("Ctrl+C here stops the demo (arm holds position).")
    print("=" * 60)
    try:
        input("\nPress Enter to start (Ctrl+C to abort)... ")
    except KeyboardInterrupt:
        print("\nAborted before launch. Nothing moved.")
        return 0

    cmd = [_venv_python(), "-m", "beavr.teleop.main",
           "--robot_name=xarm6", "--laterality=right"]
    return subprocess.run(cmd, cwd=REPO_ROOT).returncode


def main():
    patch_ips()
    ping_arm()
    return launch_teleop()


if __name__ == "__main__":
    sys.exit(main())
