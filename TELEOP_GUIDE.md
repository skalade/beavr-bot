# xArm6 Teleop Guide

## Prerequisites

- PC and Vision Pro on the same network (phone hotspot works)
- xArm6 connected to PC via direct ethernet at `192.168.1.231`
- venv activated: `source .venv/bin/activate`

---

## Startup sequence

**1. Move the arm to a safe home position first**

Before launching anything, run the homing script if the arm is in an unknown position:

```bash
source .venv/bin/activate
python go_to_home.py
```

This moves the arm slowly to `[0°, -30°, -30°, 0°, 20°, 0°]` at 10 deg/s. Wait for it to finish before continuing.

**2. Check your HOST_ADDRESS matches your PC's IP**

```bash
hostname -I | awk '{print $1}'   # get your PC's IP
python set_host_ip.py <pc-ip>    # e.g. python set_host_ip.py 10.246.12.114
```

**3. Put on the Vision Pro and open the beavr app**

Enter your **PC's IP** (not the headset IP) in the app. The headset sends to the PC.

**4. Launch recording (starts teleop internally)**

```bash
python src/beavr/scripts/control_robot.py \
    --robot.type=xarm6_only_adapter \
    --teleop.robot_name=xarm6 \
    --control.type=record \
    --control.video=true \
    --control.fps=30 \
    --control.num_episodes=10 \
    --control.warmup_time_s=5 \
    --control.episode_time_s=30 \
    --control.reset_time_s=5 \
    --control.repo_id=your_hf_username/your_dataset_name \
    --control.single_task="Describe your task here" \
    --control.resume=false
```

Wait until you see `TELEOP RESET COMPLETE` before moving your hands.

---

## Starting hand position (important)

When `TELEOP RESET COMPLETE` appears, the system captures your exact hand pose as the **motion baseline**. Every robot command after that is a delta from this captured pose, scaled to 0.5×.

**Hold your hand like this at reset:**
- Arm relaxed, elbow bent at roughly 90°
- Hand at mid-height in front of you
- Not reaching forward, not pulled back
- Palm facing roughly downward or forward (natural grip orientation)

This gives you equal room to move in all directions before hitting your joint limits.

**Do not move your hand while the reset is happening.** The first valid hand frame after `TELEOP RESET COMPLETE` is the zero point.

---

## During teleop

| Situation | What to do |
|-----------|-----------|
| Arm drifts to awkward pose | Pause via VR controller button, reposition yourself, unpause — triggers a re-reset |
| Arm moves unexpectedly | Hold your hand still; it will settle (filter lag is intentional) |
| Something goes wrong | `Ctrl+C` in the teleop terminal — arm stops safely |
| Need more reach | Increase `resolution_scale` in `xarm7_operator.py:155` (default 0.5, max ~1.0) |

**Move slowly at first.** Test with ~5cm movements before committing to full range.

**Avoid fast wrist rotations.** Orientation is filtered more aggressively than position — rapid flips can cause a jump. Rotate slowly and deliberately.

**Watch the desk axis.** Moving your hand downward moves the gripper toward the desk. Be aware of this especially when the arm is already in a low position.

**Gripper control:** Pinch thumb + index together to close, spread apart to open.
- Close threshold: ~2cm between fingertips
- Open threshold: ~6cm between fingertips

---

## Dataset recording

See the startup sequence above — recording is launched via the same `control_robot.py` command in step 4. Do not run `teleop.py` separately.

Cameras are currently configured as:
- `front` → `/dev/video4`
- `overhead` → `/dev/video6`

To change camera indices, edit `XArm6OnlyAdapterConfig` in:
`src/beavr/lerobot/common/robot_devices/robots/configs.py`

---

## Shutdown

1. `Ctrl+C` on the teleop terminal
2. The arm holds its last position — run `go_to_home.py` to return it to the safe home pose before walking away
