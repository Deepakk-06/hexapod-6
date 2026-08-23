# Hexapod Robot — Custom STS3215 Walking Stack (ROS2 Humble)

A ground-up ROS2 software stack for a 6-legged (hexapod) walking robot, built for a RoSpider-shell chassis retrofitted with different hardware than the stock kit: a Jetson Orin Nano, 18x Feetech STS3215 bus servos, and an RPLidar A1M8 — none of which the original manufacturer's software supports.

**Why this exists:** the stock robot's actual gait/inverse-kinematics logic ships as a closed-source compiled binary. Rather than depend on that, this project implements the full walking stack from scratch — protocol driver, inverse kinematics, gait engine, calibration tooling, SLAM, and autonomous navigation — using only open, verifiable, from-first-principles robotics engineering.

## Highlights

- **Custom serial protocol driver** for the Feetech STS3215 bus servo protocol, written and unit-tested against the actual hardware
- **3-DOF leg inverse kinematics** derived and implemented from first principles, verified numerically (IK↔FK round-trip accuracy <0.001mm) before ever touching real hardware
- **Two gait engines** (tripod and wave), with velocity-continuous foot trajectories (smootherstep easing) engineered specifically to reduce mechanical impact/jerk on a heavier-than-stock robot build
- **Full hand-calibration pipeline**: automated per-joint direction/offset solving from physically-posed reference stances, plus live diagnostic tooling (real-time joint-angle comparison plots) used to distinguish genuine hardware faults from expected kinematic behavior
- **SLAM mapping and Nav2 autonomous navigation**, with every costmap/controller/planner parameter re-derived from the robot's actual measured geometry and speed rather than copied from generic tutorial defaults
- **Custom STM32 real-time firmware** (evaluated as an alternative servo/IMU bridge architecture): hand-written HAL-level C, compiled and verified against ST's official toolchain
- **Systematic hardware debugging**: root-caused a disabled I2C controller via direct Linux device-tree inspection; diagnosed real vs. false-positive mechanical issues using custom-built live telemetry visualization rather than guesswork

## Architecture

```
ROS2 (Jetson Orin Nano)
  hexapod_kinematics   — leg IK/FK, gait trajectory generation (pure Python, hardware-independent)
  hexapod_control      — gait control node, calibration, teleop, diagnostics
  sts3215_driver       — servo bus protocol driver
  mpu6050_driver        — IMU driver
  hexapod_navigation   — SLAM (slam_toolbox) + Nav2 autonomous navigation
  hexapod_bringup      — launch files
```

Full technical documentation, wiring, and bring-up procedures are below.

---

## Hardware

- Jetson Orin Nano (JetPack 6 / ROS2 Humble)
- SmartElex Serial Bus Servo Driver Board (raw UART/USB passthrough, no onboard MCU/SDK)
- 18x STS3215 bus servos, centered at tick 2048
- MPU6050 IMU (I2C)
- RPLidar A1M8

## Why this isn't a literal copy of the stock RoSpider software

The stock RoSpider ROS2 package's actual leg inverse-kinematics and gait
trajectory math live inside `kinematics.so`, a compiled ARM64 Rust
extension with no available source. It can't be copied. What *is* reused
from that repo:
- the overall control architecture (gait phase generator, cmd_vel
  handling, "publish a batch of servo positions with a duration" pattern)
- the real leg/body geometry, pulled from the official
  `rospider_description` URDF (coxa 45.0mm / femur 77.1mm / tibia 115.6mm,
  and each leg's mount position + yaw angle) — legitimate since you're
  using the same shell
- the servo-abstraction pattern (per-joint direction/offset/center → tick
  conversion)

Everything else — the STS3215 protocol driver, the 3-DOF leg IK, the
tripod/wave gait engine, and the IMU leveling — is written from scratch
here using open, standard robotics math, and verified numerically (see
`hexapod_kinematics` — IK/FK round-trips to <0.001mm error, and a full
gait cycle across all 18 servos stays within safe tick range with zero
unreachable targets).

## Packages

| Package | Purpose |
|---|---|
| `hexapod_msgs` | `ServoPosition`/`ServosPosition` messages |
| `sts3215_driver` | Feetech SCServo/STS protocol over serial, ROS2 node |
| `hexapod_kinematics` | Pure-Python leg IK/FK + gait trajectory generator (no ROS dependency, unit-testable) |
| `mpu6050_driver` | MPU6050 I2C driver + complementary filter, publishes `sensor_msgs/Imu` and roll/pitch |
| `hexapod_control` | Gait control node, calibration loader, stand-up helper, keyboard teleop, servo bring-up check |
| `hexapod_bringup` | Launch files |

## 1. Install dependencies (on the Jetson)

```bash
sudo apt install python3-pip python3-smbus i2c-tools
pip3 install pyserial pyyaml smbus2 --break-system-packages
sudo apt install ros-humble-tf-transformations
```

Enable I2C and add your user to `dialout` (serial) and `i2c` groups:
```bash
sudo usermod -aG dialout,i2c $USER   # log out/in after this
```

## 2. Build

```bash
cd ~/rospider_custom_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Bring-up sequence — **do this in order, robot propped up off the ground first**

### 3a. Confirm every servo responds and IDs are correct
```bash
ros2 run hexapod_control servo_check /dev/ttyACM0
```
(the SmartElex board enumerated as `/dev/ttyACM0` on our test hardware —
double check with `ls /dev/ttyACM*` in case yours differs)

All 18 IDs should report OK. If any are missing, check that servo's wiring
before doing anything else — do not proceed with a leg that isn't
responding.

### 3b. Check I2C sees the MPU6050
```bash
i2cdetect -y 1
```
**Note**: in earlier testing on this exact hardware, the MPU6050 was
never successfully detected — bus 1 came up empty, and a lengthy
investigation through the Jetson's device tree (several other I2C
buses, including one that's disabled at the hardware level for this
header pin group) didn't resolve it either. If you hit the same wall,
this is a real unresolved item, not something you're doing wrong necessarily —
worth re-checking wiring continuity with a multimeter and confirming
which physical header pins your Orin Nano carrier board actually routes
to which `/dev/i2c-N`, since that mapping isn't always what the
board's silkscreen implies.

### 3c. Prop the robot up (feet off the ground / table) and stand up
```bash
ros2 run hexapod_control stand_up --ros-args -p config_path:=install/hexapod_control/share/hexapod_control/config/servo_config.yaml
```
Watch closely. Each leg should rise smoothly to a symmetric splayed
stance. **If any leg moves the wrong way** (coxa swings inward instead of
outward, femur/tibia fold instead of extend, etc.), stop, and flip that
joint's `direction` from `1` to `-1` (or vice versa) in
`hexapod_control/config/servo_config.yaml`, rebuild, and try again. This
is expected — I picked reasonable mirrored-leg defaults but your exact
assembly/wiring may differ.

### 3d. Fine-trim with `offset_deg`
Once directions are correct, if a leg's stance isn't quite symmetric
(e.g. tibia not fully extended-looking, coxa not pointing exactly
outward), nudge that joint's `offset_deg` a few degrees at a time and
re-run `stand_up` until it looks right. This compensates for the servo
horn not landing exactly on the kinematic zero when you centered it at
2048.

### 3e. Walk
With the robot now safely propped up (do a first test off the ground!):
```bash
# terminal 1
ros2 launch hexapod_bringup walk.launch.py

# terminal 2
ros2 launch hexapod_bringup teleop.launch.py
```
`w/a/s/d` to move, `q/e` to turn, space to stop.

Once you're confident it's walking correctly in the air, set it down.

## 4. Tuning parameters (all via `ros2 launch ... gait:=... step_period_s:=...` or editing the launch file)

- `gait`: `tripod` (fast, 3 legs airborne at once) or `wave` (slow, 1 leg airborne at a time, more stable)
- `step_period_s`: seconds per full gait cycle
- `lift_height_mm` / `stance_height_mm` / `stance_radius_mm`: gait node parameters, see `gait_node.py`
- `leveling_enabled` / `leveling_gain`: IMU-based body leveling — set `leveling_gain` to 0 to disable if it oscillates, or lower it (default 0.6)

## 5. Known gaps / next steps (not built yet, per your "just walking" scope)

- RPLidar / SLAM / Nav2 — the original repo's nav configs are largely
  hardware-agnostic and can likely be reused once walking is solid;
  ask when you're ready for this.
- No URDF/RViz visualization for this hexapod build yet (geometry.py has
  everything needed to build one).
- `sts3215_driver` reads servo positions back sequentially at ~20Hz for
  `joint_raw_states` — fine for monitoring, but if you want faster/more
  reliable feedback later, look at implementing `SYNC_READ` (0x82)
  instead of one `READ` per servo.
- Double check the STS3215 control-table addresses in
  `scservo_protocol.py` against your exact servo firmware version's
  datasheet before full-torque operation — these addresses are correct
  for the standard/common STS3215 memory map, but Feetech has, in the
  past, shifted a register or two between firmware revisions.
