# Sensor Fusion with robot_localization (EKF)

Fuses wheel odometry and IMU data using an Extended Kalman Filter to produce a smooth, drift-corrected odometry estimate for a ground rover.

## Problem

Wheel odometry alone drifts in dirt due to wheel slip. The IMU provides a stable heading but no position information. By fusing both through an EKF, we get velocities from the wheels and heading corrections from the IMU — the best of both sensors.

**The catch**: The IMU is physically mounted **backwards** on the rover (rotated 180° around Z). Without correcting for this, the filter sees the IMU yaw as opposite to the wheel odometry yaw and diverges immediately.

---

## Prerequisites

- ROS 2 Humble
- `robot_localization`: `sudo apt install ros-humble-robot-localization`

## How It Works

### EKF Configuration (`ekf.yaml`)

The `robot_localization` EKF node accepts sensor inputs and a 15-element boolean array per sensor that selects which state variables to trust:

```
[x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, x_ddot, y_ddot, z_ddot]
```

**odom0 (Wheel Odometry)** — subscribes to `/odom`:
```yaml
odom0_config: [false, false, false,    # don't trust position (it drifts)
               false, false, false,    # don't trust orientation from wheels
               true,  true,  false,    # trust X and Y velocity
               false, false, true,     # trust Yaw velocity (turn rate)
               false, false, false]    # don't trust accelerations
```
Wheel encoders give reliable instantaneous velocity but accumulate position error over time from slip, so we only fuse velocities.

**imu0 (IMU)** — subscribes to `/imu/data`:
```yaml
imu0_config: [false, false, false,    # IMU has no position data
              false, false, true,     # trust Yaw orientation (absolute heading)
              false, false, false,    # no velocity from IMU
              false, false, true,     # trust Yaw rotational velocity (gyro Z)
              false, false, false]    # ignore accelerations
```
The IMU provides the absolute yaw heading that wheels can't, plus a clean gyro-based turn rate. Roll and pitch are ignored since this is a 2D ground rover (`two_d_mode: true`).

### Handling the Backwards IMU (`localization.launch.py`)

The launch file publishes a **static transform** from `base_link` → `imu_link` with a 180° yaw rotation:

```python
arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'imu_link']
#          x    y    z    yaw       pitch  roll  parent       child
```

When the EKF sees IMU data in the `imu_link` frame, it looks up the `base_link → imu_link` transform in the TF tree and rotates the IMU readings by 180° before fusing. This mathematically flips the yaw so it aligns with the robot's actual forward direction — no code changes, no sign flipping in the config, just a correct TF.

### What the EKF Produces

The `ekf_node` outputs:
- **`/odometry/filtered`** — a fused `nav_msgs/Odometry` message combining wheel velocity with IMU-corrected heading. This is what Nav2 consumes.
- **`odom → base_link` TF** — published when `publish_tf: true`, keeping the TF tree up to date for the navigation stack.

---

## Package Structure

```
mybot_localization/
├── package.xml              # Declares dependencies (robot_localization, tf2_ros)
├── CMakeLists.txt           # Installs config/ and launch/ directories
├── config/
│   └── ekf.yaml             # EKF sensor fusion configuration
└── launch/
    └── localization.launch.py  # Starts ekf_node + IMU static transform
```


## Build & Run

```bash
cd ~/mybot_ws
colcon build --packages-select mybot_localization
source install/setup.bash
ros2 launch mybot_localization localization.launch.py
```

This starts both the EKF node (listening on `/odom` and `/imu/data`) and the static transform publisher that corrects for the backwards IMU.
