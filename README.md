# g1_drs_nonruntimewrapper

ROS 2 wrappers that run **onboard the Unitree G1 (PC2)**, around Unitree's own runtime. Unitree's runtime publishes the robot's state with no usable timestamps; these wrappers turn it into standard, properly stamped ROS 2 topics and TF. A separate launch file shows the live robot in RViz2 on the operator laptop.

| Package | Runs on | Contents |
|---|---|---|
| `g1_description` | PC2 + laptop | Unitree G1 URDFs and meshes, with `package://` mesh paths |
| `g1_state_bridge` | PC2 | C++ node: Unitree state topics → standard messages, stamped with the DDS source timestamp |
| `g1_bringup` | PC2 + laptop | `robot.launch.xml` (PC2) and `visualization.launch.xml` (laptop) |

```
PC1 (Unitree, closed)                  PC2 (Jetson, ROS 2 Foxy)                       Laptop (ROS 2 Humble)
/lowstate, /secondary_imu,  ──DDS──►   robot.launch.xml                       ──DDS──►  visualization.launch.xml
/odommodestate, /utlidar/*              ├─ g1_state_bridge   → state topics + odom TF    └─ RViz2 (+ rqt_tf_tree)
                                        ├─ robot_state_publisher → joint TF, /robot_description
                                        └─ static TF mid360_link → livox_frame
```

## Topics and TF (published on PC2)

| Topic | Type | From | Rate |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` (q, dq, tau_est) | `/lowstate` | ~1 kHz |
| `/imu/pelvis` | `sensor_msgs/Imu`, frame `imu_in_pelvis` | `/lowstate` | ~1 kHz |
| `/imu/torso` | `sensor_msgs/Imu`, frame `imu_in_torso` | `/secondary_imu` | ~1 kHz |
| `/odom` | `nav_msgs/Odometry`, `odom → pelvis` | `/odommodestate` | 500 Hz |
| `/remote_controller` | `sensor_msgs/Joy` | `/lowstate` | 50 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` (motor temp, voltage, errors) | `/lowstate` | 10 Hz |
| `/robot_description` | URDF | robot_state_publisher | latched |
| `/tf`, `/tf_static` | `odom → pelvis → … → mid360_link → livox_frame` | bridge + robot_state_publisher | 100 Hz |

Topic names, frames and message layouts are the same as in bags converted by `g1_drs`. Rates and names are set in [state_bridge.yaml](g1_state_bridge/config/state_bridge.yaml).

**Timestamps.** `/lowstate` and `/secondary_imu` have no header, and the stamp in `/odommodestate` is always 0. Every sample still carries the DDS *source timestamp*: PC1's clock at the moment PC1 published it. The bridge uses that as `header.stamp`, so these topics share one time base with the lidar headers (measured agreement: 0.2 ms; jitter: 0.1 ms).

- PC1's clock is not UTC (it was ~40 min behind on 2026-11-24).
- PC2's clock follows PC1's through chrony (`g1_drs/tools/pc2_time_sync`).
- The laptop can stay on UTC; RViz only compares stamps with each other.

**Robot model.** The default is `g1_29dof_rev_1_0`: our G1 reports `mode_machine` 5. If the URDF you launch doesn't match the `mode_machine` the robot reports, the bridge logs a warning.

## PC2 (onboard): install and run

The Unitree messages (`unitree_hg`, `unitree_go`) come from `~/jin_g1_ros2_ws`.

```bash
cd ~/jin_g1_ros2_ws/src
git clone <repository URL> g1_drs_nonruntimewrapper

cd ~/jin_g1_ros2_ws
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
colcon build --symlink-install --packages-select g1_description g1_state_bridge g1_bringup
```

Run it (in a shell where ROS 1 Noetic is **not** sourced):

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/jin_g1_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
ros2 launch g1_bringup robot.launch.xml
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `robot_model` | `g1_29dof_rev_1_0` | URDF name in `g1_description/urdf` |
| `tf_rate_hz` | `100.0` | Max rate of joint TF and odom TF |
| `lidar_roll`, `lidar_pitch`, `lidar_yaw` | `0.0066`, `-0.1634`, `0.0` | Calibrated rotation `mid360_link → livox_frame` [rad] |
| `lidar_x`, `lidar_y`, `lidar_z` | `0.0` | Translation `mid360_link → livox_frame` [m] |
| `bridge_config` | `state_bridge.yaml` | Bridge parameters |

For the plain Unitree URDF, without the lidar calibration: `ros2 launch g1_bringup robot.launch.xml lidar_roll:=0.0 lidar_pitch:=0.0`.

## Laptop: install and run

The laptop doesn't need the Unitree messages. It only needs `g1_description`, so RViz can load the meshes.

```bash
cd ~/ros2_ws/src && git clone <repository URL> g1_drs_nonruntimewrapper
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select g1_description g1_bringup
```

```bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces></General></Domain></CycloneDDS>'
ros2 launch g1_bringup visualization.launch.xml                  # fixed frame odom
ros2 launch g1_bringup visualization.launch.xml tf_tree_gui:=true fixed_frame:=pelvis
```

Replace `enp0s31f6` with the laptop's Ethernet interface on the robot network.

For testing without PC2, `robot.launch.xml` also runs on the laptop: it bridges PC1's topics over the network. This needs `unitree_hg` and `unitree_go` on the laptop. **Never run it on the laptop and on PC2 at the same time**: both would publish `/tf`, `/joint_states` and `/odom`.

## Not yet verified on the robot

- `/odom`, from Unitree's odometer (`/odommodestate`):
  - Which body point the pose refers to. It is published as `pelvis`; Unitree's `/dog_odom` calls it `robot_center`.
  - The frame of `velocity` (`odom.velocity_frame`, currently `world`).
  - Reset behaviour.
  - Accuracy: a community report found the position about 20% short on a G1.
- Covariances are 0 ("unknown") everywhere: Unitree reports none.

## Updating the description

```bash
g1_description/scripts/import_unitree_description.py ~/git/unitree_ros/robots/g1_description \
    g1_29dof_rev_1_0 g1_29dof_lock_waist_rev_1_0
```

The script copies the URDFs and the meshes they use, removes a floating base if there is one, rewrites the mesh paths to `package://g1_description/...`, and records the source commit in `g1_description/SOURCE`. The meshes and URDFs are © Unitree Robotics, BSD-3-Clause ([LICENSE](g1_description/LICENSE)).
