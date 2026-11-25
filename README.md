# g1_drs_nonruntimewrapper

ROS 2 wrappers that run **onboard the Unitree G1 (PC2)**, around Unitree's own runtime. Unitree's runtime publishes the robot's state with no usable timestamps; these wrappers turn it into standard, properly stamped ROS 2 topics and TF. They also start the head camera and record datasets. A separate launch file shows the live robot in RViz2 on the operator laptop.

| Package | Runs on | Contents |
|---|---|---|
| `g1_description` | PC2 + laptop | Unitree G1 URDFs and meshes, with `package://` mesh paths |
| `g1_state_bridge` | PC2 | C++ node: Unitree state topics → standard messages, stamped with the DDS source timestamp |
| `g1_trajectory` | laptop (or PC2) | C++ node `odom_trajectory`: odometry → trajectory (`nav_msgs/Path`) for RViz |
| `g1_bringup` | PC2 + laptop | `robot.launch.xml` and `camera.launch.xml` (PC2), `record_bag.py` (PC2), `visualization.launch.xml` (laptop) |

```
PC1 (Unitree, closed)                  PC2 (Jetson, ROS 2 Foxy)                       Laptop (ROS 2 Humble)
/lowstate, /secondary_imu,  ──DDS──►   robot.launch.xml [camera:=true]        ──DDS──►  visualization.launch.xml
/odommodestate, /utlidar/*              ├─ g1_state_bridge   → state topics + odom TF    ├─ odom_trajectory → /odom/trajectory
                                        ├─ robot_state_publisher → joint TF              └─ RViz2 (+ rqt_tf_tree)
                                        ├─ static TF mid360_link → livox_frame
                                        └─ camera.launch.xml: RealSense D435i
                                       record_bag.py → ~/logs/raw/online_rosbag_output/
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
| `/tf`, `/tf_static` | `odom → pelvis → … → mid360_link → livox_frame`, `… → d435_link → camera_link → camera_*_optical_frame` | bridge, robot_state_publisher, camera | 100 Hz |
| `/camera/color/image_raw` | `sensor_msgs/Image`, rgb8 640×480 | RealSense D435i (`camera:=true`) | 30 Hz |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image`, 16UC1 [mm] 848×480 | | 30 Hz |
| `/camera/{color,depth}/camera_info` | `sensor_msgs/CameraInfo`: intrinsics | | 30 Hz |
| `/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/Extrinsics`: depth → colour | | latched |
| `/camera/{color,depth}/metadata` | `realsense2_camera_msgs/Metadata`: exposure, hardware timestamps (JSON) | | 30 Hz |

Topic names, frames and message layouts are the same as in bags converted by `g1_drs`. Rates and names of the state topics are set in [state_bridge.yaml](g1_state_bridge/config/state_bridge.yaml).

**Timestamps.** `/lowstate` and `/secondary_imu` have no header, and the stamp in `/odommodestate` is always 0. Every sample still carries the DDS *source timestamp*: PC1's clock at the moment PC1 published it. The bridge uses that as `header.stamp`, so these topics share one time base with the lidar headers (measured agreement: 0.2 ms; jitter: 0.1 ms).

- PC1's clock is not UTC (it was ~40 min behind on 2026-11-24).
- PC2's clock follows PC1's through chrony (`g1_drs/tools/pc2_time_sync`).
- The camera stamps are the camera's hardware frame times converted to PC2's clock, so they share the same time base.
- The laptop can stay on UTC; RViz only compares stamps with each other.

**Robot model.** The default is `g1_29dof_rev_1_0`: our G1 reports `mode_machine` 5. If the URDF you launch doesn't match the `mode_machine` the robot reports, the bridge logs a warning.

## PC2 (onboard): install and run

The Unitree messages (`unitree_hg`, `unitree_go`) come from `~/jin_g1_ros2_ws`. The RealSense driver is the system package `ros-foxy-realsense2-camera` (4.51.1).

```bash
cd ~/jin_g1_ros2_ws/src
git clone <repository URL> g1_drs_nonruntimewrapper

cd ~/jin_g1_ros2_ws
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
colcon build --symlink-install --packages-select g1_description g1_state_bridge g1_trajectory g1_bringup
```

Once per PC2, allow large network receive buffers. Without this, images and lidar scans are lost on the way to the recorder (see [Recording](#recording-a-dataset-pc2)):

```bash
echo net.core.rmem_max=33554432 | sudo tee /etc/sysctl.d/60-dds-receive-buffer.conf
sudo sysctl --system
```

Run it (in a shell where ROS 1 Noetic is **not** sourced):

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/jin_g1_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
ros2 launch g1_bringup robot.launch.xml camera:=true     # without camera:=true: no camera
```

Launch arguments of `robot.launch.xml`:

| Argument | Default | Meaning |
|---|---|---|
| `robot_model` | `g1_29dof_rev_1_0` | URDF name in `g1_description/urdf` |
| `camera` | `false` | Also start the head camera (`camera.launch.xml`) |
| `tf_rate_hz` | `100.0` | Max rate of joint TF and odom TF |
| `lidar_roll`, `lidar_pitch`, `lidar_yaw` | `0.0` | Correction rotation `mid360_link → livox_frame` [rad] on top of Unitree's lidar mount |
| `lidar_x`, `lidar_y`, `lidar_z` | `0.0` | Correction translation `mid360_link → livox_frame` [m] |
| `bridge_config` | `state_bridge.yaml` | Bridge parameters |

**Lidar mount.** With the plain Unitree URDF, the live floor on 2026-11-24 (`mode_machine` 5) agreed with IMU gravity within 1°. The bags of 2026-11-04 (`mode_machine` 6) instead needed a 9.4° correction: `lidar_roll:=0.0066 lidar_pitch:=-0.1634`. If the floor looks tilted, check the mount again against IMU gravity.

### Head camera

`camera.launch.xml` starts the standard RealSense driver for the head D435i (serial 347622075822): depth 848×480 and colour 640×480, both at 30 Hz. It can also be started on its own with `ros2 launch g1_bringup camera.launch.xml`. Its arguments are `serial_no`, `depth_profile`, `color_profile`, `publish_camera_tf` and `camera_parent_frame`. Measured on 2026-11-26:

- **Colour and depth are hardware-synchronized.** Their stamps agree within 0.02 ms. Each image keeps its own stamp (`enable_sync` false); with `enable_sync` true, the driver gives both images the stamp of their frameset instead.
- **What the stamp means.** The depth stamp is the end of the exposure, about half the exposure time (6 ms indoors) after its middle. `metadata` has `sensor_timestamp`, the middle of the exposure.
- **Latency.** Depth arrives about 8 ms after its stamp, colour about 32 ms.
- **Mount.** `camera_link` is placed at the URDF's `d435_link`, with no offset. The floor seen by the depth camera then agreed with the pelvis IMU within 0.7°.
- **IMU.** The camera's IMU does not work on this Jetson ("Motion Module failure"), so it is switched off.

## Recording a dataset (PC2)

With `robot.launch.xml camera:=true` running, start the recorder in a second terminal on PC2, inside `tmux` so that a dropped ssh connection doesn't stop it:

```bash
ros2 run g1_bringup record_bag.py --label lab_walk                  # Ctrl+C to stop
ros2 run g1_bringup record_bag.py --label lab_walk --duration 120   # stop after 120 s
ros2 run g1_bringup record_bag.py --dry-run                         # only the checks
```

Default topics (groups in [config/recording.yaml](g1_bringup/config/recording.yaml)):

| Group | Topics |
|---|---|
| `state` | `/joint_states`, `/imu/pelvis`, `/odom` |
| `lidar` | `/utlidar/cloud_livox_mid360`, `/utlidar/imu_livox_mid360` (acceleration in g) |
| `camera` | colour and depth images, their `camera_info` and `metadata`, `/camera/extrinsics/depth_to_color` |
| `static` | `/tf_static`: camera frames (depth ↔ colour), camera and lidar mounts, fixed robot links |

The script and `recording.yaml` also live, identical, in `g1_drs` (`tools/recording/`), where they run with `python3 record_bag.py`; `diff` the two copies after changing one.

Add optional groups with `--add-groups torso_imu raw`. `raw` records the Unitree messages themselves (`/lowstate`, `/secondary_imu`, `/odommodestate`); these need `unitree_hg` and `unitree_go` to be read later. Add single topics with `--topics`. All options are listed by `--help`.

Each recording is a folder `~/logs/raw/online_rosbag_output/record_<label>_<date>/` (change it with `--output-dir`). It holds the bag (`*.db3` + `metadata.yaml`) and:

- `recording_info.yaml`: the checks, every topic's publishers and message rate, the clock status and the command.
- `robot_description.urdf`: the URDF in use. `/robot_description` itself is not recorded, because when replayed it would replace the replay tools' own URDF.

The data rate is about 60 MB/s (3.6 GB per minute), so the bag is split every 10 GB.

**Before recording**, the script checks the following and refuses to start if any check fails (`--force` overrides):
- every topic has a publisher;
- PC2's clock follows PC1 within 1 ms (chrony);
- `net.core.rmem_max` is large enough (see the one-time setup above);
- there are at least 20 GB free.

**After recording**, it prints each topic's rate against its nominal rate and flags topics below 95%.

**Why the recorder settings matter.** With plain `ros2 bag record`, test recordings on PC2 lost 9–14% of the images, 2–5% of the lidar scans and Livox IMU samples, and about 15% of the 1 kHz state messages. The script fixes three causes:
1. It gives the recorder a 32 MB DDS receive buffer. Images arrive at 50 MB/s, and the default 208 KB buffer overflowed.
2. It sets a queue of 2 s per topic.
3. On Foxy, it adds a write cache of 200 messages; by default Foxy writes every message to disk on its own.

With these, 45 s recordings had all images and lidar scans, and lost 1 of 47,000 state messages. One test recording had a single 1 s stall, which cost 1.3% of the state messages.

## Laptop: install and run

The laptop doesn't need the Unitree messages. It needs `g1_description` (so RViz can load the meshes) and `g1_trajectory`. `g1_state_bridge` is ignored: it needs the Unitree messages, and `g1_bringup` lists it only for the PC2 launch files.

```bash
cd ~/ros2_ws/src && git clone https://github.com/ori-drs/g1_drs_nonruntimewrapper.git
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select g1_description g1_trajectory g1_bringup --packages-ignore g1_state_bridge
```

```bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces></General></Domain></CycloneDDS>'
ros2 launch g1_bringup visualization.launch.xml                  # fixed frame odom
ros2 launch g1_bringup visualization.launch.xml tf_tree_gui:=true fixed_frame:=pelvis
ros2 service call /odom_trajectory/reset std_srvs/srv/Empty      # clear the trajectory
```

Replace `enp0s31f6` with the laptop's Ethernet interface on the robot network.

RViz shows, in the `odom` frame:
- the robot model and its TF frames;
- the lidar cloud;
- the trajectory (`/odom/trajectory`, built on the laptop by `odom_trajectory` from `/odom`);
- the colour and depth images (in the right-hand panels).

The images are about 50 MB/s over the network. They arrive complete if the laptop's `net.core.rmem_max` is large (it is on our laptop). If they stutter, set it as on PC2.

For testing without PC2, `robot.launch.xml` also runs on the laptop: it bridges PC1's topics over the network. This needs `unitree_hg` and `unitree_go` on the laptop. **Never run it on the laptop and on PC2 at the same time**: both would publish `/tf`, `/joint_states` and `/odom`.

## Not yet verified on the robot

- `/odom`, from Unitree's odometer (`/odommodestate`):
  - Which body point the pose refers to. It is published as `pelvis`; Unitree's `/dog_odom` calls it `robot_center`.
  - The frame of `velocity` (`odom.velocity_frame`, currently `world`).
  - Reset behaviour.
  - Accuracy: a community report found the position about 20% short on a G1.
- The time offset between the camera and the IMUs. It can be measured from a recording that starts with the robot turning in place left and right for a few seconds (camera motion vs gyroscope).
- Covariances are 0 ("unknown") everywhere: Unitree reports none.

## Updating the description

```bash
g1_description/scripts/import_unitree_description.py ~/git/unitree_ros/robots/g1_description \
    g1_29dof_rev_1_0 g1_29dof_lock_waist_rev_1_0
```

The script copies the URDFs and the meshes they use, removes a floating base if there is one, rewrites the mesh paths to `package://g1_description/...`, and records the source commit in `g1_description/SOURCE`. The meshes and URDFs are © Unitree Robotics, BSD-3-Clause ([LICENSE](g1_description/LICENSE)).
