# ROS 2 LiDAR Odometry

A ROS 2 Python implementation of LiDAR odometry using sequential 3D point-cloud registration with Open3D point-to-plane ICP. The node subscribes to KITTI LiDAR point clouds, downsamples each scan, estimates frame-to-frame motion, accumulates the 4×4 pose transform, and publishes the estimated vehicle odometry for visualization in RViz2.

> **Project scope:** This repository implements LiDAR odometry based on scan matching and pose accumulation. It does not currently implement a pose graph optimizer or loop-closure backend, so it should not be described as full graph-based SLAM.

## Overview

The project is designed around a simple and inspectable LiDAR odometry pipeline:

```
KITTI / ROS 2 PointCloud2
          │
          ▼
  PointCloud2 Conversion
          │
          ▼
    Voxel Downsampling
          │
          ▼
 Previous Scan ───────── Current Scan
          │                    │
          └──────► ICP ◄───────┘
                  │
                  ▼
        Relative 4×4 Transform
                  │
                  ▼
        Pose / Odometry Accumulation
                  │
                  ▼
        nav_msgs/Odometry
                  │
                  ▼
          /pointcloud/odom
                  │
                  ▼
               RViz2
```

## Features

- ROS 2 Python node for LiDAR odometry.
- PointCloud2 input processing.
- Open3D point-cloud representation.
- Voxel-grid downsampling before registration.
- Point-to-plane ICP for consecutive scans.
- 4×4 homogeneous transformation matrices for motion estimation.
- Sequential pose accumulation.
- Euler-angle and quaternion conversion utilities.
- `nav_msgs/Odometry` output.
- RViz2 configuration for point-cloud and odometry visualization.
- ROS 2 QoS override configuration.
- Unit tests for transformations, point-cloud processing, ICP, and outlier filtering.

## Technology Stack

| Component | Version / Technology |
|---|---|
| OS target | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| Point-cloud processing | Open3D 0.18.0 |
| Numerical computing | NumPy 1.26.4 |
| Scientific computing | SciPy 1.11.4 |
| Visualization | RViz2 |
| Middleware | ROS 2 / DDS |
| Dataset/input | KITTI-style LiDAR PointCloud2 stream |

The exact runtime environment can vary by ROS 2 installation. Python package versions are pinned in `requirements.txt`.

## Repository Structure

```
ROS-2-LiDAR-Odometry/
│
├── README.md
├── requirements.txt
├── package.xml
├── setup.py
├── setup.cfg
│
├── resource/
│   └── simple_lidar_odometry
│
├── simple_lidar_odometry/
│   ├── __init__.py
│   ├── conversions.py
│   └── lidar_odometry_node.py
│
├── launch/
│   ├── lidar_odometry.launch.py
│   └── lidar_odomety.launch.py
│
├── config/
│   └── reliability_override.yaml
│
├── rviz/
│   └── lidar.rviz
│
└── test/
    └── test_lidar_odometry.py
```

The repository also contains IDE metadata under `.idea/`.

## Core Implementation

### 1. ROS 2 LiDAR input

The odometry node subscribes to:

```text
/kitti/point_cloud
```

using `sensor_msgs/msg/PointCloud2`.

The callback converts the ROS 2 message into an Open3D point cloud.

### 2. Point-cloud conversion

The conversion layer reads the point fields:

```text
y, x, z
```

and constructs an Open3D `PointCloud`.

A voxel grid is then applied with:

```python
voxel_size = 1
```

to reduce point density before ICP registration.

### 3. Consecutive-frame registration

For every new scan after the first scan, the project registers:

```text
previous point cloud → current point cloud
```

using Open3D point-to-plane ICP.

Surface normals are estimated before registration. The implementation uses an ICP correspondence threshold of `1.0`.

### 4. Relative transform

ICP returns a homogeneous transformation:

```text
T_relative ∈ R^(4×4)
```

which represents the estimated motion between two consecutive LiDAR frames.

### 5. Pose accumulation

The current accumulated pose is updated by matrix multiplication:

```text
T_pose ← T_pose · T_relative
```

The initial pose is:

```text
I₄
```

the 4×4 identity matrix.

### 6. Odometry output

The accumulated translation and rotation are converted into a ROS 2 `nav_msgs/Odometry` message.

The published topic is:

```text
/pointcloud/odom
```

with:

```text
frame_id = odom
```

This output is consumed by the existing RViz configuration.

## ROS 2 Topics

| Direction | Topic | Message | Purpose |
|---|---|---|---|
| Input | `/kitti/point_cloud` | `sensor_msgs/msg/PointCloud2` | Sequential LiDAR scans |
| Output | `/pointcloud/odom` | `nav_msgs/msg/Odometry` | Estimated LiDAR odometry |

The provided RViz configuration also contains displays for several KITTI image topics and comparison/auxiliary odometry topics, but the core odometry implementation in this repository uses the LiDAR point-cloud input and `/pointcloud/odom` output.

## Coordinate / Transform Handling

The implementation maintains the accumulated pose as a 4×4 homogeneous transformation matrix:

```text
┌ R₃×₃  t₃×₁ ┐
└ 0 0 0    1 ┘
```

where:

- `R` is the accumulated rotation.
- `t` is the accumulated translation.

Rotation matrices are converted to XYZ Euler angles for logging and to quaternions for ROS 2 odometry messages.

The node logs:

```text
LiDAR Odom: x: ..., y: ..., yaw: ...
```

during processing.

## Installation

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PhysicalAIEngineer/ROS-2-LiDAR-Odometry.git
```

### 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

### 3. Install ROS 2 dependencies

```bash
sudo apt update

sudo apt install -y \
  python3-pip \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  ros-humble-rclpy \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-rviz2
```

### 4. Install Python dependencies

From the repository root:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

### 5. Build the workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 6. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

## Launch the Project

The main launch file is:

```text
launch/lidar_odometry.launch.py
```

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch simple_lidar_odometry lidar_odometry.launch.py
```

The launch file starts:

1. The `lidar_odometry_node`.
2. RViz2 with `rviz/lidar.rviz`.

## Running Without the Launch File

The installed node can also be started directly:

```bash
ros2 run simple_lidar_odometry lidar_odometry_node
```

Then start RViz separately:

```bash
rviz2 -d ~/ros2_ws/install/simple_lidar_odometry/share/simple_lidar_odometry/rviz/lidar.rviz
```

## Playing LiDAR Data

The odometry node requires a PointCloud2 stream on:

```text
/kitti/point_cloud
```

When using a ROS 2 bag, inspect the bag first:

```bash
ros2 bag info <bag_directory>
```

Then play it:

```bash
ros2 bag play <bag_directory>
```

Before starting the odometry node, verify that the expected input topic exists:

```bash
ros2 topic list
```

and inspect its message type:

```bash
ros2 topic type /kitti/point_cloud
```

Expected type:

```text
sensor_msgs/msg/PointCloud2
```

> The repository does not include the large ROS 2 bag/dataset payload itself. Keep large datasets outside Git when possible and document their source separately.

## RViz2 Visualization

The supplied configuration is:

```text
rviz/lidar.rviz
```

It uses:

```text
Fixed Frame: odom
```

and contains an enabled Simple LiDAR Odometry display subscribed to:

```text
/pointcloud/odom
```

The point-cloud display listens to:

```text
kitti/point_cloud
```

This makes it possible to visualize the incoming LiDAR data and estimated odometry together.

## QoS Configuration

The repository provides:

```text
config/reliability_override.yaml
```

The current configuration contains overrides for:

- `/tf_static`
- `/scan`

including reliability, durability, and history settings.

This file is useful when integrating the project into a ROS 2 environment where publisher/subscriber QoS policies need to be aligned. It is configuration support rather than part of the ICP algorithm itself.

## Testing

Run the repository tests with:

```bash
cd ~/ros2_ws/src/ROS-2-LiDAR-Odometry

pytest -v test/test_lidar_odometry.py
```

The current tests cover:

- Initial 4×4 identity pose.
- Rotation matrix to Euler angles.
- Rotation matrix to quaternion.
- Rotation conversion round-trip.
- Open3D point-cloud conversion and voxel downsampling.
- Known-translation ICP registration.
- Statistical outlier removal.

For a full ROS 2 integration test, the test environment should also provide a sourced ROS 2 Humble installation and the built package.

## Useful ROS 2 Commands

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Inspect the point-cloud topic:

```bash
ros2 topic info /kitti/point_cloud
```

Inspect odometry output:

```bash
ros2 topic echo /pointcloud/odom
```

Check topic rate:

```bash
ros2 topic hz /pointcloud/odom
```

Inspect the node:

```bash
ros2 node info /lidar_odometry_node
```

## Troubleshooting

### No LiDAR data appears

Check:

```bash
ros2 topic list
ros2 topic info /kitti/point_cloud
```

The node expects a PointCloud2 topic named:

```text
/kitti/point_cloud
```

If your bag publishes another topic name, remap it when launching the node.

Example:

```bash
ros2 run simple_lidar_odometry lidar_odometry_node \
  --ros-args -r /kitti/point_cloud:=/your/actual/topic
```

### RViz opens but the point cloud is empty

Verify the fixed frame is:

```text
odom
```

and verify the point-cloud topic is publishing:

```bash
ros2 topic hz /kitti/point_cloud
```

Also check the RViz Reliability Policy and the publisher QoS.

### ICP fails or produces unstable motion

The implementation is sensitive to:

- Large motion between consecutive scans.
- Very sparse scans.
- Poor overlap.
- Dynamic objects.
- The voxel size.
- ICP correspondence threshold.
- Normal estimation quality.

The current implementation is intentionally simple and does not include a scan-to-submap backend, motion prior, robust loss, loop closure, or pose-graph optimization.

### `ros2 launch` cannot find the package

Make sure the workspace has been built and sourced:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Then verify:

```bash
ros2 pkg list | grep simple_lidar_odometry
```

## Limitations

This project is an educational/research-oriented LiDAR odometry implementation rather than a complete production SLAM stack.

Current limitations include:

- No pose-graph optimization.
- No loop-closure detection.
- No global map optimization.
- No IMU preintegration.
- No wheel odometry fusion.
- No scan-to-submap optimization.
- No robust backend optimization.
- No explicit dynamic-object filtering in the main callback.
- No benchmark report or KITTI odometry leaderboard evaluation included in the repository.

These limitations are important when comparing the project with production LiDAR-inertial odometry or graph-SLAM systems.

## Development Notes

The central implementation is:

```text
simple_lidar_odometry/lidar_odometry_node.py
```

The point-cloud parsing utilities are:

```text
simple_lidar_odometry/conversions.py
```

Package installation and ROS 2 entry-point configuration are defined in:

```text
setup.py
setup.cfg
package.xml
```

Launch and visualization resources are:

```text
launch/
rviz/
config/
```

Unit tests are maintained in:

```text
test/test_lidar_odometry.py
```

## Research / Portfolio Context

This repository demonstrates practical understanding of:

- ROS 2 node development.
- LiDAR point-cloud processing.
- 3D rigid registration.
- ICP-based motion estimation.
- Homogeneous transformation mathematics.
- Odometry message publication.
- RViz2-based robotics visualization.
- Python robotics tooling with Open3D, NumPy, and SciPy.

The implementation is particularly useful as a foundation for extending toward more advanced systems such as scan-to-map LiDAR odometry, LiDAR-inertial odometry, loop closure, or pose-graph SLAM.

## License

This project is released under the MIT License.

## Author

**PhysicalAIEngineer**

GitHub: https://github.com/PhysicalAIEngineer
