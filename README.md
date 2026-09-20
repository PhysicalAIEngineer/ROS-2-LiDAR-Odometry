# ROS 2 LiDAR Odometry

A production-oriented **ROS 2 LiDAR odometry and 3D motion-estimation pipeline** built around sequential point-cloud registration and **point-to-plane ICP** using Open3D.

The system consumes LiDAR scans through ROS 2 `sensor_msgs/msg/PointCloud2`, validates and converts the data into Open3D point clouds, performs voxel-based preprocessing, estimates surface normals, registers consecutive scans using ICP, validates the estimated rigid-body transformation, accumulates vehicle pose, and publishes the resulting trajectory as `nav_msgs/msg/Odometry`.

The repository also includes configurable ICP parameters, ROS 2 launch configuration, RViz2 visualization, reusable registration utilities, unit tests, and a GitHub Actions CI pipeline.


# ROS2 Odometry Pipeline [From LiDAR PointsCloud2 to Robot Pose Using Point-to-Plane ICP]
<img width="1024" height="1536" alt="9400f2e2-4514-4822-80fb-4f27c3c64e05" src="https://github.com/user-attachments/assets/bf02eeb2-f1ed-4c1b-9d46-5109df062a0a" />



---

# System Architecture

The repository separates the ROS-facing layer from the point-cloud registration layer.
<img width="1024" height="1536" alt="4273abb7-8e9d-4b04-8b94-39539fff84ab" src="https://github.com/user-attachments/assets/bf8f1825-1476-4218-875f-30a7d09b0be0" />

---

# Pipeline

## 1. LiDAR Input

The node subscribes to:

```text
/kitti/point_cloud
```

with:

```text
sensor_msgs/msg/PointCloud2
```

The incoming ROS 2 point-cloud message is converted into a NumPy/Open3D representation.

---

## 2. Point Cloud Validation

The point-cloud conversion layer verifies that:

* Input data has the expected 3D structure.
* Points contain finite values.
* At least three valid points remain.

Invalid points containing `NaN` or `Inf` are removed before registration.

---

## 3. Voxel Downsampling

Voxel-grid downsampling reduces point density:

```python
cloud.voxel_down_sample(voxel_size)
```

The default configuration is:

```text
voxel_size = 1.0
```

The purpose is to reduce:

* computational cost
* memory usage
* redundant points

while retaining the overall geometric structure required for registration.

---

## 4. Surface Normal Estimation

Point-to-plane ICP requires target surface normals.

The implementation estimates normals using:

```text
KDTreeSearchParamHybrid
```

with configurable:

```text
normal_radius
normal_max_nn
```

Default values:

```text
normal_radius = 1.0
normal_max_nn = 30
```

---

## 5. Point-to-Plane ICP

Consecutive point clouds are registered using Open3D point-to-plane ICP.

The estimator solves for a rigid transformation:

```text
T_relative ∈ SE(3)
```

represented as:

```text
┌ R  t ┐
└ 0  1 ┘
```

where:

```text
R ∈ SO(3)
t ∈ R³
```

The default correspondence distance is:

```text
1.0 meter
```

---

# ICP Motion Estimation

The central registration module is:

```text
simple_lidar_odometry/icp_motion_estimator.py
```

It exposes:

```python
ICPMotionEstimator
```

and:

```python
ICPConfig
ICPResult
```

---

## ICPConfig

Registration parameters are represented by a typed configuration object.

Example:

```python
ICPConfig(
    voxel_size=1.0,
    correspondence_distance=1.0,
    normal_radius=1.0,
    normal_max_nn=30,
    max_iteration=50,
    relative_fitness=0.15,
    max_inlier_rmse=0.50,
    max_translation=5.0,
    max_rotation_deg=45.0,
)
```

This avoids scattering hard-coded thresholds throughout the codebase.

---

## ICPResult

Each registration produces a structured result containing:

```text
transformation
fitness
inlier_rmse
accepted
reason
```

Example:

```python
ICPResult(
    transformation=T_relative,
    fitness=0.84,
    inlier_rmse=0.17,
    accepted=True,
    reason="accepted",
)
```

---

# Registration Quality Gating

A production robotics system should not blindly integrate every ICP result.

This project therefore performs several validation checks.

## Transformation validation

The estimated transformation must:

* be a `4 × 4` matrix
* contain finite values
* have an orthonormal rotation matrix
* have determinant approximately equal to `1`
* contain a valid homogeneous bottom row

Conceptually:

```text
RᵀR ≈ I
det(R) ≈ 1
T[3] ≈ [0, 0, 0, 1]
```

---

## Fitness gate

ICP fitness must satisfy:

```text
fitness >= relative_fitness
```

Default:

```text
relative_fitness = 0.15
```

Low fitness indicates insufficient geometric correspondence.

---

## RMSE gate

The inlier RMSE must satisfy:

```text
inlier_rmse <= max_inlier_rmse
```

Default:

```text
max_inlier_rmse = 0.50
```

---

## Translation gate

The magnitude of estimated translation is checked:

```text
||t|| <= max_translation
```

Default:

```text
max_translation = 5.0 m
```

---

## Rotation gate

The relative rotation angle is checked:

```text
θ <= max_rotation_deg
```

Default:

```text
max_rotation_deg = 45°
```

These gates prevent obviously bad registrations from corrupting the accumulated trajectory.

---

# SE(3) Pose Estimation

The accumulated vehicle pose is represented by a homogeneous transformation matrix:

```text
┌ R₃×₃  t₃×₁ ┐
└ 0 0 0   1  ┘
```

The initial pose is:

```text
I₄
```

For each accepted ICP result:

```text
T_pose(k) = T_pose(k-1) · T_relative(k)
```

This produces the accumulated LiDAR trajectory.

---

# Rotation Representation

The project supports conversion between:

```text
Rotation Matrix
      ↓
Euler XYZ
```

and:

```text
Rotation Matrix
      ↓
Quaternion [x, y, z, w]
```

SciPy is used for these conversions.

Euler angles are useful for diagnostics and logging.

Quaternions are used for ROS odometry orientation fields.

---

# ROS 2 Node

The ROS-facing implementation is:

```text
simple_lidar_odometry/lidar_odometry_node.py
```

The class:

```python
LidarOdometry
```

inherits from:

```python
rclpy.node.Node
```

It handles:

* ROS parameters
* PointCloud2 subscription
* point-cloud conversion
* ICP invocation
* pose accumulation
* odometry publication
* diagnostics
* error handling

---

# ROS 2 Topics

## Input

```text
/kitti/point_cloud
```

Message:

```text
sensor_msgs/msg/PointCloud2
```

Purpose:

```text
Sequential LiDAR scans
```

---

## Output

```text
/pointcloud/odom
```

Message:

```text
nav_msgs/msg/Odometry
```

Purpose:

```text
Accumulated LiDAR odometry
```

The odometry message uses:

```text
frame_id = odom
child_frame_id = base_link
```

---

# Runtime Parameters

The ROS node exposes configurable parameters.

| Parameter                 |              Default | Description                           |
| ------------------------- | -------------------: | ------------------------------------- |
| `input_topic`             | `/kitti/point_cloud` | LiDAR input topic                     |
| `output_topic`            |   `/pointcloud/odom` | Odometry output topic                 |
| `voxel_size`              |                `1.0` | Voxel downsampling size               |
| `correspondence_distance` |                `1.0` | ICP correspondence threshold          |
| `normal_radius`           |                `1.0` | Normal estimation radius              |
| `normal_max_nn`           |                 `30` | Maximum normal neighbors              |
| `max_iteration`           |                 `50` | ICP iteration limit                   |
| `relative_fitness`        |               `0.15` | Minimum accepted ICP fitness          |
| `max_inlier_rmse`         |               `0.50` | Maximum accepted ICP RMSE             |
| `max_translation`         |                `5.0` | Maximum accepted relative translation |
| `max_rotation_deg`        |               `45.0` | Maximum accepted relative rotation    |

---

# Configuration

Runtime configuration is stored in:

```text
config/odometry.yaml
```

Example:

```yaml
/**:
  ros__parameters:
    input_topic: /kitti/point_cloud
    output_topic: /pointcloud/odom
    voxel_size: 1.0
    correspondence_distance: 1.0
    normal_radius: 1.0
    normal_max_nn: 30
    max_iteration: 50
    relative_fitness: 0.15
    max_inlier_rmse: 0.50
    max_translation: 5.0
    max_rotation_deg: 45.0
```

The main launch file loads this configuration automatically.

---

# Technology Stack

| Component            | Technology                      |
| -------------------- | ------------------------------- |
| Operating System     | Ubuntu 22.04                    |
| ROS                  | ROS 2 Humble                    |
| Language             | Python 3.10                     |
| Middleware           | ROS 2 / DDS                     |
| LiDAR Message        | `sensor_msgs/msg/PointCloud2`   |
| Odometry Message     | `nav_msgs/msg/Odometry`         |
| Point Cloud          | Open3D 0.18.0                   |
| Numerical Computing  | NumPy 1.26.4                    |
| Scientific Computing | SciPy 1.11.4                    |
| Visualization        | RViz2                           |
| Testing              | Pytest                          |
| Coverage             | pytest-cov                      |
| CI                   | GitHub Actions                  |
| Dataset Style        | KITTI LiDAR / ROS 2 PointCloud2 |

---

# Repository Structure

```text
ROS-2-LiDAR-Odometry/
│
├── .github/
│   └── workflows/
│       └── ci.yml
│
├── config/
│   ├── odometry.yaml
│   └── reliability_override.yaml
│
├── launch/
│   └── lidar_odometry.launch.py
│
├── resource/
│   └── simple_lidar_odometry
│
├── rviz/
│   └── lidar.rviz
│
├── simple_lidar_odometry/
│   ├── __init__.py
│   ├── conversions.py
│   ├── icp_motion_estimator.py
│   └── lidar_odometry_node.py
│
├── test/
│   ├── test_icp_motion_estimator.py
│   └── test_lidar_odometry.py
│
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

---

# Installation

## 1. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone https://github.com/PhysicalAIEngineer/ROS-2-LiDAR-Odometry.git
```

---

## 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

---

## 3. Install system dependencies

```bash
sudo apt update

sudo apt install -y \
  python3-pip \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  python3-pytest \
  python3-pytest-cov \
  ros-humble-rclpy \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-rviz2
```

For headless environments using Open3D, install the required OpenGL runtime library:

```bash
sudo apt install -y libgl1
```

---

# Python Dependencies

From the repository root:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

Pinned dependencies include:

```text
numpy==1.26.4
open3d==0.18.0
scipy==1.11.4
PyYAML==6.0.1
matplotlib==3.8.2
pandas==2.1.4
pytest==7.4.4
pytest-cov==4.1.0
tqdm==4.66.1
```

---

# Build

Move to the workspace:

```bash
cd ~/ros2_ws
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Verify that the package is available:

```bash
ros2 pkg list | grep simple_lidar_odometry
```

Expected result:

```text
simple_lidar_odometry
```

---

# Launch

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

```text
LiDAR Odometry Node
        +
     RViz2
```

and automatically loads:

```text
config/odometry.yaml
```

---

# Run Without RViz2

Start only the odometry node:

```bash
ros2 run simple_lidar_odometry lidar_odometry_node
```

You can then start RViz independently:

```bash
rviz2 -d \
~/ros2_ws/install/simple_lidar_odometry/share/simple_lidar_odometry/rviz/lidar.rviz
```

---

# ROS 2 Bag Playback

The node expects:

```text
/kitti/point_cloud
```

Check the available topics:

```bash
ros2 topic list
```

Inspect the topic:

```bash
ros2 topic info /kitti/point_cloud
```

Check its message type:

```bash
ros2 topic type /kitti/point_cloud
```

Expected:

```text
sensor_msgs/msg/PointCloud2
```

Inspect a ROS 2 bag before playback:

```bash
ros2 bag info <bag_directory>
```

Play the bag:

```bash
ros2 bag play <bag_directory>
```

---

# Topic Remapping

If the dataset uses a different LiDAR topic:

```bash
ros2 run simple_lidar_odometry lidar_odometry_node \
  --ros-args \
  -r /kitti/point_cloud:=/your/lidar/topic
```

Example:

```bash
ros2 run simple_lidar_odometry lidar_odometry_node \
  --ros-args \
  -r /kitti/point_cloud:=/velodyne_points
```

---

# RViz2 Visualization

The repository contains:

```text
rviz/lidar.rviz
```

The configuration uses:

```text
Fixed Frame: odom
```

The primary displays include:

```text
LiDAR PointCloud
      +
LiDAR Odometry
```

The odometry output:

```text
/pointcloud/odom
```

can be visualized in RViz.

---

# QoS Configuration

The repository also contains:

```text
config/reliability_override.yaml
```

This provides ROS 2 QoS settings for integration environments where publisher and subscriber QoS policies need to be aligned.

Typical QoS concepts include:

```text
Reliability
Durability
History
Depth
```

QoS mismatches can result in a topic appearing available but not delivering data to the subscriber.

---

# Testing

The project uses Pytest for automated testing.

Run the complete test suite:

```bash
pytest -v
```

Run the ICP-specific tests:

```bash
pytest -v test/test_icp_motion_estimator.py
```

Run the ROS-node tests:

```bash
pytest -v test/test_lidar_odometry.py
```

---

# Test Coverage

The tests cover the core functionality.

## ICP estimator tests

Coverage includes:

* configuration validation
* invalid point-cloud shape handling
* NumPy → Open3D conversion
* finite point filtering
* point-cloud preprocessing
* normal estimation
* ICP registration
* known translation recovery
* transformation validation
* invalid initial transformations

---

## ROS node tests

Coverage includes:

* initial identity pose
* Euler conversion
* quaternion conversion
* rotation round-trip
* point-cloud conversion
* ICP wrapper compatibility
* statistical outlier filtering

---

# Continuous Integration

The repository includes:

```text
.github/workflows/ci.yml
```

GitHub Actions executes the CI pipeline on pushes to `main` and pull requests.

The workflow performs:

```text
Checkout
   ↓
ROS 2 Humble Environment
   ↓
System Dependencies
   ↓
Python Dependencies
   ↓
ROS 2 Build
   ↓
Python Compilation
   ↓
Unit Tests
   ↓
Coverage
```

The CI environment uses:

```text
Ubuntu 22.04
ROS 2 Humble
Python 3.10
```

Open3D's headless runtime dependency is explicitly installed:

```text
libgl1
```

to avoid:

```text
libGL.so.1: cannot open shared object file
```

---

# Coverage Policy

Coverage is measured specifically against the core ICP estimator:

```text
simple_lidar_odometry.icp_motion_estimator
```

This keeps the coverage gate focused on the reusable registration component rather than artificially requiring unit tests to cover every ROS runtime path.

The CI gate is configured to require:

```text
80% minimum coverage
```

for the ICP estimator.

---

# Useful ROS 2 Commands

## List nodes

```bash
ros2 node list
```

## List topics

```bash
ros2 topic list
```

## Inspect the LiDAR topic

```bash
ros2 topic info /kitti/point_cloud
```

## Inspect odometry

```bash
ros2 topic echo /pointcloud/odom
```

## Measure odometry frequency

```bash
ros2 topic hz /pointcloud/odom
```

## Inspect node information

```bash
ros2 node info /lidar_odometry_node
```

## Check package installation

```bash
ros2 pkg list | grep simple_lidar_odometry
```

---

# Troubleshooting

## 1. `libGL.so.1` error

If Open3D fails during import:

```text
OSError: libGL.so.1:
cannot open shared object file
```

Install:

```bash
sudo apt install -y libgl1
```

For GitHub Actions, ensure the same dependency exists in the CI container.

---

## 2. No LiDAR data

Check:

```bash
ros2 topic list
```

and:

```bash
ros2 topic info /kitti/point_cloud
```

Then:

```bash
ros2 topic hz /kitti/point_cloud
```

If the topic is different, use topic remapping.

---

## 3. RViz is empty

Check:

```text
Fixed Frame = odom
```

and verify:

```bash
ros2 topic hz /kitti/point_cloud
ros2 topic hz /pointcloud/odom
```

Also verify the RViz display topic and QoS reliability policy.

---

## 4. ICP is unstable

ICP performance depends strongly on:

* overlap between scans
* initial motion estimate
* point density
* voxel size
* correspondence threshold
* normal estimation
* dynamic objects
* scene geometry

A large vehicle motion between consecutive frames can move the scans outside the useful convergence basin.

---

## 5. ICP returns low fitness

Possible causes include:

```text
Insufficient scan overlap
Sparse cloud
Incorrect topic
Large motion
Dynamic scene
Incorrect coordinate convention
```

Consider:

```text
smaller voxel size
larger correspondence threshold
better initial transform
motion prior
outlier filtering
```

---

## 6. ICP returns excessive RMSE

Potential causes:

```text
Poor geometry
Incorrect normals
Dynamic objects
Bad correspondence threshold
Incorrect initial transformation
```

Inspect the registration metrics:

```text
fitness
inlier_rmse
translation magnitude
rotation magnitude
```

---

## 7. ROS package cannot be found

Rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Then source:

```bash
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep simple_lidar_odometry
```

---

# Performance Considerations

ICP complexity depends heavily on point count and nearest-neighbor searches.

Voxel downsampling is therefore important for reducing computational cost.

The primary performance controls are:

```text
voxel_size
normal_radius
normal_max_nn
correspondence_distance
max_iteration
```

For higher-frequency LiDAR:

```text
smaller clouds
+
good initial transform
+
appropriate correspondence threshold
```

can substantially improve runtime.

For large-scale deployment, the next optimization targets would include:

* C++ ROS 2 implementation
* multi-threaded point-cloud processing
* Open3D tensor backend
* GPU acceleration where applicable
* scan-to-submap registration
* motion prediction
* incremental map management
* adaptive voxel filtering

---

# Coordinate Frames

The current implementation publishes odometry with:

```text
frame_id:
odom

child_frame_id:
base_link
```

The accumulated pose is represented internally using a 4×4 homogeneous transformation.

The rotation matrix is converted to a quaternion for the ROS message.

A complete TF tree for a larger autonomous-driving stack could eventually look like:

```text
map
 │
 ▼
odom
 │
 ▼
base_link
 │
 ├── lidar
 │
 ├── camera
 │
 └── imu
```

The current repository focuses on LiDAR odometry and does not implement the complete TF tree.

---

# Current Limitations

This repository is a **LiDAR odometry system**, not a complete graph-SLAM or LiDAR-inertial odometry stack.

Current limitations include:

* no loop-closure detection
* no pose-graph optimization
* no global map optimization
* no scan-to-submap backend
* no IMU preintegration
* no wheel-odometry fusion
* no LiDAR–IMU tightly coupled optimization
* no LiDAR motion deskewing
* no continuous-time trajectory estimation
* no dedicated dynamic-object masking
* no robust M-estimator loss in the ICP objective
* no explicit covariance estimation
* no degeneracy-aware Hessian analysis
* no automatic keyframe management
* no KITTI benchmark report in the repository
* no ATE/RPE evaluation pipeline
* no production hardware benchmark suite

The current implementation should therefore be understood as a strong **LiDAR odometry / registration foundation** that can be extended into a complete SLAM system.

---

# Future Extensions

Potential extensions include:

## LiDAR-Inertial Odometry

```text
LiDAR
  +
IMU
  ↓
Preintegration
  ↓
Tightly Coupled Optimization
```

---

## Scan-to-Submap Registration

Instead of:

```text
previous scan → current scan
```

use:

```text
current scan → local submap
```

which can improve robustness and reduce frame-to-frame drift.

---

## Loop Closure

Add place-recognition and loop-closure detection:

```text
Current Keyframe
       ↓
Place Recognition
       ↓
Loop Candidate
       ↓
Geometric Verification
       ↓
Pose Graph
```

---

## Pose Graph Optimization

The accumulated trajectory could become a graph:

```text
K0 ── K1 ── K2 ── K3 ── K4
      │             │
      └──── Loop ───┘
```

followed by global optimization.

---

## KITTI Benchmarking

A benchmarking layer could report:

```text
Absolute Trajectory Error
Relative Pose Error
Translation Error
Rotation Error
Trajectory Drift
Processing Time
FPS
Memory Usage
```

This would allow quantitative evaluation against standard LiDAR odometry systems.

---

# Development Workflow

Recommended development loop:

```bash
git checkout -b feature/my-change

python -m compileall simple_lidar_odometry test

pytest -q

colcon build --symlink-install

git diff

git commit -m "feat: ..."
```

Then push and allow GitHub Actions to validate the change.

---

# Research and Portfolio Value

This repository demonstrates hands-on experience with several important robotics concepts:

### Perception

```text
3D LiDAR
Point Clouds
Voxel Filtering
Surface Normals
Rigid Registration
```

### Localization

```text
Frame-to-Frame Registration
SE(3)
Pose Estimation
Odometry
```

### Robotics Software

```text
ROS 2
DDS
PointCloud2
nav_msgs/Odometry
RViz2
Launch Files
Parameters
QoS
```

### Engineering

```text
Modular Architecture
Unit Tests
Configuration Management
Error Handling
CI
Coverage
```

The separation of:

```text
ROS 2 Node
       +
ICP Estimator
       +
Tests
       +
CI
```

also makes the project easier to extend and maintain.

---

# Resume Description

### ROS 2 LiDAR Odometry

**Technologies:** Python, ROS 2 Humble, Open3D, NumPy, SciPy, ICP, KITTI, RViz2, Pytest, GitHub Actions

* Developed a ROS 2 LiDAR odometry pipeline for sequential `PointCloud2` processing, converting LiDAR scans into Open3D point clouds and estimating frame-to-frame vehicle motion.

* Implemented voxel-grid preprocessing, surface-normal estimation, and point-to-plane ICP to estimate relative rigid-body transformations between consecutive 3D LiDAR scans.

* Built SE(3)-based pose accumulation with homogeneous 4×4 transformations and rotation conversions between matrices, Euler angles, and quaternions for ROS-compatible odometry publication.

* Added production-oriented registration quality gates using ICP fitness, inlier RMSE, translation bounds, rotation bounds, and rigid-transform validation to reject unreliable motion estimates.

* Integrated automated unit testing and GitHub Actions CI with ROS 2 Humble builds, headless Open3D runtime dependencies, and coverage checks for the core ICP estimator.

---
