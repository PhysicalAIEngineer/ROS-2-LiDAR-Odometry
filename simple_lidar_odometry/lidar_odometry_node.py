"""ROS 2 LiDAR odometry node using guarded point-to-plane ICP."""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

from .conversions import read_points
from .icp_motion_estimator import ICPConfig, ICPMotionEstimator


class LidarOdometry(Node):
    """Estimate frame-to-frame LiDAR motion and publish odometry."""

    def __init__(self) -> None:
        super().__init__("lidar_odometry_node")

        self.declare_parameter("input_topic", "/kitti/point_cloud")
        self.declare_parameter("output_topic", "/pointcloud/odom")
        self.declare_parameter("voxel_size", 1.0)
        self.declare_parameter("correspondence_distance", 1.0)
        self.declare_parameter("normal_radius", 1.0)
        self.declare_parameter("normal_max_nn", 30)
        self.declare_parameter("max_iteration", 50)
        self.declare_parameter("relative_fitness", 0.15)
        self.declare_parameter("max_inlier_rmse", 0.50)
        self.declare_parameter("max_translation", 5.0)
        self.declare_parameter("max_rotation_deg", 45.0)

        config = ICPConfig(
            voxel_size=float(self.get_parameter("voxel_size").value),
            correspondence_distance=float(self.get_parameter("correspondence_distance").value),
            normal_radius=float(self.get_parameter("normal_radius").value),
            normal_max_nn=int(self.get_parameter("normal_max_nn").value),
            max_iteration=int(self.get_parameter("max_iteration").value),
            relative_fitness=float(self.get_parameter("relative_fitness").value),
            max_inlier_rmse=float(self.get_parameter("max_inlier_rmse").value),
            max_translation=float(self.get_parameter("max_translation").value),
            max_rotation_deg=float(self.get_parameter("max_rotation_deg").value),
        )
        self.icp_estimator = ICPMotionEstimator(config)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher_ = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.listener_callback, 10
        )

        self.prev_cloud = None
        self.odometry = np.eye(4, dtype=np.float64)
        self.last_relative_transform = np.eye(4, dtype=np.float64)

        self.get_logger().info(
            f"LiDAR odometry initialized: input={input_topic}, output={output_topic}"
        )

    def listener_callback(self, msg: PointCloud2) -> None:
        """Process one LiDAR frame."""
        try:
            cloud = self.pointcloud2_to_pointcloud(msg)

            if self.prev_cloud is None:
                self.prev_cloud = cloud
                self.get_logger().info("Initialized odometry from first LiDAR scan")
                return

            result = self.icp_estimator.estimate(
                self.prev_cloud, cloud, self.last_relative_transform
            )

            if not result.accepted:
                self.get_logger().warning(
                    "ICP rejected: "
                    f"reason={result.reason}, fitness={result.fitness:.4f}, "
                    f"rmse={result.inlier_rmse:.4f}"
                )
                return

            self.last_relative_transform = result.transformation
            self.odometry = self.odometry @ result.transformation

            rotation = self.odometry[:3, :3]
            translation = self.odometry[:3, 3]
            euler_deg = self.rotation_to_euler(rotation)

            self.get_logger().debug(
                f"LiDAR odom: x={translation[0]:.3f}, y={translation[1]:.3f}, "
                f"z={translation[2]:.3f}, yaw={euler_deg[2]:.3f} deg, "
                f"fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.4f}"
            )

            self.publish_odometry(translation, rotation)
            self.prev_cloud = cloud

        except (ValueError, RuntimeError, TypeError) as exc:
            self.get_logger().error(f"LiDAR frame rejected: {exc}")

    def perform_icp_point_to_plane(self, source, target):
        """Backward-compatible wrapper for the legacy ICP API."""
        result = self.icp_estimator.estimate(
            source,
            target,
            self.last_relative_transform,
        )
        self.last_relative_transform = result.transformation
        return result.transformation, result.inlier_rmse

    def remove_outliers(self, point_cloud):
        """Backward-compatible statistical outlier filtering helper."""
        if not isinstance(point_cloud, o3d.geometry.PointCloud):
            raise TypeError("point_cloud must be an Open3D PointCloud")
        if len(point_cloud.points) == 0:
            raise ValueError("point_cloud must not be empty")

        filtered, _ = point_cloud.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0,
        )
        return filtered

    def pointcloud2_to_pointcloud(self, msg: PointCloud2):
        """Convert ROS 2 PointCloud2 to a downsampled Open3D cloud."""
        data = read_points(msg, skip_nans=True, field_names=("y", "x", "z"))
        cloud = self.icp_estimator.numpy_to_open3d(data)
        return cloud

    def publish_odometry(self, translation: np.ndarray, rotation: np.ndarray) -> None:
        """Publish accumulated pose as nav_msgs/Odometry."""
        quaternion = self.rotation_to_quaternion(rotation)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"

        message.pose.pose.position.x = float(translation[0])
        message.pose.pose.position.y = float(translation[1])
        message.pose.pose.position.z = float(translation[2])
        message.pose.pose.orientation.x = float(quaternion[0])
        message.pose.pose.orientation.y = float(quaternion[1])
        message.pose.pose.orientation.z = float(quaternion[2])
        message.pose.pose.orientation.w = float(quaternion[3])

        self.publisher_.publish(message)

    @staticmethod
    def rotation_to_euler(rotation: np.ndarray) -> np.ndarray:
        return Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)

    @staticmethod
    def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
        return Rotation.from_matrix(rotation).as_quat()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LiDAR odometry")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
