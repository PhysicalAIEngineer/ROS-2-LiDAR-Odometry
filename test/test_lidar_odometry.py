import numpy as np
import open3d as o3d
import pytest

from scipy.spatial.transform import Rotation

from simple_lidar_odometry.lidar_odometry_node import LidarOdometry


@pytest.fixture
def odometry_node(monkeypatch):
    """Create the odometry class without creating a live ROS 2 node."""
    class DummyNode:
        def get_logger(self):
            return self

        def info(self, *_args, **_kwargs):
            pass

        def get_clock(self):
            return self

        def now(self):
            return self

        def to_msg(self):
            return None

        def create_publisher(self, *_args, **_kwargs):
            return None

        def create_subscription(self, *_args, **_kwargs):
            return None

    import rclpy

    monkeypatch.setattr(rclpy, "create_node", lambda *_args, **_kwargs: DummyNode())

    if not rclpy.ok():
        rclpy.init(args=[])

    node = LidarOdometry()

    yield node

    if rclpy.ok():
        rclpy.shutdown()


def test_initial_odometry_is_identity(odometry_node):
    """The accumulated pose starts at the 4x4 identity transform."""
    np.testing.assert_allclose(
        odometry_node.odometry,
        np.eye(4),
        atol=1e-12,
    )


def test_rotation_to_euler_identity(odometry_node):
    """Identity rotation should produce zero XYZ Euler angles."""
    result = odometry_node.rotation_to_euler(np.eye(3))

    np.testing.assert_allclose(
        result,
        np.zeros(3),
        atol=1e-12,
    )


def test_rotation_to_quaternion_identity(odometry_node):
    """Identity rotation should produce the identity quaternion."""
    result = odometry_node.rotation_to_quaternion(np.eye(3))

    # scipy returns [x, y, z, w].
    np.testing.assert_allclose(
        np.abs(result),
        np.array([0.0, 0.0, 0.0, 1.0]),
        atol=1e-12,
    )


def test_rotation_conversion_round_trip(odometry_node):
    """Euler -> rotation -> quaternion should preserve the same orientation."""
    expected_rotation = Rotation.from_euler(
        "xyz",
        [10.0, -5.0, 25.0],
        degrees=True,
    ).as_matrix()

    quaternion = odometry_node.rotation_to_quaternion(expected_rotation)
    reconstructed_rotation = Rotation.from_quat(quaternion).as_matrix()

    np.testing.assert_allclose(
        reconstructed_rotation,
        expected_rotation,
        atol=1e-10,
    )


def test_pointcloud_downsampling(odometry_node):
    """Point-cloud conversion/downsampling should return an Open3D point cloud."""
    rng = np.random.default_rng(42)

    # Two dense clusters with duplicate/nearby points so voxel filtering
    # should reduce the number of points.
    points = np.vstack(
        [
            rng.normal(loc=[0.0, 0.0, 0.0], scale=0.01, size=(500, 3)),
            rng.normal(loc=[5.0, 5.0, 1.0], scale=0.01, size=(500, 3)),
        ]
    )

    class DummyMsg:
        pass

    msg = DummyMsg()

    def fake_read_points(_msg, **_kwargs):
        return points

    import simple_lidar_odometry.lidar_odometry_node as module

    original_read_points = module.read_points
    module.read_points = fake_read_points

    try:
        cloud = odometry_node.pointcloud2_to_pointcloud(msg)
    finally:
        module.read_points = original_read_points

    assert isinstance(cloud, o3d.geometry.PointCloud)
    assert len(cloud.points) == len(points)
    assert np.all(np.isfinite(np.asarray(cloud.points)))


def test_icp_recovers_known_translation(odometry_node):
    """Point-to-plane ICP should recover a small known rigid translation."""
    base_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.5, 0.5, 0.0],
            [0.25, 0.75, 0.0],
            [0.75, 0.25, 0.0],
            [1.5, 0.5, 0.0],
        ],
        dtype=float,
    )

    translation = np.array([0.08, -0.04, 0.0])
    target_points = base_points + translation

    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(base_points)

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(target_points)

    source.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=2.0,
            max_nn=8,
        )
    )
    target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=2.0,
            max_nn=8,
        )
    )

    transform, rmse = odometry_node.perform_icp_point_to_plane(
        source,
        target,
    )

    assert np.isfinite(rmse)
    np.testing.assert_allclose(
        transform[:3, 3],
        translation,
        atol=2e-2,
    )
    np.testing.assert_allclose(
        transform[:3, :3],
        np.eye(3),
        atol=2e-2,
    )


def test_remove_outliers(odometry_node):
    """Statistical outlier removal should reduce an injected isolated point."""
    points = np.array(
        [
            [0.00, 0.00, 0.00],
            [0.02, 0.01, 0.00],
            [0.01, 0.02, 0.00],
            [0.00, 0.01, 0.01],
            [0.01, 0.00, 0.02],
            [0.02, 0.02, 0.01],
            [100.0, 100.0, 100.0],
        ],
        dtype=float,
    )

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)

    filtered = odometry_node.remove_outliers(cloud)

    assert len(filtered.points) < len(points)
    assert len(filtered.points) > 0
