import numpy as np
import open3d as o3d
import pytest

from simple_lidar_odometry.icp_motion_estimator import ICPConfig, ICPMotionEstimator


def make_cloud() -> o3d.geometry.PointCloud:
    points = np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0], [1.0, 1.0, 1.0], [0.5, 0.2, 1.4],
        [1.3, 0.7, 0.4], [0.2, 1.4, 0.6], [1.4, 1.3, 1.2],
    ], dtype=np.float64)
    return ICPMotionEstimator.numpy_to_open3d(points)


def test_config_rejects_invalid_values():
    with pytest.raises(ValueError):
        ICPConfig(voxel_size=0).validate()
    with pytest.raises(ValueError):
        ICPConfig(relative_fitness=1.5).validate()


def test_numpy_conversion_rejects_bad_shape():
    estimator = ICPMotionEstimator(ICPConfig(voxel_size=0.05, normal_radius=0.5))
    with pytest.raises(ValueError):
        estimator.numpy_to_open3d(np.zeros((3, 2)))


def test_icp_recovers_small_translation():
    source = make_cloud()
    translation = np.array([0.08, -0.04, 0.0])
    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(
        np.asarray(source.points) + translation
    )

    estimator = ICPMotionEstimator(ICPConfig(
        voxel_size=0.01,
        correspondence_distance=0.5,
        normal_radius=0.5,
        normal_max_nn=8,
        max_inlier_rmse=0.2,
        relative_fitness=0.1,
    ))
    result = estimator.estimate(source, target)

    assert result.accepted
    assert np.isfinite(result.inlier_rmse)
    np.testing.assert_allclose(
        result.transformation[:3, 3], translation, atol=2e-2
    )


def test_bad_initial_transform_is_rejected():
    estimator = ICPMotionEstimator()
    with pytest.raises(ValueError):
        estimator.estimate(make_cloud(), make_cloud(), np.zeros((3, 3)))
