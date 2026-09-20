"""Production-oriented LiDAR ICP motion estimation utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import open3d as o3d


@dataclass(frozen=True)
class ICPConfig:
    """Configuration for frame-to-frame point-to-plane ICP."""

    voxel_size: float = 1.0
    correspondence_distance: float = 1.0
    normal_radius: float = 1.0
    normal_max_nn: int = 30
    max_iteration: int = 50
    relative_fitness: float = 0.15
    max_inlier_rmse: float = 0.50
    max_translation: float = 5.0
    max_rotation_deg: float = 45.0

    def validate(self) -> None:
        values = {
            "voxel_size": self.voxel_size,
            "correspondence_distance": self.correspondence_distance,
            "normal_radius": self.normal_radius,
            "max_iteration": self.max_iteration,
        }
        for name, value in values.items():
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.normal_max_nn < 3:
            raise ValueError("normal_max_nn must be >= 3")
        if not 0.0 <= self.relative_fitness <= 1.0:
            raise ValueError("relative_fitness must be in [0, 1]")
        if self.max_inlier_rmse <= 0:
            raise ValueError("max_inlier_rmse must be > 0")
        if self.max_translation <= 0:
            raise ValueError("max_translation must be > 0")
        if self.max_rotation_deg <= 0:
            raise ValueError("max_rotation_deg must be > 0")


@dataclass(frozen=True)
class ICPResult:
    """Result of an ICP registration attempt."""

    transformation: np.ndarray
    fitness: float
    inlier_rmse: float
    accepted: bool
    reason: str


class ICPMotionEstimator:
    """Robust wrapper around Open3D point-to-plane ICP."""

    def __init__(self, config: ICPConfig | None = None) -> None:
        self.config = config or ICPConfig()
        self.config.validate()

    @staticmethod
    def numpy_to_open3d(points: np.ndarray) -> o3d.geometry.PointCloud:
        """Convert an Nx3 NumPy array to an Open3D point cloud."""
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")

        points = points[np.all(np.isfinite(points), axis=1)]
        if points.shape[0] < 3:
            raise ValueError("at least 3 finite points are required")

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    def preprocess(
        self,
        cloud: o3d.geometry.PointCloud,
    ) -> o3d.geometry.PointCloud:
        """Voxel-downsample and validate a point cloud."""
        if not isinstance(cloud, o3d.geometry.PointCloud):
            raise TypeError("cloud must be an Open3D PointCloud")
        if len(cloud.points) < 3:
            raise ValueError("point cloud must contain at least 3 points")

        filtered = cloud.voxel_down_sample(self.config.voxel_size)
        if len(filtered.points) < 3:
            raise ValueError("voxel downsampling left fewer than 3 points")
        return filtered

    def _estimate_normals(self, cloud: o3d.geometry.PointCloud) -> None:
        """Estimate normals required by point-to-plane ICP."""
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.config.normal_radius,
                max_nn=self.config.normal_max_nn,
            )
        )
        cloud.normalize_normals()

        if len(cloud.normals) != len(cloud.points):
            raise RuntimeError("normal estimation failed")

    @staticmethod
    def _is_valid_transform(transform: np.ndarray) -> bool:
        """Validate an SE(3)-like homogeneous transform."""
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            return False

        rotation = transform[:3, :3]
        return (
            np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            and math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-3)
            and np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6)
        )

    @staticmethod
    def _motion_magnitude(transform: np.ndarray) -> tuple[float, float]:
        """Return translation magnitude and rotation angle in degrees."""
        translation = float(np.linalg.norm(transform[:3, 3]))
        trace = float(np.trace(transform[:3, :3]))
        cosine = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        angle_deg = math.degrees(math.acos(float(cosine)))
        return translation, angle_deg

    def estimate(
        self,
        previous_cloud: o3d.geometry.PointCloud,
        current_cloud: o3d.geometry.PointCloud,
        initial_transform: np.ndarray | None = None,
    ) -> ICPResult:
        """Estimate relative motion from previous scan to current scan."""
        source = self.preprocess(previous_cloud)
        target = self.preprocess(current_cloud)

        self._estimate_normals(source)
        self._estimate_normals(target)

        if initial_transform is None:
            initial_transform = np.eye(4, dtype=np.float64)
        else:
            initial_transform = np.asarray(initial_transform, dtype=np.float64)

        if not self._is_valid_transform(initial_transform):
            raise ValueError("initial_transform must be a valid 4x4 transform")

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=self.config.max_iteration,
        )

        registration = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.config.correspondence_distance,
            initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria,
        )

        transform = np.asarray(registration.transformation, dtype=np.float64)
        fitness = float(registration.fitness)
        inlier_rmse = float(registration.inlier_rmse)

        if not self._is_valid_transform(transform):
            return ICPResult(
                initial_transform.copy(),
                fitness,
                inlier_rmse,
                False,
                "invalid transformation",
            )

        translation, rotation_deg = self._motion_magnitude(transform)

        accepted = (
            np.isfinite(fitness)
            and np.isfinite(inlier_rmse)
            and fitness >= self.config.relative_fitness
            and inlier_rmse <= self.config.max_inlier_rmse
            and translation <= self.config.max_translation
            and rotation_deg <= self.config.max_rotation_deg
        )

        if accepted:
            reason = "accepted"
        elif fitness < self.config.relative_fitness:
            reason = "low fitness"
        elif inlier_rmse > self.config.max_inlier_rmse:
            reason = "high inlier RMSE"
        elif translation > self.config.max_translation:
            reason = "translation gate exceeded"
        else:
            reason = "rotation gate exceeded"

        return ICPResult(
            transform if accepted else initial_transform.copy(),
            fitness,
            inlier_rmse,
            accepted,
            reason,
        )
