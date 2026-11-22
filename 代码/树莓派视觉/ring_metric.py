"""Recover signed forward/left offsets from a calibrated camera and a known rim."""

import json
from pathlib import Path

import cv2
import numpy as np


def image_center(image_size):
    """Gripper reference in the original, uncropped camera image."""
    return tuple((dimension - 1) / 2 for dimension in image_size)


class RingMetric:
    """Calibration for one fixed arm observation pose and horizontal targets.

    X points forward and Y left. Board coordinates may use any common unit
    and origin. The known rim diameter supplies the scale. The gripper
    reference is always the original image center, not the optical principal point.
    """

    def __init__(self, image_size, camera_matrix, dist_coeffs, ground_marks, diameters_mm):
        self.image_size = tuple(image_size)
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        marks = np.asarray(ground_marks, dtype=np.float64)
        self.diameters = {int(key): float(value) for key, value in diameters_mm.items()}
        if (len(self.image_size) != 2 or any(type(v) is not int or v <= 0 for v in self.image_size)
                or self.camera_matrix.shape != (3, 3) or not np.isfinite(self.camera_matrix).all()
                or min(self.camera_matrix[0, 0], self.camera_matrix[1, 1]) <= 0
                or not np.allclose(self.camera_matrix[2], [0, 0, 1])
                or len(self.dist_coeffs) not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.dist_coeffs).all()):
            raise ValueError("相机尺寸、内参或畸变参数无效")
        if (not self.diameters or not self.diameters.keys() <= {1, 2, 3}
                or any(not np.isfinite(v) or v <= 0 for v in self.diameters.values())):
            raise ValueError("请填写已实测编号靶的正数环径")
        if marks.ndim != 2 or marks.shape[1] != 4 or len(marks) < 4 or not np.isfinite(marks).all():
            raise ValueError("需要至少4个地面标定点，每行填写 u、v、前方坐标、左侧坐标")
        pixels = np.ascontiguousarray(marks[:, :2])
        xy = marks[:, 2:]
        if (np.any(pixels < 0) or np.any(pixels >= self.image_size)
                or any(np.linalg.matrix_rank(points-points.mean(axis=0)) < 2 for points in (pixels, xy))):
            raise ValueError("标定点应在图像内，并覆盖地面上的二维区域")
        plane = np.column_stack((xy, np.zeros(len(xy))))
        ok, rvec, tvec = cv2.solvePnP(plane, pixels, self.camera_matrix, self.dist_coeffs)
        if not ok:
            raise ValueError("无法求出相机相对底盘的姿态")
        self.rotation = cv2.Rodrigues(rvec)[0]
        self.camera_position = -self.rotation.T @ tvec.ravel()
        projected = cv2.projectPoints(plane, rvec, tvec, self.camera_matrix, self.dist_coeffs)[0]
        error = np.linalg.norm(projected.reshape(-1, 2)-pixels, axis=1)
        if (not np.isfinite(error).all() or error.max() > 2 or self.camera_position[2] <= 0
                or np.any((plane @ self.rotation.T + tvec.ravel())[:, 2] <= 0)):
            raise ValueError("地面标定重投影误差过大或坐标轴方向错误")
        self.zero_pixel = image_center(self.image_size)
        zero = self._plane_points(np.asarray(self.zero_pixel).reshape(1, 2))
        if zero is None:
            raise ValueError("图像中心视线无法投影到地面")
        self.zero_on_plane = zero[0]

    @classmethod
    def load(cls, path):
        config = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        # Read old calibration files without allowing their taught UV to
        # override the image-center rule. Intrinsics/extrinsics stay intact.
        config.pop("zero_pixel", None)
        return cls(**config)

    def _plane_points(self, pixels):
        rays = cv2.undistortPoints(np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2),
                                   self.camera_matrix, self.dist_coeffs).reshape(-1, 2)
        directions = np.column_stack((rays, np.ones(len(rays)))) @ self.rotation
        if not np.isfinite(directions).all() or np.any(directions[:, 2] >= -0.1):
            return None
        return directions[:, :2] / -directions[:, 2:]

    def offset(self, ring, frame_shape):
        """Return (forward_mm, left_mm), or None if the rim cannot support it."""
        diameter = self.diameters.get(ring.number)
        if (tuple(frame_shape[:2][::-1]) != self.image_size or diameter is None
                or ring.rim_points is None or len(ring.rim_points) < 72):
            return None
        # Intersect with a horizontal plane one unit below the camera. The
        # shape has the correct proportions, with one unknown scale factor.
        xy = self._plane_points(ring.rim_points)
        if xy is None:
            return None
        mean = xy.mean(axis=0)
        local = xy - mean
        system = np.column_stack((2*local, np.ones(len(local))))
        fit, _, rank, _ = np.linalg.lstsq(system, np.sum(local**2, axis=1), rcond=None)
        radius_squared = fit[2] + np.sum(fit[:2]**2)
        if rank != 3 or not np.isfinite(radius_squared) or radius_squared <= 0:
            return None
        radius = np.sqrt(radius_squared)
        vectors = local - fit[:2]
        residual = np.abs(np.linalg.norm(vectors, axis=1)-radius)/radius
        angles = np.sort(np.arctan2(vectors[:, 1], vectors[:, 0]))
        if np.quantile(residual, 0.90) > 0.04 or np.diff(np.r_[angles, angles[0]+2*np.pi]).max() > 1.6:
            return None
        scale = diameter/(2*radius)
        center = mean + fit[:2]
        offset = scale*(center-self.zero_on_plane)
        return tuple(float(v) for v in offset)
