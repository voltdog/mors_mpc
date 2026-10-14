"""
Модель лидара Robosense Airy-96 для MuJoCo (трассировка лучей через MuJoCo-LiDAR).

Airy - полусферический лидар 360° x 90°: 96 каналов по углу места, 900 точек на канал
по азимуту (0.4°) при 10 Гц. Кадр мгновенный: вся геометрия берётся с одного снимка
состояния, поэтому времени точек внутри кадра нет. ПРИБЛИЖЕНИЕ: равномерная сетка углов
места, начало СК в центре нижней монтажной поверхности, нулевой азимут и порядок
каналов не сверены с документацией и записью реального Airy.
"""

import mujoco
import numpy as np
from mujoco_lidar import MjLidarWrapper
from scipy.spatial.transform import Rotation


AIRY_BODY_NAME = "robosense_airy"
AIRY_SITE_NAME = "robosense_airy"
AIRY_GEOM_NAME = "robosense_airy_body"

SCENE_GEOM_GROUP = 0
ROBOT_GEOM_GROUP = 1


class RobosenseAiryScanPattern:
    """Направления лучей кадра (theta - азимут, phi - угол места) в СК лидара."""

    def __init__(self, channels, horizontal_points, elevation_range_deg, frame_rate):
        phi = np.deg2rad(np.linspace(*elevation_range_deg, int(channels)))
        theta = 2.0 * np.pi * np.arange(int(horizontal_points)) / int(horizontal_points)
        # по столбцам, как идут пакеты вращающегося лидара: азимут снаружи, каналы внутри
        theta_grid, phi_grid = np.meshgrid(theta, phi, indexing="ij")
        self.theta = theta_grid.ravel()
        self.phi = phi_grid.ravel()
        self.ring = np.tile(np.arange(int(channels), dtype=np.uint16), int(horizontal_points))
        self.frame_period = 1.0 / float(frame_rate)

    @classmethod
    def from_config(cls, scan_cfg):
        return cls(**scan_cfg)


def _mount_pose(mount_cfg):
    pos = np.asarray(mount_cfg["xyz"], dtype=float)
    quat_wxyz = Rotation.from_euler("xyz", mount_cfg["rpy_deg"], degrees=True).as_quat(scalar_first=True)
    return pos, quat_wxyz


class RobosenseAiryLidar:
    """Лидар Airy на теле `robosense_airy`: выставляет монтаж из конфига и считает кадры облака."""

    def __init__(self, model, cfg):
        body_id = model.body(AIRY_BODY_NAME).id
        model.body_pos[body_id], model.body_quat[body_id] = _mount_pose(cfg["mount"])

        self.frame_id = cfg["frame_id"]
        self.pattern = RobosenseAiryScanPattern.from_config(cfg["scan"])
        self.range_min = float(cfg["range_min"])
        self.range_max = float(cfg["range_max"])
        self.range_noise_std = float(cfg["range_noise_std"])
        self.intensity = float(cfg["intensity"])
        self._rng = np.random.default_rng()

        geomgroup = np.zeros(mujoco.mjNGROUP, dtype=np.uint8)
        geomgroup[SCENE_GEOM_GROUP] = 1
        geomgroup[ROBOT_GEOM_GROUP] = int(bool(cfg["see_self"]))
        self._lidar = MjLidarWrapper(model,
                                     AIRY_SITE_NAME,
                                     backend="cpu",
                                     cutoff_dist=self.range_max,
                                     args={"geomgroup": geomgroup, "bodyexclude": -1})

    @property
    def frame_period(self):
        return self.pattern.frame_period

    def scan(self, data):
        """
        Args:
            data: MjData с посчитанной кинематикой (mj_kinematics / mj_forward)
        Returns:
            xyz (N, 3), intensity (N,), ring (N,) - точки в СК лидара
        """
        dist = self._lidar.trace_rays(data, self.pattern.theta, self.pattern.phi)
        points = self._lidar.get_hit_points()

        valid = (dist >= self.range_min) & (dist <= self.range_max)
        points, dist = points[valid], dist[valid]
        if self.range_noise_std > 0.0:
            noisy_dist = dist + self._rng.normal(0.0, self.range_noise_std, dist.shape)
            points = points * (noisy_dist / dist)[:, None]

        xyz = points.astype(np.float32)
        intensity = np.full(len(xyz), self.intensity, dtype=np.float32)
        return xyz, intensity, self.pattern.ring[valid]


def disable_lidar_body(model, data):
    """Лидар выключен: убрать массу и корпус Airy, чтобы модель робота не отличалась от прежней."""
    body_id = model.body(AIRY_BODY_NAME).id
    model.body_mass[body_id] = 0.0
    model.body_inertia[body_id] = 0.0
    model.geom_rgba[model.geom(AIRY_GEOM_NAME).id, 3] = 0.0
    mujoco.mj_setConst(model, data)
