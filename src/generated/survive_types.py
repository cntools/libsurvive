from dataclasses import dataclass

import numpy as np

@dataclass
class SurvivePose:
    Pos: np.array = np.array((0, 0, 0))
    Rot: np.array = np.array((1, 0, 0, 0))

@dataclass
class SurviveAxisAnglePose:
    Pos: np.array = np.array((0., 0., 0))
    AxisAngleRot: np.array = np.array((0, 0, 0))

@dataclass
class SurviveKalmanModel:
    Pose: SurvivePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    AccScale: float = 1
    IMUCorrection: np.array = np.array((1, 0., 0., 0))
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

@dataclass
class SurviveKalmanErrorModel:
    Pose: SurviveAxisAnglePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    AccScale: float = 1
    IMUCorrection: np.array = (1, 0., 0., 0)
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

@dataclass
class BaseStationCal:
    phase: float = 0
    tilt: float = 0
    curve: float = 0
    gibpha: float = 0
    gibmag: float = 0
    ogeephase: float = 0
    ogeemag: float = 0

Vec3d = np.array([0, 0, 0])