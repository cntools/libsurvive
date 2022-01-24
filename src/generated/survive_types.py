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
class SurviveIMUBiasModel:
    AccScale: np.array = np.array((0., 0., 0))
    IMUCorrection: np.array = np.array((1, 0., 0., 0))
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

@dataclass
class SurviveKalmanModel:
    Pose: SurvivePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    IMUBias: SurviveIMUBiasModel = SurviveIMUBiasModel()

@dataclass
class SurviveIMUBiasErrorModel:
    AccScale: np.array = np.array((0., 0., 0))
    IMUCorrection: np.array = np.array((0., 0., 0))
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

@dataclass
class SurviveKalmanErrorModel:
    Pose: SurviveAxisAnglePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    IMUBias: SurviveIMUBiasErrorModel = SurviveIMUBiasErrorModel()

@dataclass
class SurviveLighthouseModel:
    Pose: SurvivePose

@dataclass
class SurviveLighthouseErrorModel:
    Pose: SurviveAxisAnglePose

@dataclass
class BaseStationCal:
    phase: float = 0
    tilt: float = 0
    curve: float = 0
    gibpha: float = 0
    gibmag: float = 0
    ogeephase: float = 0
    ogeemag: float = 0
    def __add__(self, other):
        return BaseStationCal(

        )

@dataclass
class SurviveJointKalmanModel:
    Object: SurviveKalmanModel
    Lighthouse: SurvivePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveLighthouseKalmanModel:
    Lighthouse: SurvivePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveLighthouseKalmanErrorModel:
    Lighthouse: SurviveAxisAnglePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveJointKalmanErrorModel:
    Object: SurviveKalmanErrorModel
    Lighthouse: SurviveAxisAnglePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

Vec3d = np.array([0, 0, 0])