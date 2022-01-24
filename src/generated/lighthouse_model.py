import math

import cnkalman.codegen as cg
import lighthouse_gen1
import lighthouse_gen2
from common_math import *
from kalman_kinematics import *
from survive_types import *


def invert_pose(obj_p):
    r = quatgetreciprocal(quatnormalize(obj_p.Rot))
    X, Y, Z = quatrotatevector(r, obj_p.Pos)
    return SurvivePose(Pos=[-X, -Y, -Z], Rot=[*r])


@cg.generate_code()
def LighthouseIMUPrediction(x1: SurvivePose):
    G = [0, 0, 1]
    r = quatgetreciprocal(quatnormalize(x1.Rot))
    return quatrotatevector(r, G)


@cg.generate_code()
def LighthouseErrorIMUPrediction(x1: SurvivePose, error_state: SurviveAxisAnglePose):
    return LighthouseIMUPrediction(SurvivePoseAddErrorModel(x1, error_state))


def BaseStationCalToErrorModel(x1: BaseStationCal, x0: BaseStationCal):
    return BaseStationCal(
        phase=x1.phase - x0.phase,
        tilt=x1.tilt - x0.tilt,
        curve=x1.curve - x0.curve,
        gibpha=x1.gibpha - x0.gibpha,
        gibmag=x1.gibmag - x0.gibmag,
        ogeephase=x1.ogeephase - x0.ogeephase,
        ogeemag=x1.ogeemag - x0.ogeemag,
    )


def BaseStationCalAddErrorModel(x0: BaseStationCal, x1: BaseStationCal):
    return BaseStationCal(
        phase=x1.phase + x0.phase,
        tilt=x1.tilt + x0.tilt,
        curve=x1.curve + x0.curve,
        gibpha=x1.gibpha + x0.gibpha,
        gibmag=x1.gibmag + x0.gibmag,
        ogeephase=x1.ogeephase + x0.ogeephase,
        ogeemag=x1.ogeemag + x0.ogeemag,
    )

@cg.generate_code()
def SurviveLighthouseKalmanModelToErrorModel(x1: SurviveLighthouseKalmanModel, x0: SurviveLighthouseKalmanModel):
    return SurviveLighthouseKalmanErrorModel(
        Lighthouse = SurvivePoseToErrorModel(x1.Lighthouse, x0.Lighthouse),
        BSD0=BaseStationCalToErrorModel(x1.BSD0, x0.BSD0),
        BSD1=BaseStationCalToErrorModel(x1.BSD1, x0.BSD1),
    )

@cg.generate_code()
def SurviveLighthouseKalmanModelAddErrorModel(x0: SurviveLighthouseKalmanModel, error_state: SurviveLighthouseKalmanErrorModel):
    return SurviveLighthouseKalmanModel(
        Lighthouse=SurvivePoseAddErrorModel(x0.Lighthouse, error_state.Lighthouse),
        BSD0=BaseStationCalAddErrorModel(x0.BSD0, error_state.BSD0),
        BSD1=BaseStationCalAddErrorModel(x0.BSD1, error_state.BSD1),
    )

@cg.generate_code()
def SurviveJointKalmanModelToErrorModel(x1: SurviveJointKalmanModel, x0: SurviveJointKalmanModel):
    return SurviveJointKalmanErrorModel(
        Object=SurviveKalmanModelToErrorModel(x1.Object, x0.Object),
        Lighthouse=SurvivePoseToErrorModel(x1.Lighthouse, x0.Lighthouse),
        BSD0=BaseStationCalToErrorModel(x1.BSD0, x0.BSD0),
        BSD1=BaseStationCalToErrorModel(x1.BSD1, x0.BSD1),
    )


@cg.generate_code()
def SurviveJointKalmanModelAddErrorModel(x0: SurviveJointKalmanModel, error_state: SurviveJointKalmanErrorModel):
    return SurviveJointKalmanModel(
        Object=SurviveKalmanModelAddErrorModel(x0.Object, error_state.Object),
        Lighthouse=SurvivePoseAddErrorModel(x0.Lighthouse, error_state.Lighthouse),
        BSD0=BaseStationCalAddErrorModel(x0.BSD0, error_state.BSD0),
        BSD1=BaseStationCalAddErrorModel(x0.BSD1, error_state.BSD1),
    )

def SurviveJointKalmanModel_LightMeas(dt: float, fn, x0: SurviveJointKalmanModel, sensor_pt: list,
                                      axis: int):
    x1 = SurviveKalmanModelPredict(dt, x0.Object)
    return fn(x1.Pose, sensor_pt, invert_pose(x0.Lighthouse), x0.BSD[axis])


def SurviveJointKalmanErrorModel_LightMeas(dt: float, fn, x0: SurviveJointKalmanModel,
                                           error_model: SurviveJointKalmanErrorModel,
                                           sensor_pt: list, axis: int):
    x1 = SurviveJointKalmanModelAddErrorModel(x0, error_model)
    x2Object = SurviveKalmanModelPredict(dt, x1.Object)
    return fn(x2Object.Pose, sensor_pt, invert_pose(x1.Lighthouse), x1.BSD0 if axis == 0 else x1.BSD1)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_x_gen1(dt: float, x0: SurviveJointKalmanModel,
                                                  error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_x, x0, error_model, sensor_pt,
                                                  0)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_y_gen1(dt: float, x0: SurviveJointKalmanModel,
                                                  error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_y, x0, error_model, sensor_pt,
                                                  1)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_x_gen2(dt: float, x0: SurviveJointKalmanModel,
                                                  error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_x_gen2, x0, error_model, sensor_pt,
                                                  0)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_y_gen2(dt: float, x0: SurviveJointKalmanModel,
                                                  error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_y_gen2, x0, error_model, sensor_pt,
                                                  1)
