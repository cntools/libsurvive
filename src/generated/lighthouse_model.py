import math

import cnkalman.codegen as cg
import lighthouse_gen1
import lighthouse_gen2
from common_math import *
from kalman_kinematics import *
from survive_types import *


def invert_pose(obj_p):
    r = quatgetreciprocal(quatnormalize(obj_p.Rot))
    X,Y,Z = quatrotatevector(r, obj_p.Pos)
    return SurvivePose(Pos=[-X,-Y,-Z], Rot = [*r])


@cg.generate_code()
def LighthouseIMUPrediction(x1: SurvivePose):
    G = [ 0, 0, 1 ]
    return quatrotatevector(x1.Rot, G)

@cg.generate_code()
def SurviveJointKalmanModelToErrorModel(x1: SurviveJointKalmanModel, x0: SurviveJointKalmanModel):
    return SurviveJointKalmanErrorModel(
        Object=SurviveKalmanModelToErrorModel(x1.Object, x0.Object),
        Lighthouse=SurvivePoseToErrorModel(x1.Lighthouse, x0.Lighthouse)
    )


@cg.generate_code()
def SurviveJointKalmanModelAddErrorModel(x0: SurviveJointKalmanModel, error_state: SurviveJointKalmanErrorModel):
    return SurviveJointKalmanModel(
        Object=SurviveKalmanModelAddErrorModel(x0.Object, error_state.Object),
        Lighthouse=SurvivePoseAddErrorModel(x0.Lighthouse, error_state.Lighthouse)
    )


def SurviveJointKalmanModel_LightMeas(dt: float, fn, x0: SurviveJointKalmanModel, sensor_pt: list,
                                      bsc0: BaseStationCal):
    x1 = SurviveKalmanModelPredict(dt, x0.Object)
    return fn(x1.Pose, sensor_pt, invert_pose(x0.Lighthouse), bsc0)


def SurviveJointKalmanErrorModel_LightMeas(dt: float, fn, x0: SurviveJointKalmanModel, error_model: SurviveJointKalmanErrorModel,
                                          sensor_pt: list, bsc0: BaseStationCal):
    x1 = SurviveJointKalmanModelAddErrorModel(x0, error_model)
    x2Object = SurviveKalmanModelPredict(dt, x1.Object)
    return fn(x2Object.Pose, sensor_pt, invert_pose(x1.Lighthouse), bsc0)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_x_gen1(dt: float, x0: SurviveJointKalmanModel, error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list, bsc0: BaseStationCal):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_x, x0, error_model, sensor_pt,
                                                  bsc0)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_y_gen1(dt: float, x0: SurviveJointKalmanModel, error_model: SurviveJointKalmanErrorModel,
                                                  sensor_pt: list, bsc0: BaseStationCal):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_y, x0, error_model, sensor_pt,
                                                  bsc0)

@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_x_gen2(dt: float, x0: SurviveJointKalmanModel, error_model: SurviveJointKalmanErrorModel,
                                             sensor_pt: list, bsc0: BaseStationCal):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_x_gen2, x0, error_model, sensor_pt,
                                                 bsc0)


@cg.generate_code()
def SurviveJointKalmanErrorModel_LightMeas_y_gen2(dt: float, x0: SurviveJointKalmanModel, error_model: SurviveJointKalmanErrorModel,
                                             sensor_pt: list, bsc0: BaseStationCal):
    return SurviveJointKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_y_gen2, x0, error_model, sensor_pt,
                                                 bsc0)
