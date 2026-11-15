#!/usr/bin/env python3
from math import exp

from opendbc.car import get_friction, get_safety_config, structs
from opendbc.car.byd.bydparams import get_bool
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.tuning import Tuning
from opendbc.car.byd.values import (
  BydFlags, BydSafetyFlags, CanBus, EXP_LONG_CAR, PLATFORM_QIN_SEAL06, PLATFORM_SEAL, PLATFORM_TENGSHI,
  PT_RADAR_CAR, RADAR_CAR, SAFETY_SEAL_CAR, SAFETY_TORQUE_CAR, TORQUE_LAT_CAR,
)
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import FRICTION_THRESHOLD, CarInterfaceBase, LatControlInputs, TorqueFromLateralAccelCallbackType

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool,
                                       gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    a, b, c = Tuning.LAT_SIGLIN_TABLE
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque / torque_params.latAccelFactor + friction)

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if get_bool("BydLatUseSiglin"):
      return self.torque_from_lateral_accel_siglin
    return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
    ret.dashcamOnly = False

    # panda safety: only the torque protocol (HAN/TANG family) and Seal angle
    # protocol are implemented in safety_byd.h today. Everything else is
    # surfaced read-only until its safety is written and bench validated.
    if candidate in SAFETY_SEAL_CAR:
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.SEAL.value
    elif candidate in SAFETY_TORQUE_CAR:
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.HAN_TANG_DMEV.value
    else:
      ret.dashcamOnly = True

    if get_bool("EnableExtRadar"):
      ret.radarUnavailable = False
    else:
      ret.radarUnavailable = candidate not in (PT_RADAR_CAR | RADAR_CAR)

    ret.enableBsm = 0x418 in fingerprint[CanBus.ESC]
    ret.transmissionType = TransmissionType.direct

    if candidate in TORQUE_LAT_CAR:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    else:
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.flags |= BydFlags.ANGLE_CONTROL.value

    ret.openpilotLongitudinalControl = candidate in EXP_LONG_CAR

    ret.longitudinalTuning.kpBP, ret.longitudinalTuning.kiBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV, ret.longitudinalTuning.kiV = [[1.], [0.]]
    ret.longitudinalTuning.kf = 1.0

    # car specified settings
    if candidate in TORQUE_LAT_CAR:
      ret.minEnableSpeed = -1.
      ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS
      ret.autoResumeSng = True
      ret.startingState = True
      ret.startAccel = 0.8
      ret.stopAccel = -0.3
      ret.vEgoStarting = 0.1
      ret.vEgoStopping = 0.1
      ret.longitudinalActuatorDelay = 0.5
      ret.steerActuatorDelay = 0.2
      ret.steerLimitTimer = 0.6
    elif candidate in PLATFORM_QIN_SEAL06:
      ret.minEnableSpeed = -1.
      ret.minSteerSpeed = 0.
      ret.autoResumeSng = True
      ret.startingState = True
      ret.startAccel = 0.8
      ret.stopAccel = -0.3
      ret.vEgoStarting = 0.1
      ret.vEgoStopping = 0.1
      ret.longitudinalActuatorDelay = 0.5
      ret.steerActuatorDelay = 0.3
      ret.steerLimitTimer = 0.5
    elif candidate in (PLATFORM_SEAL | PLATFORM_TENGSHI):
      ret.minEnableSpeed = -1.
      ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS
      ret.autoResumeSng = True
      ret.startingState = True
      ret.startAccel = 0.8
      ret.stopAccel = -0.3
      ret.vEgoStarting = 0.1
      ret.vEgoStopping = 0.1
      ret.longitudinalActuatorDelay = 0.5
      ret.steerActuatorDelay = 0.1
      ret.steerLimitTimer = 1.0
    else:
      ret.dashcamOnly = True

    return ret
