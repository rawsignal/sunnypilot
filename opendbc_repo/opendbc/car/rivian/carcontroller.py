import numpy as np
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal
from opendbc.car.rivian.values import CarControllerParams

from opendbc.sunnypilot.car.rivian.mads import MadsCarController

# EPS may fault if torque is applied above this angle for too long; cut request + drop torque before fault
MAX_ANGLE = 85  # deg (matches Hyundai)
MAX_ANGLE_FRAMES = 89  # ~0.9s at 100 Hz before cutting (matches Hyundai)
MAX_ANGLE_CONSECUTIVE_FRAMES = 2  # frames to cut before re-enabling (blip)
BLIP_RECOVERY_RAMP = 40  # torque units per frame when ramping back after blip


class CarController(CarControllerBase, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0
    self.recovering_from_blip = False

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, CC, CC_SP, CS)
    actuators = CC.actuators
    can_sends = []

    apply_torque = 0
    steer_max = round(float(np.interp(CS.out.vEgoRaw, CarControllerParams.STEER_MAX_LOOKUP[0],
                                      CarControllerParams.STEER_MAX_LOOKUP[1])))

    # Fault avoidance: cut request + zero torque when steering angle above limit for too long
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(
      abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
      self.angle_limit_counter, MAX_ANGLE_FRAMES, MAX_ANGLE_CONSECUTIVE_FRAMES)

    # Zero torque during blip, ramp back at BLIP_RECOVERY_RAMP per frame when recovery
    if not apply_steer_req:
      apply_torque = 0
      self.apply_torque_last = 0
      self.recovering_from_blip = False
    else:
      if self.apply_torque_last == 0:
        self.recovering_from_blip = True
      if self.mads.lat_active:
        new_torque = int(round(CC.actuators.torque * steer_max))
        # Use faster ramp rate during blip recovery (bypass normal STEER_DELTA_UP=4 limit)
        if self.recovering_from_blip:
          recovery_params = SimpleNamespace(
            STEER_MAX=CarControllerParams.STEER_MAX,
            STEER_DELTA_UP=BLIP_RECOVERY_RAMP,
            STEER_DELTA_DOWN=BLIP_RECOVERY_RAMP,
            STEER_DRIVER_ALLOWANCE=CarControllerParams.STEER_DRIVER_ALLOWANCE,
            STEER_DRIVER_MULTIPLIER=CarControllerParams.STEER_DRIVER_MULTIPLIER,
            STEER_DRIVER_FACTOR=CarControllerParams.STEER_DRIVER_FACTOR,
          )
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                          CS.out.steeringTorque, recovery_params, steer_max)
          # Reached target when rate limiter didn't need to clamp (we've caught up)
          target_with_normal_rate = apply_driver_steer_torque_limits(
            new_torque, self.apply_torque_last, CS.out.steeringTorque, CarControllerParams, steer_max)
          if apply_torque == target_with_normal_rate:
            self.recovering_from_blip = False
        else:
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams, steer_max)
      self.apply_torque_last = apply_torque

    # send steering command
    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd, apply_torque, CC.enabled, CC.latActive, self.mads, apply_steer_req))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer, self.frame, accel, CC.enabled))
    # VDM_AdasSts not available on this tap - cannot cancel stock ACC

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
