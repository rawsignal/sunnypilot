import numpy as np
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal
from opendbc.car.rivian.values import CarControllerParams

from opendbc.sunnypilot.car.rivian.mads import MadsCarController

# EPS may fault if torque is applied above this angle for too long; cut request + drop torque before fault
MAX_ANGLE = 87  # deg
MAX_ANGLE_FRAMES = 89  # ~0.9s at 100 Hz before cutting (matches Hyundai)
MAX_ANGLE_CONSECUTIVE_FRAMES = 2  # frames to cut before re-enabling (blip)


class CarController(CarControllerBase, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0
    self.torque_before_blip = 0

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

    # Zero torque during blip; after blip, apply same value we had before (snap back)
    if not apply_steer_req:
      if self.apply_torque_last != 0:
        self.torque_before_blip = self.apply_torque_last
      apply_torque = 0
      self.apply_torque_last = 0
    else:
      if self.mads.lat_active:
        new_torque = int(round(CC.actuators.torque * steer_max))
        if self.apply_torque_last == 0 and self.torque_before_blip != 0:
          # Snap back: apply same value, bypass rate limit but keep driver/steer limits
          snap_params = SimpleNamespace(
            STEER_MAX=CarControllerParams.STEER_MAX,
            STEER_DELTA_UP=steer_max,
            STEER_DELTA_DOWN=steer_max,
            STEER_DRIVER_ALLOWANCE=CarControllerParams.STEER_DRIVER_ALLOWANCE,
            STEER_DRIVER_MULTIPLIER=CarControllerParams.STEER_DRIVER_MULTIPLIER,
            STEER_DRIVER_FACTOR=CarControllerParams.STEER_DRIVER_FACTOR,
          )
          apply_torque = apply_driver_steer_torque_limits(
            self.torque_before_blip, 0, CS.out.steeringTorque, snap_params, steer_max)
          self.torque_before_blip = 0
        else:
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams, steer_max)
      self.apply_torque_last = apply_torque

    # send steering command
    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd, apply_torque, CC.enabled, CC.latActive, self.mads, apply_steer_req))

    # tester present - keeps ADAS ECU disabled whenever on road (controller only runs when on road)
    if self.frame % 100 == 0:
      can_sends.append(make_tester_present_msg(0x730, 0, suppress_response=True))

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
