"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import messaging
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision


class SmartCruiseControl:
  def __init__(self, CP):
    self.vision = SmartCruiseControlVision(CP)

  def update(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> None:
    self.vision.update(sm, sm['carControl'].longActive, v_ego, a_ego, v_cruise)
