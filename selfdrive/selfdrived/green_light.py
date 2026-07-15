"""Green light / lead departure chimes, ported from sunnypilot.

Source: sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py (MIT).
No camera traffic-light classifier — an end-to-end proxy. While the car is
stopped and openpilot is disengaged:
  - green light: the model's planned path endpoint opens up past
    GREEN_LIGHT_X_THRESHOLD, i.e. the model "wants to go"
  - lead departure: a close lead pulls away by LEAD_DEPART_DIST_THRESHOLD

Adapted from sunnypilot's 20 Hz planner loop to selfdrived's 100 Hz loop
(timers use DT_CTRL instead of DT_MDL).
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL

GREEN_LIGHT_X_THRESHOLD = 30  # m, model path endpoint that counts as "path open"
LEAD_DEPART_DIST_THRESHOLD = 1.0  # m
TRIGGER_TIMER_THRESHOLD = 0.3  # s
PARAMS_UPDATE_PERIOD = 5.0  # s


class E2EStates:
  INACTIVE = 0
  ARMED = 1
  CONSUMED = 2


class GreenLightHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.green_light_state = E2EStates.INACTIVE
    self.lead_depart_state = E2EStates.INACTIVE

    self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
    self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

    self.green_light_trigger_timer = 0
    self.lead_depart_trigger_timer = 0
    self.last_lead_distance = -1.0
    self.last_moving_frame = -1

    self.allowed = False
    self.last_allowed = False
    self.has_lead = False

    self.lead_depart_arm_timer = 0
    self.lead_depart_confirmed_lead = False
    self.lead_depart_armed = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
      self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

  def _update_alert_trigger(self, CS, CC, model_x, lead_one) -> tuple[bool, bool]:
    max_idx = len(model_x) - 1
    self.has_lead = lead_one.status
    lead_dRel = lead_one.dRel

    moving = not CS.standstill and CS.vEgo > 0.1
    if moving:
      self.last_moving_frame = self.frame
    recent_moving = self.last_moving_frame == -1 or (self.frame - self.last_moving_frame) * DT_CTRL < 2.0

    self.allowed = not moving and not CS.gasPressed and not CC.enabled and not recent_moving

    # Green Light Alert
    green_light_trigger = False
    if self.green_light_state == E2EStates.ARMED:
      if max_idx >= 0 and model_x[max_idx] > GREEN_LIGHT_X_THRESHOLD:
        self.green_light_trigger_timer += 1
      else:
        self.green_light_trigger_timer = 0

      if self.green_light_trigger_timer * DT_CTRL > TRIGGER_TIMER_THRESHOLD:
        green_light_trigger = True
    else:
      self.green_light_trigger_timer = 0

    # Lead Departure Alert
    close_lead_valid = self.has_lead and lead_dRel < 8.0
    if self.allowed and not self.last_allowed and close_lead_valid:
      self.lead_depart_confirmed_lead = True
    elif not self.allowed:
      self.lead_depart_confirmed_lead = False

    if self.allowed and self.lead_depart_confirmed_lead and close_lead_valid:
      self.lead_depart_arm_timer += 1
      if self.lead_depart_arm_timer * DT_CTRL >= 1.0:
        self.lead_depart_armed = True
    else:
      self.lead_depart_arm_timer = 0
      self.lead_depart_armed = False

    lead_depart_trigger = False
    if self.lead_depart_state == E2EStates.ARMED:
      if self.last_lead_distance == -1 or lead_dRel < self.last_lead_distance:
        self.last_lead_distance = lead_dRel

      if self.last_lead_distance != -1 and (lead_dRel - self.last_lead_distance > LEAD_DEPART_DIST_THRESHOLD):
        self.lead_depart_trigger_timer += 1
      else:
        self.lead_depart_trigger_timer = 0

      if self.lead_depart_trigger_timer * DT_CTRL > TRIGGER_TIMER_THRESHOLD:
        lead_depart_trigger = True
    else:
      self.last_lead_distance = -1.0
      self.lead_depart_trigger_timer = 0

    self.last_allowed = self.allowed

    return green_light_trigger, lead_depart_trigger

  @staticmethod
  def _update_state_machine(state: int, enabled: bool, allowed: bool, triggered: bool) -> tuple[int, bool]:
    if state != E2EStates.INACTIVE:
      if not allowed or not enabled:
        state = E2EStates.INACTIVE
      elif state == E2EStates.ARMED and triggered:
        state = E2EStates.CONSUMED
    else:
      if allowed and enabled:
        state = E2EStates.ARMED
      triggered = False

    return state, triggered

  def update(self, CS, sm) -> tuple[bool, bool]:
    """Returns (green_light_alert, lead_depart_alert). Call at 100 Hz from selfdrived."""
    self.frame += 1
    self._read_params()

    if not (self.green_light_alert_enabled or self.lead_depart_alert_enabled):
      return False, False

    green_light_trigger, lead_depart_trigger = self._update_alert_trigger(
      CS, sm['carControl'], sm['modelV2'].position.x, sm['radarState'].leadOne)

    self.green_light_state, green_light_alert = self._update_state_machine(
      self.green_light_state,
      self.green_light_alert_enabled,
      self.allowed and not self.has_lead,
      green_light_trigger,
    )

    self.lead_depart_state, lead_depart_alert = self._update_state_machine(
      self.lead_depart_state,
      self.lead_depart_alert_enabled,
      self.allowed and self.lead_depart_armed,
      lead_depart_trigger,
    )

    return green_light_alert, lead_depart_alert
