"""Green light / lead departure chimes, ported from sunnypilot.

Source: sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py (MIT).
No camera traffic-light classifier — an end-to-end proxy. While the car is
stopped and openpilot is not doing longitudinal control (disengaged or
lateral-only — on this fork you drop out of ACC to stop at a light):
  - green light: the driving model "wants to go" again
  - lead departure: a close lead pulls away by LEAD_DEPART_DIST_THRESHOLD

Adapted from sunnypilot's 20 Hz planner loop to selfdrived's 100 Hz loop
(timers use DT_CTRL instead of DT_MDL).

Fork-specific signal changes (measured on comma 4 / this model, drive
00000001--05edcaba20, 2026-07-20):
  This model emits a near-NULL trajectory at a dead stop: over 3486 armable
  stopped frames, position.x[-1] had p99=8.4 m (never the >30 m sunnypilot's
  port assumed) and velocity.x[0] p99=0.10 m/s. The "wants to go" signal only
  appears in the brief pre-roll window of a real launch, where velocity.x[0]
  spikes to several m/s while vEgo is still <= 0.1. So:
   - green light triggers on modelV2.velocity.x[0] > GREEN_LIGHT_V_THRESHOLD
     (with position.x[-1] kept as an OR fallback for models that do predict a
     long path at standstill), and the sustain window is short because the
     pre-roll window is short;
   - the lead source falls back to modelV2.leadsV3 when the radar lead is
     absent (radar is often disabled on this setup), with a brief presence
     hold so a flickering vision lead doesn't disarm mid-stop.
Both thresholds are cloudlog-instrumented so a re-test drive reveals the
margins for tuning.
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog

GREEN_LIGHT_X_THRESHOLD = 30.0  # m, model path endpoint that counts as "path open" (fallback signal)
GREEN_LIGHT_V_THRESHOLD = 0.5   # m/s, model's predicted current speed that counts as "wants to go" (primary)
LEAD_DEPART_DIST_THRESHOLD = 1.0  # m
GREEN_LIGHT_TRIGGER_S = 0.05  # s, sustain for green light. This model's "wants to go"
# signal at a dead stop is extremely brief: drive 00000001--05edcaba20 had exactly one
# clean launch, velocity.x[0] > 0.5 for 9 consecutive frames (90 ms). Sustain must stay
# under that window to fire at all. Noise floor is far below (v0 p99=0.10 m/s stopped),
# so 6 frames of >0.5 m/s is still a confident trigger. Re-tune once more launches are logged.
LEAD_DEPART_TRIGGER_S = 0.3  # s, sustain for lead departure
LEAD_PRESENCE_HOLD_S = 1.0   # s, keep treating a lead as present after its last detection
LEADSV3_PROB_THRESHOLD = 0.5  # model lead confidence to count as a lead
PARAMS_UPDATE_PERIOD = 5.0  # s
LOG_PERIOD_S = 1.0  # s, throttle for the periodic signal snapshot


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
    self.last_lead_seen_frame = -1
    self.last_lead_dRel = -1.0

    self.allowed = False
    self.last_allowed = False
    self.lead_allowed = False
    self.last_lead_allowed = False
    self.has_lead = False

    self.lead_depart_arm_timer = 0
    self.lead_depart_confirmed_lead = False
    self.lead_depart_armed = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
      self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

  def _resolve_lead(self, radar_lead, model_leads) -> tuple[bool, float]:
    """Prefer the radar lead; fall back to the model's vision lead (leadsV3).

    Radar is often disabled on this setup (radarUnavailable), so radarState's
    leadOne can be permanently absent while the model still sees a lead. A
    short presence hold bridges brief model dropouts so arming can complete.
    """
    if radar_lead.status:
      dRel = float(radar_lead.dRel)
    elif len(model_leads) and model_leads[0].prob > LEADSV3_PROB_THRESHOLD and len(model_leads[0].x):
      dRel = float(model_leads[0].x[0])
    else:
      dRel = -1.0

    if dRel >= 0.0:
      self.last_lead_seen_frame = self.frame
      self.last_lead_dRel = dRel
      return True, dRel

    # No lead this frame: hold the last-seen lead briefly so a flickering
    # vision lead at standstill doesn't reset the departure state machine.
    held = self.last_lead_seen_frame != -1 and \
        (self.frame - self.last_lead_seen_frame) * DT_CTRL < LEAD_PRESENCE_HOLD_S
    if held:
      return True, self.last_lead_dRel
    return False, -1.0

  def _update_alert_trigger(self, CS, CC, model_x, model_v0, radar_lead, model_leads) -> tuple[bool, bool]:
    max_idx = len(model_x) - 1
    self.has_lead, lead_dRel = self._resolve_lead(radar_lead, model_leads)

    moving = not CS.standstill and CS.vEgo > 0.1
    if moving:
      self.last_moving_frame = self.frame
    recent_moving = self.last_moving_frame != -1 and (self.frame - self.last_moving_frame) * DT_CTRL < 2.0

    self.allowed = not moving and not CS.gasPressed and not CC.longActive and not recent_moving
    # Lead departure fires even while op-long is engaged. On this fork (no-pedal
    # ACC on a pre-AP car) it stays long-engaged while stopped behind a
    # lead but cannot launch itself from a standstill (below min cruise, no
    # creep), so a lead pulling away needs a "go" ding even when engaged —
    # drive 00000007--6b36d9331b sat 12 s behind a lead at dRel 3.5 m with
    # longActive=1 the whole time, so the not-longActive gate never opened.
    # The not-moving gate still closes this once the car actually rolls. Green
    # light keeps not-longActive: if op is doing longitudinal it pulls away itself.
    self.lead_allowed = not moving and not CS.gasPressed and not recent_moving

    # Green Light Alert — the model "wants to go": predicted current speed rises
    # (primary), or the planned path endpoint opens up (fallback for models that
    # extend the path at standstill).
    path_open = (max_idx >= 0 and model_x[max_idx] > GREEN_LIGHT_X_THRESHOLD) or (model_v0 > GREEN_LIGHT_V_THRESHOLD)
    green_light_trigger = False
    if self.green_light_state == E2EStates.ARMED:
      if path_open:
        self.green_light_trigger_timer += 1
      else:
        self.green_light_trigger_timer = 0

      if self.green_light_trigger_timer * DT_CTRL > GREEN_LIGHT_TRIGGER_S:
        green_light_trigger = True
    else:
      self.green_light_trigger_timer = 0

    # Lead Departure Alert
    close_lead_valid = self.has_lead and lead_dRel < 8.0
    if self.lead_allowed and not self.last_lead_allowed and close_lead_valid:
      self.lead_depart_confirmed_lead = True
    elif not self.lead_allowed:
      self.lead_depart_confirmed_lead = False

    if self.lead_allowed and self.lead_depart_confirmed_lead and close_lead_valid:
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

      if self.lead_depart_trigger_timer * DT_CTRL > LEAD_DEPART_TRIGGER_S:
        lead_depart_trigger = True
    else:
      self.last_lead_distance = -1.0
      self.lead_depart_trigger_timer = 0

    self.last_allowed = self.allowed
    self.last_lead_allowed = self.lead_allowed

    # Tuning visibility: periodic snapshot while stopped, plus an edge log on any trigger.
    if self.allowed and self.frame % int(LOG_PERIOD_S / DT_CTRL) == 0:
      model_x_end = model_x[max_idx] if max_idx >= 0 else 0.0
      cloudlog.debug("GreenLight: allowed vEgo=%.2f v0=%.2f xEnd=%.1f lead=%s dRel=%.1f gl_state=%d ld_state=%d",
                     CS.vEgo, model_v0, model_x_end, self.has_lead, lead_dRel,
                     self.green_light_state, self.lead_depart_state)
    if green_light_trigger:
      cloudlog.warning("GreenLight: TRIGGER v0=%.2f xEnd=%.1f", model_v0,
                       model_x[max_idx] if max_idx >= 0 else 0.0)
    if lead_depart_trigger:
      cloudlog.warning("LeadDepart: TRIGGER dRel=%.1f from=%.1f", lead_dRel, self.last_lead_distance)

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

    model = sm['modelV2']
    model_v0 = model.velocity.x[0] if len(model.velocity.x) else 0.0
    green_light_trigger, lead_depart_trigger = self._update_alert_trigger(
      CS, sm['carControl'], model.position.x, model_v0, sm['radarState'].leadOne, model.leadsV3)

    self.green_light_state, green_light_alert = self._update_state_machine(
      self.green_light_state,
      self.green_light_alert_enabled,
      self.allowed and not self.has_lead,
      green_light_trigger,
    )

    self.lead_depart_state, lead_depart_alert = self._update_state_machine(
      self.lead_depart_state,
      self.lead_depart_alert_enabled,
      self.lead_allowed and self.lead_depart_armed,
      lead_depart_trigger,
    )

    return green_light_alert, lead_depart_alert
