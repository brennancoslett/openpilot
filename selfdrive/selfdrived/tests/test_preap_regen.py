from types import SimpleNamespace

from openpilot.selfdrive.selfdrived.preap_regen import (
  REGEN_DEMAND_EVIDENCE_COUNT,
  RegenDemandCheck,
  required_lead_decel,
)

# get_preap_accel_limits floor is -1.5 m/s²; -2.0 clears the trigger margin.
OVERFLOW_TARGET = -2.0
# Inside the envelope: the plan can no longer report the overflow on its own
# once the MPC solves within the Pre-AP regen limits.
FITTING_TARGET = -1.0


def _lead(*, d_rel, v_lead, status=True):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead)


def _update(check, *, a_target=OVERFLOW_TARGET, v_ego=15.0,
            pedal_long_active=True, brake_pressed=False, lead=None):
  return check.update(
    pedal_long_active=pedal_long_active,
    brake_pressed=brake_pressed,
    a_target=a_target,
    v_ego=v_ego,
    lead=lead,
  )


def test_demand_prompt_requires_sustained_overflow():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT - 1):
    assert not _update(check)
  assert _update(check)


def test_demand_prompt_silent_when_plan_fits_envelope():
  check = RegenDemandCheck()
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, a_target=-1.5)


def test_demand_prompt_survives_single_sample_dropouts():
  check = RegenDemandCheck()
  fired = False
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    for _ in range(9):
      fired = _update(check) or fired
    fired = _update(check, a_target=-1.6) or fired
    if fired:
      break
  assert fired


def test_demand_prompt_does_not_fire_at_standstill():
  check = RegenDemandCheck()
  for _ in range(2 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, v_ego=0.0)


def test_demand_prompt_clears_when_driver_brakes():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  assert not _update(check, brake_pressed=True)
  assert not check.active


def test_demand_prompt_uses_hysteresis_before_clearing():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  # Back inside the trigger margin but still beyond the clear margin.
  assert _update(check, a_target=-1.6, v_ego=1.5)

  # Demand returns to the envelope: prompt clears.
  assert not _update(check, a_target=-1.5)
  assert not check.active


def test_demand_prompt_resets_when_pedal_long_inactive():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  assert not _update(check, pedal_long_active=False)
  assert not check.active


def test_closing_lead_prompts_even_when_the_plan_fits_the_envelope():
  # 15 m/s onto a stopped lead 20 m ahead: 14 m of usable gap needs about
  # 8 m/s², five times what regen can deliver, while the plan itself is
  # pinned inside the envelope and reports nothing.
  check = RegenDemandCheck()
  lead = _lead(d_rel=20.0, v_lead=0.0)
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT - 1):
    assert not _update(check, a_target=FITTING_TARGET, lead=lead)
  assert _update(check, a_target=FITTING_TARGET, lead=lead)


def test_gentle_overtake_of_a_moving_lead_is_not_a_demand():
  # Closing 5 m/s over 54 m of gap is 0.23 m/s². Measuring absolute speeds
  # instead of the difference would read this as 2.5 m/s² and prompt.
  check = RegenDemandCheck()
  lead = _lead(d_rel=60.0, v_lead=25.0)
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, a_target=FITTING_TARGET, v_ego=30.0, lead=lead)


def test_lead_pulling_away_is_not_a_demand():
  check = RegenDemandCheck()
  lead = _lead(d_rel=12.0, v_lead=20.0)
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, a_target=FITTING_TARGET, v_ego=15.0, lead=lead)


def test_lead_demand_clears_once_the_gap_is_recoverable():
  check = RegenDemandCheck()
  close_lead = _lead(d_rel=20.0, v_lead=0.0)
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check, a_target=FITTING_TARGET, lead=close_lead)
  assert check.active

  assert not _update(check, a_target=FITTING_TARGET, lead=_lead(d_rel=200.0, v_lead=14.0))
  assert not check.active


def test_dropped_lead_track_is_not_read_as_a_satisfied_demand():
  assert required_lead_decel(15.0, None) == 0.0
  assert required_lead_decel(15.0, _lead(d_rel=20.0, v_lead=0.0, status=False)) == 0.0


def test_lead_inside_the_stop_distance_stays_finite():
  demand = required_lead_decel(15.0, _lead(d_rel=6.0, v_lead=0.0))
  assert demand < -1.5
  assert demand > -1e6


def test_unusable_radar_values_are_ignored():
  assert required_lead_decel(15.0, _lead(d_rel=float("nan"), v_lead=0.0)) == 0.0
  assert required_lead_decel(15.0, _lead(d_rel=20.0, v_lead=float("inf"))) == 0.0
