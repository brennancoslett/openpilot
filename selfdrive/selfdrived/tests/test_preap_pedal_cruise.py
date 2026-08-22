from openpilot.selfdrive.selfdrived.preap_pedal_cruise import PedalCruiseStatus


def _update(status, *, pedal_long_active=True, cruise_enabled=True, long_override=False):
  return status.update(
    pedal_long_active=pedal_long_active,
    cruise_enabled=cruise_enabled,
    long_override=long_override,
  )


def test_engage_and_disengage_report_once():
  status = PedalCruiseStatus()
  assert _update(status) == (True, False)
  assert _update(status) == (False, False)
  assert _update(status, pedal_long_active=False) == (False, True)
  assert _update(status, pedal_long_active=False) == (False, False)


def test_accelerator_override_is_not_a_disengagement():
  status = PedalCruiseStatus()
  _update(status)

  # Authority goes back to the driver's foot for the whole override.
  for _ in range(200):
    assert _update(status, pedal_long_active=False, long_override=True) == (False, False)

  # And comes back without announcing an engagement that never lapsed.
  assert _update(status) == (False, False)
  assert status.engaged


def test_feathering_the_pedal_stays_silent():
  status = PedalCruiseStatus()
  _update(status)

  # Easing on and off the accelerator without fully lifting crosses the
  # override threshold repeatedly. None of those crossings is an engagement
  # change, so none of them chimes.
  for _ in range(20):
    assert _update(status, pedal_long_active=False, long_override=True) == (False, False)
    assert _update(status) == (False, False)


def test_losing_cruise_during_an_override_still_reports():
  status = PedalCruiseStatus()
  _update(status)
  _update(status, pedal_long_active=False, long_override=True)

  assert _update(status, pedal_long_active=False, cruise_enabled=False,
                 long_override=True) == (False, True)


def test_brake_during_an_override_reports_when_the_override_ends():
  # The brake drops longitudinal while keeping lateral, so cruise stays
  # enabled and the override holds the report until the foot comes off.
  status = PedalCruiseStatus()
  _update(status)
  assert _update(status, pedal_long_active=False, long_override=True) == (False, False)
  assert _update(status, pedal_long_active=False) == (False, True)


def test_override_before_the_first_engagement_reports_nothing():
  status = PedalCruiseStatus()
  for _ in range(10):
    assert _update(status, pedal_long_active=False, long_override=True) == (False, False)
  assert not status.engaged

  assert _update(status) == (True, False)


def test_reset_clears_without_reporting():
  status = PedalCruiseStatus()
  _update(status)
  status.reset()

  assert not status.engaged
  assert _update(status) == (True, False)
