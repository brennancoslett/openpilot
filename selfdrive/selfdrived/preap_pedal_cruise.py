"""Pre-AP pedal-cruise engage/disengage edges for the driver chimes.

carState.pedalLongActive answers whether the Comma Pedal firmware currently
holds longitudinal authority, which is not the same question as whether pedal
cruise is on. A driver accelerator override deliberately hands actuation back
to the foot and takes it again on release, so the raw flag toggles on every
crossing of the override threshold: feathering the pedal in traffic -- easing
off without fully lifting -- put an engage or a disengage chime on each
crossing.

An override is therefore held rather than reported. Every real way to lose
pedal cruise clears something else instead: the brake and a single stalk pull
drop the longitudinal FSM, cancel and a steering disengage clear cruise
entirely, and a pedal fault latches pedalAuthorityFailed. All of those still
report on the frame they happen.
"""


class PedalCruiseStatus:
  """Tracks whether pedal cruise is engaged, across driver overrides."""

  def __init__(self):
    self.engaged = False

  def reset(self):
    self.engaged = False

  def update(self, *, pedal_long_active: bool, cruise_enabled: bool,
             long_override: bool) -> tuple[bool, bool]:
    """Advance one frame. Returns (just_engaged, just_disengaged)."""
    engaged = bool(pedal_long_active and cruise_enabled)

    # Only an override that interrupts an established engagement is held. One
    # that is already running when cruise is requested is not: the car has not
    # taken longitudinal yet, and reporting an engagement before the first
    # frame of authority would announce something that has not happened.
    if not engaged and self.engaged and cruise_enabled and long_override:
      engaged = True

    just_engaged = engaged and not self.engaged
    just_disengaged = self.engaged and not engaged
    self.engaged = engaged
    return just_engaged, just_disengaged
