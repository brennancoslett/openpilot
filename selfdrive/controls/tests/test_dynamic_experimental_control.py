from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.dynamic_experimental_control import DynamicExperimentalController, TRAJECTORY_SIZE

T_HORIZON = 10.0  # s, the model's last trajectory point


class FakeParams:
  def __init__(self, enabled=True):
    self.enabled = enabled

  def __bool__(self):
    return False  # the planner's test stubs define __bool__ False; DEC must still use them

  def get_bool(self, key):
    assert key == "DynamicExperimentalControl"
    return self.enabled


def make_sm(*, v_ego, v_cruise, endpoint_x=None, standstill=False, experimental=True, lead=False):
  if endpoint_x is None:
    endpoint_x = v_ego * T_HORIZON
  xs = [endpoint_x * i / (TRAJECTORY_SIZE - 1) for i in range(TRAJECTORY_SIZE)]
  return {
    'carState': SimpleNamespace(vEgo=v_ego, vCruise=v_cruise * 3.6, standstill=standstill),
    'radarState': SimpleNamespace(leadOne=SimpleNamespace(status=lead)),
    'modelV2': SimpleNamespace(position=SimpleNamespace(x=xs), orientation=SimpleNamespace(x=[0.0] * TRAJECTORY_SIZE)),
    'selfdriveState': SimpleNamespace(experimentalMode=experimental),
  }


def make_dec(radar_unavailable=True, enabled=True):
  CP = SimpleNamespace(radarUnavailable=radar_unavailable)
  mpc = SimpleNamespace(crash_cnt=0)
  return DynamicExperimentalController(CP, mpc, params=FakeParams(enabled)), mpc


def run(dec, sm, frames):
  for _ in range(frames):
    dec.update(sm)


def test_starts_in_acc():
  dec, _ = make_dec()
  assert dec.mode() == 'acc'


def test_cruising_below_set_speed_stays_in_acc():
  # Drive 00000117: 43 mph against a 47 mph set speed, lead pulling away. The
  # model asked for ~0 m/s^2 and the MPC for +0.3; ACC is what lets it go.
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=19.2, v_cruise=21.0), 200)
  assert dec.active()
  assert dec.mode() == 'acc'


def test_predicted_stop_hands_the_plan_to_the_model():
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0), 40)
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0, endpoint_x=20.0), 20)
  assert dec.mode() == 'blended'


def test_mild_slowdown_needs_hysteresis_before_switching():
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0), 40)
  # 54 kph wants ~142 m; 105 m is a shortfall but far from a stop
  sm = make_sm(v_ego=15.0, v_cruise=21.0, endpoint_x=105.0)
  dec.update(sm)
  assert dec.mode() == 'acc'
  run(dec, sm, 100)
  assert dec.mode() == 'blended'


def test_returns_to_acc_once_the_slowdown_clears():
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0, endpoint_x=20.0), 40)
  assert dec.mode() == 'blended'
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0), 200)
  assert dec.mode() == 'acc'


def test_standstill_is_blended():
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=0.0, v_cruise=21.0, endpoint_x=40.0, standstill=True), 40)
  assert dec.mode() == 'blended'


def test_mpc_fcw_is_blended_immediately():
  dec, mpc = make_dec()
  sm = make_sm(v_ego=20.0, v_cruise=25.0)
  run(dec, sm, 40)
  mpc.crash_cnt = 1
  run(dec, sm, 3)
  assert dec.mode() == 'blended'


def test_radar_lead_keeps_acc_on_a_mild_slowdown():
  dec, _ = make_dec(radar_unavailable=False)
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0, endpoint_x=105.0, lead=True), 200)
  assert dec.mode() == 'acc'


def test_inactive_without_experimental_mode_or_the_toggle():
  dec, _ = make_dec()
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0, experimental=False), 5)
  assert not dec.active()

  dec, _ = make_dec(enabled=False)
  run(dec, make_sm(v_ego=15.0, v_cruise=21.0), 5)
  assert not dec.active()
