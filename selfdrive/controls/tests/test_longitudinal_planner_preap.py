import numpy as np
import pytest

from cereal import car, log, messaging
from opendbc.car.interfaces import ACCEL_MIN
from opendbc.car.tesla.preap.nap_conf import REGEN_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants

BRAKING_ONSET_ACCEL = -0.3  # m/s²; past coasting, the driver feels this


def make_preap_params():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  return params


def make_planner_inputs(*, v_ego, v_cruise, pitch, throttle_probability, lead=None):
  radar = messaging.new_message("radarState").radarState
  if lead is not None:
    d_rel, v_lead = lead
    radar.leadOne.status = True
    radar.leadOne.dRel = d_rel
    radar.leadOne.vLead = v_lead
    radar.leadOne.vLeadK = v_lead
    radar.leadOne.aLeadK = 0.0
    radar.leadOne.aLeadTau = 1.5
    radar.leadOne.modelProb = 1.0
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.pid
  car_state.vEgo = v_ego
  car_state.vCruise = v_cruise * 3.6
  car_control.orientationNED = [0.0, pitch, 0.0]

  position = log.XYZTData.new_message()
  position.x = ((v_ego + 0.5) * np.array(ModelConstants.T_IDXS)).tolist()
  model.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = ((v_ego + 0.5) * np.ones_like(ModelConstants.T_IDXS)).tolist()
  velocity.x[0] = v_ego
  model.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.acceleration = acceleration
  model.meta.disengagePredictions.gasPressProbs = [throttle_probability] * 6

  return {
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
  }


def test_preap_cruise_ignores_model_throttle_suppression():
  planner = LongitudinalPlanner(make_preap_params(), init_v=20.9)
  inputs = make_planner_inputs(
    v_ego=20.9,
    v_cruise=21.0,
    pitch=0.046,
    throttle_probability=0.15,
  )

  for _ in range(60):
    planner.update(inputs)

  assert planner.output_a_target > -0.05
  assert planner.allow_throttle


@pytest.mark.parametrize(("brand", "fingerprint", "openpilot_longitudinal", "pcm_cruise"), [
  ("honda", "HONDA_CIVIC", True, False),
  ("tesla", "TESLA_MODEL_S_PREAP", False, True),
])
def test_non_vdas_modes_keep_model_throttle_suppression(
  brand, fingerprint, openpilot_longitudinal, pcm_cruise,
):
  params = make_preap_params()
  params.brand = brand
  params.carFingerprint = fingerprint
  params.openpilotLongitudinalControl = openpilot_longitudinal
  params.pcmCruise = pcm_cruise
  planner = LongitudinalPlanner(params, init_v=20.9)
  inputs = make_planner_inputs(
    v_ego=20.9,
    v_cruise=21.0,
    pitch=0.046,
    throttle_probability=0.15,
  )

  for _ in range(60):
    planner.update(inputs)

  assert not planner.allow_throttle
  assert planner.output_a_target < -0.5


def test_preap_planner_plans_inside_the_regen_envelope():
  # The pedal decelerates the car with regen alone, which VirtualDAS clips at
  # REGEN_MAX. The solver has to know that: planning against the generic
  # ACCEL_MIN allows it to defer braking past what regen can recover from.
  planner = LongitudinalPlanner(make_preap_params(), init_v=25.0)
  assert planner.plan_accel_min == pytest.approx(REGEN_MAX)

  inputs = make_planner_inputs(
    v_ego=25.0,
    v_cruise=30.0,
    pitch=0.0,
    throttle_probability=1.0,
    lead=(25.0, 0.0),
  )
  for _ in range(20):
    planner.update(inputs)

  assert np.all(planner.mpc.params[:, 0] == pytest.approx(REGEN_MAX))
  assert planner.output_a_target >= REGEN_MAX - 1e-6

  # The gap is sized from the same deceleration the plan is allowed to ask for,
  # which is what makes the approach start early enough to stay gentle.
  assert planner.plan_comfort_brake == pytest.approx(-REGEN_MAX)
  assert np.all(planner.mpc.params[:, 6] == pytest.approx(-REGEN_MAX))
  assert -REGEN_MAX < COMFORT_BRAKE


def test_non_preap_planner_keeps_the_generic_accel_floor():
  params = make_preap_params()
  params.brand = "honda"
  params.carFingerprint = "HONDA_CIVIC"
  planner = LongitudinalPlanner(params, init_v=25.0)
  assert planner.plan_accel_min == pytest.approx(ACCEL_MIN)

  inputs = make_planner_inputs(
    v_ego=25.0,
    v_cruise=30.0,
    pitch=0.0,
    throttle_probability=1.0,
    lead=(25.0, 0.0),
  )
  planner.update(inputs)

  assert np.all(planner.mpc.params[:, 0] == pytest.approx(ACCEL_MIN))
  assert planner.plan_comfort_brake == pytest.approx(COMFORT_BRAKE)
  assert np.all(planner.mpc.params[:, 6] == pytest.approx(COMFORT_BRAKE))


def _gap_at_braking_onset(params, *, v_ego, v_lead, d_rel, comfort_brake=None,
                          dt=0.05, max_steps=1200):
  """Follow an approach to a slower lead until the plan asks for real braking."""
  planner = LongitudinalPlanner(params, init_v=v_ego)
  if comfort_brake is not None:
    planner.plan_comfort_brake = comfort_brake
  v_cruise = v_ego
  for _ in range(max_steps):
    planner.update(make_planner_inputs(
      v_ego=v_ego,
      v_cruise=v_cruise,
      pitch=0.0,
      throttle_probability=1.0,
      lead=(d_rel, v_lead),
    ))
    a_target = float(planner.output_a_target)
    if a_target <= BRAKING_ONSET_ACCEL:
      return d_rel
    v_ego = max(v_ego + a_target * dt, 0.0)
    d_rel = max(d_rel + (v_lead - v_ego) * dt, 0.0)
  return 0.0


def test_comfort_brake_sets_how_early_an_approach_starts_braking():
  # Which knob moves the onset. The a_min constraint only binds once the plan
  # is already braking hard, so on its own it barely moves the point where
  # braking starts -- the comfort brake does, because it sizes the room the
  # cost wants for shedding the closing speed. Measured on the same approach,
  # so a change to either one shows up here as a distance rather than a guess.
  approach = {"v_ego": 25.0, "v_lead": 12.0, "d_rel": 200.0}
  gentle = _gap_at_braking_onset(make_preap_params(), comfort_brake=1.5, **approach)
  generic = _gap_at_braking_onset(make_preap_params(), comfort_brake=COMFORT_BRAKE, **approach)

  assert generic > 0.0
  assert gentle >= generic + 10.0
