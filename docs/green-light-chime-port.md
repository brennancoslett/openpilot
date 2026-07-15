# Porting sunnypilot's Green Light Chime

Plan for porting the "Green Traffic Light Alert" (and optional "Lead Departure
Alert") from upstream sunnypilot to this fork.

## How sunnypilot does it

Source: `sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py` (MIT license).
There is no camera-based traffic light classifier. It is an end-to-end proxy:

- **Arm** when the car is stopped: `carState.standstill`, no gas pressed,
  openpilot **not engaged** (`carControl.enabled == False`), not moving within
  the last 2 s, and no lead (`radarState.leadOne.status == False`).
- **Trigger** when the model's planned path endpoint
  (`modelV2.position.x[-1]`) exceeds 30 m continuously for 0.3 s — i.e. the
  driving model "wants to go", which at a stop almost always means the light
  turned green.
- A small INACTIVE → ARMED → CONSUMED state machine fires the chime once per
  stop.
- Sibling feature in the same file: **lead departure alert** — while stopped
  behind a lead closer than 8 m, chime when the lead pulls away by more than
  1 m.

## Why it ports cleanly to this fork

- `selfdrive/selfdrived/selfdrived.py` already subscribes to `modelV2`,
  `radarState`, `carState`, and `carControl` (see the SubMaster at
  `selfdrived.py:83`) — every input the helper needs.
- Chime sound already exists: `AudibleAlert.prompt` / `prompt.wav`, wired in
  `selfdrive/ui/soundd.py`.
- This fork already has custom events (e.g. `pedalCruiseEnabled @99` in
  `cereal/log.capnp`, alert defined in `selfdrive/selfdrived/events.py:1023`)
  — the exact pattern to copy.
- `ET.PERMANENT` alerts are active even while disengaged
  (`selfdrive/selfdrived/state.py:13`), which is required since the chime
  fires while openpilot is not engaged.

## Implementation steps

1. **Helper** — new file `selfdrive/selfdrived/green_light.py`: port
   `E2EAlertsHelper` from sunnypilot. Strip the sunnypilot scaffolding
   (`PARAMS_UPDATE_PERIOD` import, `EventsSP`); return plain booleans
   `(green_light_alert, lead_depart_alert)` from `update(CS, sm)`.
   Keep the constants: `GREEN_LIGHT_X_THRESHOLD = 30`,
   `LEAD_DEPART_DIST_THRESHOLD = 1.0`, `TRIGGER_TIMER_THRESHOLD = 0.3`.
   One adaptation: sunnypilot runs this at the 20 Hz planner rate (`DT_MDL`);
   selfdrived loops at 100 Hz, so all frame timers use `DT_CTRL` instead.
   Sunnypilot quirk kept intentionally: the alert never arms until the car
   has moved at least once after boot (`last_moving_frame == -1` counts as
   recently-moving), so no chime on the first stop after startup.

2. **Event enum** — `cereal/log.capnp`: add `greenLightChime` (and optionally
   `leadDepartChime`) to `OnroadEvent.EventName`, next ordinal after the
   fork's existing custom entries (after `@99 pedalCruiseEnabled` block).

3. **Alert definition** — `selfdrive/selfdrived/events.py`: add entry modeled
   on `pedalCruiseEnabled`, but as `ET.PERMANENT` (must display while
   disengaged):

   ```python
   EventName.greenLightChime: {
     ET.PERMANENT: Alert(
       "Green Light",
       "",
       AlertStatus.normal, AlertSize.small,
       Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.),
   },
   ```

4. **Hook** — `selfdrive/selfdrived/selfdrived.py`: instantiate the helper in
   `__init__`, call it in `update_events()` (near the pedal-cruise event block
   around line 200), and `self.events.add(EventName.greenLightChime)` when it
   fires.

5. **Param + toggle** — add `{"GreenLightAlert", {PERSISTENT, BOOL}}` (and
   optionally `LeadDepartAlert`) to `common/params_keys.h`; add a
   `BigParamControl("green light chime", "GreenLightAlert")` toggle in
   `selfdrive/ui/mici/layouts/settings/toggles.py` (or `nap.py` alongside the
   other NAP options).

## Fork-specific caveats

- Chimes only while **disengaged** — matches sitting at a light on the pre-AP
  vision-ACC setup. The `not CC.enabled` gate could be relaxed to
  "engaged but at standstill" later if desired.
- The no-lead gate uses `radarState.leadOne`; if the pre-AP Bosch radar lead
  is flaky at standstill, fall back to `modelV2.leadsV3`.
- E2e proxy means occasional false chimes (path opens for reasons other than
  a green light, e.g. cross traffic clearing).

## Verification

- Replay a drive with a red-to-green stop through the helper
  (`tools/replay` feeding `selfdrived`) or unit-test the state machine
  directly: feed synthetic `modelV2.position.x` endpoints (< 30 m stopped,
  then > 30 m) and assert single trigger per stop.
- On-device: stop at a light with the toggle on, disengaged — expect one
  `prompt.wav` chime + "Green Light" alert when the light changes; no repeat
  until the car moves and stops again.
