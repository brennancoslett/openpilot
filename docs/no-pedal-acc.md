# No-pedal ACC on `no-pedal-acc`

What this branch adds on top of `nap-release`, how the stalk-spam cruise
controller works, and how it differs from the Tinkla `tesla_unity` ACC module it
descends from.

Branch pair:

| Repo | Branch | Base | Commits |
|---|---|---|---|
| openpilot | `no-pedal-acc` | `nap-release` | 11 |
| opendbc | `no-pedal-acc` | `cc39c7fa` | 12 |

Since 2026-08-04 that pair is frozen at its road-validated tips for other
users; ongoing tuning happens on the `no-pedal-acc-dev` pair (both repos),
which is what this document tracks.

---

## 1. The feature

A pre-AP Model S with no Comma Pedal has no throttle actuator. The only speed
authority openpilot has is the stock cruise control's set speed, and the only way
to move that is to spoof cruise-stalk presses on `STW_ACTN_RQ` (0x45). So
no-pedal ACC runs the normal openpilot longitudinal planner and translates its
accel request into a stream of 1 mph / 5 mph stalk presses that walk the DI's set
speed toward the planned speed.

Decel authority is whatever the DI does when its set speed drops — motor regen
only, ~-1.5 m/s² at best, nothing below ~18 mph, no friction brakes. It cannot
stop for a stopped car. Large decel demands drop cruise entirely (coasting regen
beats the DI's shallow slew) and hand braking to the driver.

### Pieces

```
model → planner → controlsd ──► CC.actuators.accel, CC.hudControl.setSpeed
                                          │
opendbc/car/tesla/carcontroller.py        ▼
  └─ NoPedalACCController.update()   preap/no_pedal_acc.py   (button decision)
        │ non-CANCEL                          │ CANCEL
        ▼                                     ▼
  StockCCSpoofer.request_button()      CS.preap_cc_cancel_needed
        └────────────► 0x45 STW_ACTN_RQ ◄─────┘
                            │  (panda: tesla_preap.h value + rate gate)
                            ▼
                     DI (stock CC) ── DI_state.DI_cruiseSet ──► preap/carstate.py
```

Ownership is split, unlike the module it was ported from:

- `preap/engagement.py` (`PreAPEngagement`) owns engagement and the driver's
  **ceiling** (`pedal_speed_kph`, shared with pedal mode). Double-pull engages,
  driver stalk presses move the ceiling, brake drops longitudinal.
- `preap/no_pedal_acc.py` (`NoPedalACCController`) is decision-only: one button
  or `None` per 100 Hz frame. It never touches CAN.
- `preap/stock_cc_spoofer.py` (`StockCCSpoofer`) owns every 0x45 TX, one slot per
  `frame % 10 == 0`, with a fixed priority: cancel > engage/renorm > ACC speed
  press. A queued speed press is dropped, not deferred, if a cancel or engage is
  in flight.
- `opendbc/safety/modes/tesla_preap.h` is the independent backstop.

### Mode selection (`preap/interface.py`)

`no_pedal_acc = nap_conf.no_pedal_acc and not use_pedal` — a fitted pedal always
wins. When set: `openpilotLongitudinalControl = True`, `pcmCruise = False`,
safety flag `PREAP_FLAG_NO_PEDAL_ACC = 8`, `longitudinalActuatorDelay = 0.8`, and
the stock zero-gain longitudinal tuning, so `LongControl` passes `a_target`
through as pure feedforward — which is exactly what the controller consumes.
Read at fingerprint time, so the toggle needs a reboot.

### Control law

Per frame, in order:

1. **Gates.** Requires FSM long mode + `CC.longActive` + `not gasPressed` +
   `di_cruise_state == "ENABLED"`. Engaging the stock CC is the spoofer's job;
   after a CANCEL the driver must double-pull again (no autoresume).
2. **Hard-brake CANCEL** — `aReq < -1.3` returns immediately, bypassing every
   holdoff and the offset math (which self-corrects too fast to ever reach its
   own CANCEL rung).
3. **Unmet-decel CANCEL** — `aReq < -0.3` *and* `aReq - aEgo < -0.3`, sustained
   0.7 s. "The planner wants to slow and the car isn't slowing", i.e. the steps
   are failing. Disarmed for 2500 ms after any driver stalk speed action, because
   lowering the ceiling drops the planner's target instantly and produces the
   same signature before any down-press has been taken.
4. **Holdoffs** — 500 ms since the last human stalk action, 600 ms since the last
   automated press.
5. **Target** — `desired_kph = vEgo + aReq × 1.5 s`, capped at the ceiling.
6. **Ceiling floor**, only when `aReq >= 0.0`:
   - projection ≥ current set → floor at `cc_set + half_step + 0.05`, so the set
     speed keeps climbing to the ceiling instead of stalling under it once
     `aReq → 0`. Floor-driven climbs are rate-limited to
     `half_kph / (3.6 × aReq)` ms, clamped to [600, 10000].
   - projection < current set → **hold at `cc_set`**, no press. That is the
     normal "still catching up" state and must not read as overspeed. One
     escape (2026-08-04): an unconditional hold deadlocks one step under MAX —
     the DI's bang-bang dead zone parks `vEgo` ~1.5 kph under the set, from
     where the projection needs aReq ≥ ~+0.28 to reach the set while the
     planner asks a median +0.16 near its target (922 stalled frames on drive
     `0000004f`, longest run 66 s). When `CC.planSpeedTarget − vEgo ≥ 1.5 kph`
     (the planner's own 2.5 s terminal speed wants meaningfully more — false
     when settled behind a lead, measured median −1.2 kph there) *and*
     `cc_set − vEgo < 2.2 kph` (DI idle in its dead zone, so one press yields
     one bang-bang step; excludes the catching-up state), take one
     rate-limited floor climb instead of holding.
7. **`calc_button`** — the decision table, on `offset = desired - cc_set`:

   | condition | button |
   |---|---|
   | `desired < 17.1 mph` | CANCEL |
   | `offset < -2 × full` | CANCEL |
   | `offset < -0.6 × full` | DECEL_2ND |
   | `offset < -0.9 × half` | DECEL_SET |
   | `offset ≥ full`, headroom under ceiling | RES_ACCEL_2ND |
   | `offset ≥ half`, headroom under ceiling | RES_ACCEL |

   Plus the SCCM crash guard: any decel press that would step below the min
   cruise speed becomes CANCEL instead.

In practice only `UP_1ST` / `DN_1ST` / `CANCEL` fire — both 2ND rungs need
offsets the projection almost never produces. Making them reachable was tried and
reverted (tracked speeds got worse).

### Instrumentation

Every frame emits a `NoPedalACC.tlm` line at 5 Hz while modulating, 1 Hz
otherwise, carrying `aReq`/`aEgo`/`ccSet`/`desired`/`ceil`/`off`/`btn`/`reason`
plus the projection constant so runs at different gains are comparable.
Button changes additionally log a change-triggered `NoPedalACC press` line —
telemetry is decimated and misses one-frame presses, so press counts must come
from the latter.

---

## 2. Compared to `tesla_unity`

Source of the original: **`origin/tesla-unity`** (NotAutopilot/openpilot),
`selfdrive/car/tesla/ACC_module.py` (`ACCController`), driven from
`LONG_module.py`. openpilot 0.9.x, C2/C3 only.

`ACC_module.py` there is byte-identical to the older Tinkla
`boggyver/openpilot@tesla_unity_dev` copy apart from the `openpilot.`-prefixed
import namespace, so the two are interchangeable for this comparison.
`LONG_module.py` differs only in imports, added Model X fingerprints, reshaped
AP long-control messages, `FleetSpeed` pruned from the module (still used inside
`ACCController`), and `pcm_speed` dropped from the `update()` signature. The ACC
path is unchanged: `v_target = long_plan.longitudinalPlan.speeds[-1]`, called at
`frame % 20`, TX via `messages.insert(0, create_action_request(...))`.

### Structural

| | tesla_unity `ACCController` | this branch |
|---|---|---|
| Scope | monolith: engagement FSM, ceiling, decision, event emission | decision only; FSM/ceiling/TX/events live in separate modules |
| Engagement | own double-pull (750 ms), own `enable_adaptive_cruise`, own `accEnabled`/`ccDisabled` events | `PreAPEngagement` (400 ms window) already existed for stock CC and pedal mode; unchanged path |
| Ceiling | own `acc_speed_kph`, seeded from `vEgoRaw`, stepped by mapping driver presses, clipped 0–170 | `engagement.pedal_speed_kph`, the same driver-owned target pedal mode uses |
| Cadence | 5 Hz (`frame % 20`) | 100 Hz, gated by `AUTO_ACTION_SPACING_MS` |
| TX | LONG_module hand-builds the frame and `insert(0, …)`s it to race the real stalk | `StockCCSpoofer` owns the slot, priority, and counter |
| Panda safety | address whitelist only (`{0x45, 0, 8}`, `{0x45, 2, 8}` in `safety_tesla.h`) — any payload, any rate | value whitelist, 2ND-press flag gate, 300 ms rate floor |
| Radar | subscribes `radarState` inside the car module for lead / TTC logic | no radar dependency; leads are the planner's problem |
| Speed limits | `set_speed_limit_active`, offsets, `FleetSpeed` averager fold into the ceiling | not ported |
| Non-adaptive "just CC" mode | yes (`enableJustCC`) | not ported |

### What the controller is actually fed

This is the biggest divergence. tesla_unity fed `_calc_button` a **speed**:
`self.v_target = long_plan.longitudinalPlan.speeds[-1]`. This branch feeds it an
**accel projected into a speed**: `vEgo + CC.actuators.accel × 1.5 s`.

The horizon is *not* the difference — `longitudinalPlan.speeds` covers 2.5 s on
both codebases (`log.capnp:1261`, identical comment on `origin/tesla-unity`).
The difference is **absolute vs relative**: `speeds[-1]` is the MPC's own target,
independent of measured speed; the projection is anchored on `vEgo`, i.e. on the
plant's output.

Consequences, and they are the root of most of the extra machinery here:

- **Stall.** The projection collapses to `vEgo` as `aReq → 0`, so at equilibrium
  the target is "current speed" and the set speed parks under the driver's
  ceiling. Hence the **ceiling floor** (§1.6), which has no unity analogue.
  (`speeds[-1]` would NOT have fixed this, despite converging toward `v_cruise`
  with no lead — measured on drive `0000004f`: from a standing 3–5 kph deficit
  the MPC's 2.5 s terminal speed sat a median 0.6 kph *below* the DI set,
  above it only 7.2% of frames. Anything anchored near `vEgo` deadlocks
  against the DI's bang-bang dead zone; see §1.6's hold-branch escape.)
- **Ratchet / hunting.** The DI is itself a closed-loop speed controller, so
  putting `vEgo` inside our target for it closes a positive feedback path:
  press UP → DI accelerates → `vEgo` rises → target rises → press UP. That is
  drive `0000002e` at 18:14:52 (set 61.2 → 72.4 kph in 3.6 s) and the 18:44:52
  limit cycle. `speeds[-1]` has no such path. Hence the floor's **rate limit**
  and the **hold-at-set** direction gate.

Stall and ratchet are the same defect seen from two sides. See §4 for the
assessment of whether this was the right call.

### Hand-off / safety behaviour

tesla_unity's only escape hatch was the `offset < -2 × full` CANCEL rung inside
the decision table, and a lead-based `_fast_decel_required` used solely to *block
auto re-engagement*. This branch adds two triggers that fire outside the table
entirely, because the table's own rung self-corrects before it can trip (each
down-press lowers `cc_set`, closing the offset):

- **hard-brake CANCEL** at `aReq < -1.3`, gate-bypassing;
- **unmet-decel CANCEL**, gap-based (`aReq - aEgo`) rather than a fixed decel
  level, sustained 0.7 s, with a 2500 ms disarm after driver stalk input.

The hand-off also raises a one-shot `noPedalAccBrakeHandoff` event
(`brake_handoff_edge`) for a distinct "Brake / ACC can't slow" chime — unity had
no notion of telling the driver it had given up.

**No autoresume.** unity's `_should_autoengage_cc` (+ `TinklaAutoResumeACC`
param) would re-RESUME the stock CC from STANDBY once conditions looked safe.

Not ported. Honest history: this was **deferred, not decided** — the original
plan listed it as a phase-3 "optional auto-reengage, default off" item, phase 3
never happened, and no commit in the series mentions it. The only record is the
parenthetical at `no_pedal_acc.py:244`.

Reasons it stays out, retrospective but sound:

- **Dependencies absent.** `_should_autoengage_cc` needs `CS.HSO.human_control`
  (no Human Steering Override module here), `user_has_braked` /
  `has_gone_below_min_speed` / `fast_decel_time` (state internal to unity's own
  ACCController, replaced by `PreAPEngagement`), and radar leads via
  `_fast_decel_required` (no radar subscription in the car module, and radar is
  usually off on this setup). Porting means rewriting, not copying.
- **Architecture.** `no_pedal_acc.py` is decision-only; engaging belongs to
  `StockCCSpoofer` driven by FSM intent flags.
- **Safety.** CANCEL here *is* the brake hand-off. Re-arming speed hold seconds
  after telling the driver "you are the brakes" is the wrong reflex — and
  unity's guard against exactly that is the radar-dependent part we cannot
  reproduce, so the port would be the resume logic without its interlock.

**Cost, and it is unresolved.** The same triggers exist here (below min cruise →
CANCEL, brake → drops longitudinal) plus two hand-off CANCELs that fire on
ordinary decel events, so re-double-pulling is frequent. And an ACC-driven CANCEL
does not clear the FSM: `check_can_engage` (`engagement.py:123`) clears
`cruiseEnabled`/`enableLongControl` only on doors, gear, or seatbelt. After a
hand-off where the driver doesn't brake, `cruiseState.enabled` stays true,
openpilot reports engaged, lateral keeps running, and `_decide()` returns `None`
every frame at `reason=di_standby` — inert until a double-pull. unity at least
cleared `enable_adaptive_cruise` and emitted `accDisabled`. The driver's only cue
that longitudinal is dead is the one-shot brake chime, which is opt-in and
defaults off.

### Timing

| | unity | here |
|---|---|---|
| human holdoff | 3000 ms | 500 ms |
| automated spacing | 400 ms | 600 ms |
| "steps have failed" settle | — | 2500 ms |

unity's single 3000 ms constant was doing two jobs: deferring *presses* to the
driver, and deferring the *conclusion that stepping has failed*. Splitting them
is what let the press holdoff drop to 500 ms (bounded below by the 400 ms
double-pull window, the 300 ms spoof-echo window, and staying under the 600 ms
spacing) while the failure inference still outlasts the DI's response to a
set-speed step.

600 ms rather than 400 ms because each 1 mph step makes the DI surge to ~+1.0
m/s² and decay over ~500 ms; at 400 ms presses re-surged before the last settled.

### Smaller divergences

- **Echo suppression.** Spoofed presses come back on the RX bus. `StockCCSpoofer`
  stamps `engagement.preap_last_speed_spoof_ms` on TX and
  `_is_speed_spoof_echo()` demotes a matching button event to `unknown` within
  300 ms, so the controller can't self-holdoff or ratchet its own ceiling. unity
  fed RX buttons straight into `should_be_throttled` → `human_cruise_action_time`.
- **SCCM guard.** unity: `v_cruise_actual - 1 < MIN_CRUISE`, a hardcoded 1 kph.
  Here: `cc_set_kph - half_kph < min_cruise_kph`, i.e. step-aware and correct in
  both unit systems.
- **Min-speed CANCEL.** In unity `_calc_button`, the `desired < MIN_CRUISE` CANCEL
  is a bare `if` followed by a separate `if/elif` chain, so a below-minimum target
  with a small offset **overwrites** the CANCEL with `DECEL_SET` or with nothing.
  Here it is an early `return`.
- **Set-speed read-back — unity was reading the wrong field on pre-AP.**
  `tesla_can_pre1916.dbc` on that branch declares
  `SG_ DI_cruiseSet : 32|9@1+ (0.5,0)`, and `carstate.py:285` reads
  `self.v_cruise_actual = cp.vl["DI_state"]["DI_cruiseSet"]`
  (`ret.cruiseState.speed` at 310/312 too). On pre-AP firmware bits 32–40 are the
  *displayed vehicle speed* at scale 1, so `v_cruise_actual` came back as vehicle
  speed at half scale — a permanently large positive `speed_offset_kph`, i.e.
  `RES_ACCEL_2ND` spam. The entire offset math ran on a garbage set speed. Same
  bug this branch hit and fixed in `508e20c5`: bits 48–55 scale 1, corrected in
  both `tesla_preap.dbc` and `preap/carstate.py` — see §3.
- **Stale resume.** A physical stalk pull on the pre-AP DI is a RESUME of the
  *stored* set speed, so a double-pull after an earlier faster cruise leaves the
  DI chasing the old number. unity had no handling. `StockCCSpoofer` gained a
  `_PHASE_RENORM`: on finding the DI ENABLED with the set >1 full step from vEgo,
  CANCEL, wait for observed STANDBY (don't trust ENABLED through CAN latency),
  then `DECEL_SET` to set at current speed. Two rounds, 1.5 s timeout, fails safe.
- **Telemetry.** unity's decision logging is a commented-out `print`.

---

## 3. Everything else on this branch, not in `nap-release`

### Green-light and lead-departure chimes (`a4f01dd85`)

Port of sunnypilot's `e2e_alerts_helper.py` (MIT) into
`selfdrive/selfdrived/green_light.py`, driven from `selfdrived.py`. Two new
`ET.PERMANENT` events (`greenLightChime`, `leadDepartChime` — permanent because
they fire while disengaged or lateral-only), two new params
(`GreenLightAlert`, `LeadDepartAlert`) with mici toggles. Independent of
no-pedal ACC.

Retuned for this fork's model, which emits a near-null trajectory at a dead stop
(`position.x[-1]` p99 = 8.4 m, so sunnypilot's >30 m test never fires):
green light triggers on `modelV2.velocity.x[0] > 0.5` with the path endpoint kept
as an OR fallback, and the sustain window is 0.05 s because the pre-roll "wants to
go" spike lasts ~90 ms. Lead source falls back to `modelV2.leadsV3` when radar is
absent, with a 1 s presence hold. Rationale in `docs/green-light-chime-port.md`.

### Brake-handoff chime (`6096a8fdf`, `49f73152e`, opendbc `9d587a51`)

`noPedalAccBrakeHandoff` event → `AudibleAlert.warningSoft`, `Priority.HIGH` so it
masks the `teslaCCDisengaged` that follows ~200 ms later. Plumbed as
`CarState.noPedalAccBrakeHandoff` (capnp @66) and raised in
`selfdrive/car/car_specific.py`. Opt-in via `NAPNoPedalACCBrakeChime`, default
off, no reboot needed — the param read sits inside the one-shot edge, so it costs
nothing per frame. Road-confirmed.

### Pinned MAX box + relocated DMoji (`cbf5bf7fb`, `49f73152e`)

`NAPAlwaysShowMaxSpeed` holds the mici HUD's set-speed box up for the whole drive
instead of fading it a moment after the set speed settles, and moves the
driver-monitoring icon to the bottom-right so the two don't share the top-left
corner. Exists because the stock cluster's own readout blanks between spoofed
presses, leaving this as the driver's only persistent view of the target.
**mici only** — the tici HUD draws MAX unconditionally, so a tici toggle would be
dead. Read through `ui_state.always_show_max_speed` (params re-read every 5 s).

### Independent pre-AP fixes bundled in the opendbc series

These are not part of the feature and would stand alone upstream:

- **`DI_state` field swap** (`508e20c5`). Pre-AP firmware puts the displayed
  vehicle speed at bits 32–40 (scale 1) and the cruise set speed at bits 48–55
  (scale 1), the opposite of the AP-era DBC. Re-confirmed from raw 0x368 on a
  2026-07-26 drive: bits 32–40 track vEgo 1:1 in mph; bits 48–55 sit flat while
  the car swings 31–37 mph. `tesla_preap.dbc` signal names now match.
- **Driver-override detection during stock CC** (`preap/carstate.py`).
  `DI_pedalPos` is CC-authored while the DI holds cruise — 100% of 31,580 ENABLED
  frames on a deliberate foot-off drive read "pressed" by a raw threshold — and
  this car never reports the DI's `OVERRIDE` cruise state. So `gasPressed` during
  ENABLED is now `pedal_threshold and vEgo > set + 2 mph and not
  set_change_recent`, with a 3 s grace after any `DI_cruiseSet` change to
  suppress the legitimate lag after a stalk step or a spoof. Validated against
  known ground truth (3 real overrides detected on one drive, 0 on the foot-off
  drive). Outside ENABLED, the raw threshold still applies.
- **Cancel latch bug** (`stock_cc_spoofer.py`). `cancel_frame` was re-stamped
  every frame while `preap_cc_cancel_needed` was asserted, so
  `frame - cancel_frame` stayed 0 and `cancel_ready` never became true. The old
  one-shot FSM cancel never hit this; a sustained ACC cancel did, and commanded
  CANCEL for ~1.8 s with the DI still ENABLED. Now latched on the rising edge.
- **`pedalNotCalibrated` gating** (`car_specific.py`). Was raised for any
  `pcmCruise=False` pre-AP car; no-pedal ACC also runs `pcmCruise=False` with no
  pedal, so it is now gated on `nap_conf.use_pedal`.
- **Reboot-modal guard** (`selfdrive/ui/layouts/settings/nap.py`). Reboot prompts
  are now additionally guarded on `ui_state.is_offroad()`.

### Panda-side safety for 0x45 (`64d27060`)

`tesla_preap.h` previously allowed any `STW_ACTN_RQ` payload. Now:
`SpdCtrlLvr_Stat` is value-whitelisted (0/1/2/4/8/16/32); `RES_ACCEL_2ND` and
`DECEL_2ND` are hard-blocked unless `PREAP_FLAG_NO_PEDAL_ACC` is set, since the
base engage/cancel/renorm FSM never sends them; and all four speed-adjust values
are rate-limited to one per 300 ms while the flag is set. CANCEL and MAIN are
exempt — a cancel must never be delayed.

### Tests

`preap/tests/test_no_pedal_acc.py` (new, ~550 lines), plus additions to
`test_preap_carstate.py`, `test_stock_cc_spoofer.py`,
`safety/tests/test_tesla_preap.py`, and openpilot's
`selfdrive/car/tests/test_car_specific_tesla_preap.py`. 150 pre-AP tests pass on
device as of the current tip.

---

## 4. Assessment: which divergences were improvements

### Input signal — likely a regression

The accel projection is probably the wrong choice, and the three floor
mechanisms are compensation for it rather than features in their own right.

Against it:

- Both the stall and the ratchet trace to one property, `vEgo` being inside the
  target. `speeds[-1]` is anchored to the MPC's solution and has neither failure
  mode.
- Horizon buys nothing: both signals are ~1.5–2.5 s. The projection is not
  "more responsive" in exchange for the instability — it is the same time scale
  with a feedback path attached.
- The compensation is not settled. By the tuning record: the floor rate limit is
  inert in closed-loop simulation and is kept as a derived guard, not a
  demonstrated fix; and hold-at-set's predicted stall (set speed converging
  ~1.6 kph above `vEgo`, never reaching the ceiling) **happened** — drive
  `0000004f`, 2026-08-04, 922 stalled frames — and needed a fourth mechanism,
  the plan-speed escape (§1.6). Four interacting mechanisms with
  drive-specific constants, patching a signal choice.

For it, and these are real:

- `CC.actuators.accel` is already in `CarControl`. `longitudinalPlan` is not, and
  opendbc is now a standalone library that cannot subscribe to cereal — unity
  could only do it because `LONG_module.py` lived inside `selfdrive/car/` and
  called `messaging.sub_sock` directly. The choice was architecturally
  constrained, not lazy.
- `aReq` is post-`LongControl`, so it already respects `get_pid_accel_limits`
  and NAP's adaptive accel limits. Raw `speeds[-1]` would bypass both.

The plumbing now exists (2026-08-04): `CarControl.planSpeedTarget @18` carries
`longitudinalPlan.speeds[-1]` from controlsd, added for the hold-branch escape
(§1.6). A full signal swap would be a controller change only from here — but
note the measured caveat above: near equilibrium `speeds[-1]` sits *below* the
DI set speed, so the swap alone would not have fixed the stall either, and the
projection-vs-plan question is now about the transient behavior, not the
steady state.

Not a free win, and it fixes less than it looks like:

- It would not fix hand-off latency. The planner's demand going -0.18 → -1.37 in
  about a second is upstream of the signal choice.
- It could be worse behind a lead. `speeds[-1]` during a closing approach can sit
  well below `vEgo`, producing large negative offsets that reach the `DECEL_2ND`
  and CANCEL rungs more often — which is the "cuts CC far too soon" behaviour
  that already got the headway trigger rejected on the road.

**Do not build or deploy this without agreement.** It is a hypothesis about the
ratchet and the stall, not a tuning change.

### Hand-off — an improvement

Better on the merits, not just bigger:

- unity watched `offset`, a **set-speed error**. On a car whose decel authority
  is unknown and weak, set-speed error says nothing about whether the car is
  actually slowing. `aReq - aEgo` measures demand-not-met, which is the correct
  observable for an actuator of uncertain authority.
- unity's rung also races its own output: each `DECEL_SET` lowers `cc_set`, which
  shrinks `|offset|`, so it fights the very condition it is waiting for.
- The chime is unambiguously better. A car that cannot stop, silently giving up,
  is the worst available failure mode; unity had no notification at all.

Better is not sufficient, and the record says so: hysteresis on the arm bought
about 0.2 s, `hard_brake_cancel` still outfires `sustained_decel_cancel` 2:1, and
the driver-set-speed false positive needed the 2500 ms settle patch. The trigger
design is right and still runs into the same upstream wall.

## 5. Known rough edges

- The `tesla_preap.h` comment above `PREAP_SPEED_BUTTON_TX_MIN_INTERVAL_US` still
  cites `AUTO_ACTION_SPACING_MS=400ms`; python is 600. The 300 ms floor is still
  correctly below it, so only the comment is stale.
- `PREAP_FLAG_NO_PEDAL_ACC = 8` in `preap/interface.py` sits between two import
  statements. Harmless, but worth moving before an upstream PR.
- The floor climb rate limit is a derived guard, not a demonstrated fix — it is
  inert in closed-loop simulation against the measured DI response, because
  hold-at-set already stops the floor being re-entered at low demand.
- The predicted hold-at-set stall (set speed converging ~1.6 kph above vEgo and
  never reaching the ceiling) was confirmed on the road 2026-08-04 (drive
  `0000004f`: 922 frames parked 1–2 mph under MAX, longest run 66 s) and fixed
  by the plan-speed escape in §1.6. The fix is replay-validated against that
  drive but not yet road-confirmed.
- The feature is **not** declared road-ready and defaults off.
