# oneloop_nB — settings guide

This guide covers every GCS setting of the `oneloop_nB` controller ([oneloop_nB.c](oneloop_nB.c)):

- what each setting does
- when it has an effect
- what its default is

It ends with two step-by-step guides for motor-fault flights: the **PlusQuadV3** with faulted pitch motors, and the **RW3C** with faulted pitch motors in quad and faulted roll motors in forward flight.

- Settings page: `conf/modules/oneloop_nB.xml` (GCS tab *oneloop*)
- Airframes using this controller: `PlusQuadV3` (`PlusQuad_v3.xml`) and `RW3C_nB` (`rotwing_v3c_nB_ail.xml`), both in `conf/userconf/tudelft/rotwing_conf.xml`

## How defaults work

Most tunables have a C default in `oneloop_nB.c`. Many of them can be overridden from the airframe, in the section `<section PREFIX="ONELOOP_NB_" NAME="ONELOOP_NB">`. For example, `<define name="DELTA_FAULT" value="700.0"/>` becomes `ONELOOP_NB_DELTA_FAULT`. The GCS setting then changes the live value in flight.

Rule for new parameters: every new tunable must have
1. a C default (`#ifndef ONELOOP_NB_<NAME>`),
2. a define in every airframe that uses the controller, and
3. a `dl_setting`.

In the tables below, "Define" is the airframe define name without the `ONELOOP_NB_` prefix. "—" means the setting can only be changed in flight.

## Flight modes and controller type

The fault logic only runs when the controller type is **nB** (`CTRL_NB_INDI` or `CTRL_NB_ANDI`). Which type is active depends on the mode (`conf/autopilot/rotorcraft_oneloop_nB.xml`):

| RC MODE switch | AP_MODE_SWITCH (AUX3) | Mode (GCS name) | Controller type | Loop | Faults active? |
|---|---|---|---|---|---|
| 0 (manual) | 0 | ATTITUDE_DIRECT (`ATT_ANDI`) | CTRL_ANDI | half (sticks) | no |
| 0 (manual) | 1 | RATE_RC_CLIMB (`nB_ANDI`) | CTRL_NB_ANDI | half (sticks) | **yes** |
| 0 (manual) | 2 | RATE_DIRECT (`nB INDI`) | CTRL_NB_INDI | half (sticks) | **yes** |
| 1 or 2 (auto) | 0 | MODULE (`ONE`) | CTRL_ANDI | full (nav) | no |
| 1 or 2 (auto) | 1 | NAV | CTRL_NB_ANDI | full (nav) | **yes** |
| 1 or 2 (auto) | 2 | FORWARD (`nB_NAV_INDI`) | CTRL_NB_INDI | full (nav) | **yes** |

What this means in practice:
- **Fault flags in a non-nB mode are ignored.** The motors behave normally in that mode. When you enter an nB mode with a flag still ON, the fault becomes active immediately.
- **Entering any mode resets the controller.** This includes `safety_killer_trigger`.
- **RW3C wing position in the manual modes:** all three manual (half-loop) modes put `rotwing_state` in FORCE_HOVER, so the wing is at skew 0°. The only exception is when `rotwing_state.force_skew` is set.

---

## Settings reference

### Fault injection

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `fault_pitch_motors` | OFF | — | OFF/ON | Faults **FRONT and BACK**. Their effectiveness columns are zeroed, so the allocator stops using them, and they receive the static fault command (see below). Yaw effectiveness of RIGHT/LEFT is also zeroed. |
| `fault_roll_motors` | OFF | — | OFF/ON | Same for **RIGHT and LEFT**. Yaw effectiveness of FRONT/BACK is zeroed. |
| `fault_ailerons` | ON (`RW3C_nB`: **OFF**) | `FAULT_AILERONS` | OFF/ON | On `RW3C_nB` it zeroes the aileron column, so the ailerons are not used. On every airframe it also **selects the allocation case** (see [Which axes are kept](#which-axes-are-kept-during-a-fault)). Leave it ON on the PlusQuad. |
| `delta_fault` | 1000 (PlusQuad: 700) | `DELTA_FAULT` | 0 – 5000 | The static command is `throttle stick − delta_fault`. **Lower `delta_fault` means more command to the faulted motors, which means a slower spin.** |
| `max_fault_mot` | 3000 (RW3C: 3300) | `MAX_FAULT_MOT` | 0 – 9600 | Upper limit on `throttle − delta_fault`, and the start value of the auto ramp. Keep it below `spin_prot_max_cmd`, because anything above that ceiling is cut. |
| `auto_fault_cmd` | OFF | — | OFF/ON | Test ramp: the static command goes from `max_fault_mot` down to 0 over 30 s (both values hard-coded), then switches itself OFF. The spin trim is disabled during the ramp. The envelope stays active. |

**The static fault command follows the throttle stick in every mode, including NAV and FORWARD.** In the nav modes the pilot's throttle stick still sets the faulted motors' command.

**Both pairs faulted:** not a supported case. It is treated exactly as no fault: no effectiveness is zeroed, no static command is applied, and the allocator keeps its nominal weights. You can switch from one faulted pair to the other directly; while both flags are briefly ON, the drone flies normally.

### Yaw spin protection (envelope)

This works like the PX4 TECS underspeed protection, but acts on the yaw rate instead of airspeed. The faulted motors' command is blended towards `spin_prot_max_cmd` as |r| grows from the start rate to the max rate.

```
ratio = clamp((|r| − spin_prot_start_rate) / (spin_prot_max_rate − spin_prot_start_rate), 0, 1)
cmd   = (1 − ratio) · min(static + spin_trim, spin_prot_max_cmd) + ratio · spin_prot_max_cmd
```

It only applies when exactly one pair is faulted and the controller type is nB. r is the filtered body yaw rate `LP.r.out` (see `r_LP_freq`).

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `spin_prot_max_rate` | 34.0 rad/s | `SPIN_PROT_MAX_RATE` | 0 – 60 | At or above this \|r\|, the faulted motors get `spin_prot_max_cmd`. This is the safety limit. 34 rad/s is just under the gyro range (±2000 °/s ≈ 34.9 rad/s). |
| `spin_prot_start_rate` | 28.0 rad/s | `SPIN_PROT_START_RATE` | 0 – 60 | The envelope starts to blend in above this \|r\| (and the trim starts rising). |
| `spin_prot_max_cmd` | 4800 | `SPIN_PROT_MAX_CMD` | 0 – 9600 | **The single ceiling** for the faulted motors. Nothing (static command, trim or envelope) can send more than this. |
| `spin_prot_min_gap` | 1.0 rad/s | `SPIN_PROT_MIN_GAP` | 0.1 – 10 | Minimum gap enforced between release < start < max. |

### Spin trim (slow integrator)

When the battery voltage drops, the same static command gives less counter-torque, and the spin speeds up. The trim slowly raises the static command while the envelope is active. This pushes the equilibrium back below `spin_prot_start_rate`, so the envelope only acts in real emergencies.

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `spin_trim_on` | ON | `SPIN_TRIM_ON` | OFF/ON | On/off switch. |
| `spin_trim_rate` | 1000 pprz/s | `SPIN_TRIM_RATE` | 0 – 1000 | How fast the trim rises, multiplied by `ratio` (e.g. 333 pprz/s at 30 rad/s, ratio 1/3). |
| `spin_trim_bleed_rate` | 1000 pprz/s | `SPIN_TRIM_BLEED_RATE` | 0 – 1000 | How fast the trim decays below the release rate. 0 means the trim only ever increases. |
| `spin_trim_release_rate` | 24.0 rad/s | `SPIN_TRIM_RELEASE_RATE` | 0 – 60 | The trim decays below this \|r\|. Between release and start it holds. |
| `spin_trim` | 0 | — | 0 – 9600 | **The trim itself.** Watch it live, or type 0 to reset it. It is always kept in [0, `spin_prot_max_cmd`]. |

The trim is reset to 0 unless **all** of these hold:
- `spin_trim_on` is ON,
- exactly one pair is faulted,
- the controller type is nB,
- the drone is in flight,
- `auto_fault_cmd` is OFF.

With the defaults, the \|r\| bands are:

```
0 ──── trim bleeds ──── 24 ── hold ── 28 ── envelope + trim rising ── 34 ── fully at spin_prot_max_cmd ──▶
                       release        start                            max
```

**How the parameters are kept valid:**
- **At compile time:** `_Static_assert` stops the build if the airframe values break the order `0 ≤ release ≤ start − gap`, `start ≤ max − gap`, `max ≤ 60`, or if a command/rate is out of its range.
- **At runtime:** `spin_prot_bound_params()` enforces the same limits every loop, so a bad in-flight setting cannot break anything. **The max rate is never raised to fix a setting.** If a setting breaks the order, the lower rates (start, release) are pushed down instead.
- **If you change a limit:** it appears in three places (the `_Static_assert`s, `spin_prot_bound_params()` and the settings XML), so update all three.

### Spin transition maneuver (nominal ↔ one pair faulted)

A GCS-triggered maneuver for entering and leaving a single-pair fault spin smoothly, with either the pitch pair (FRONT/BACK) or the roll pair (RIGHT/LEFT). It works in any nB mode.

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `spin_man_up` | OFF | — | OFF/ON | **Spin up.** With no fault active, the yaw-rate reference ramps from the current r towards the middle of the hold band, (release + start)/2. Once \|r\| ≥ `spin_trim_release_rate`, the fault flag of `spin_man_pair` is switched ON automatically and that pair blends to the static command. Clears itself when done. |
| `spin_man_pair` | PITCH | `SPIN_MAN_PAIR` (0 = pitch, 1 = roll) | PITCH/ROLL | Which pair the spin-up faults. Not used by the spin-down, which un-faults whichever single pair is faulted. |
| `spin_man_pitch_dir` | POS | `SPIN_MAN_PITCH_DIR` (1.0 / −1.0) | NEG/POS | Sign of r when the pitch pair is faulted (positive on the PlusQuad). The roll pair spins the opposite way. The spin-up ramps in this direction, so the spin never has to reverse when the fault engages. |
| `spin_man_down` | OFF | — | OFF/ON | **Spin down.** With exactly one pair faulted, the trim is raised open-loop (no bleed) until \|r\| < release − gap. Then the fault flag is switched OFF, that pair blends back to the allocator, and the yaw-rate reference starts at the current r and ramps to 0. It ends when \|r\| < `spin_man_done_rate`. Clears itself when done. |
| `spin_man_ramp_rate` | 2.0 rad/s² | `SPIN_MAN_RAMP_RATE` | 0.1 – 10 | Yaw-rate reference ramp, for both spin-up and stopping. |
| `spin_man_blend_time` | 1.0 s | `SPIN_MAN_BLEND_TIME` | 0.1 – 5 | Time over which FRONT/BACK blend between allocator and static command (no kick). |
| `spin_man_trim_rate` | 300 pprz/s | `SPIN_MAN_TRIM_RATE` | 0 – 1000 | Open-loop trim rise while slowing down. |
| `spin_man_done_rate` | 0.5 rad/s | `SPIN_MAN_DONE_RATE` | 0.1 – 5 | \|r\| below which the spin counts as stopped, and normal heading hold takes over. |

**During any phase:**
- the sticks command **N/E** instead of body axes;
- the yaw stick is ignored;
- the desired heading follows the actual heading.

**Spin direction:** `spin_man_pitch_dir` for the pitch pair, the opposite for the roll pair.

**Aborting:**

| Flag cleared during | What happens |
|---|---|
| Spin-up ramp | The spin is ramped back to 0 without faulting |
| Slow-down phase | Stops; the drone stays faulted and the normal trim takes over |
| Either blend | The blend always completes |
| Final ramp to 0 | Ignored. This is the safe way out |

**Reset:** a mode change, landing, or leaving an nB mode resets the maneuver.

**A trigger is rejected (cleared) when its precondition isn't met:**
- `spin_man_up` needs no fault flag set;
- `spin_man_down` needs exactly one of `fault_pitch_motors` / `fault_roll_motors` set.

**Heading after any fault:** whenever a fault flag is ON in an nB mode, `psi_des` follows the actual heading, so turning a fault OFF (by hand or by the maneuver) never causes a heading jump.

### Manual (half-loop) and position control

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `vel_ctrl_in_manual` | ON | — | OFF/ON | Manual modes only. ON: the roll/pitch sticks command a velocity of ±`pid_v_max_manual`, tracked by the k_K/P/I/D PID. OFF: the sticks command attitude directly, scaled by `max_phi`/`max_theta`. |
| `k_K` | 0.6 | — | 0.1 – 5 | Position error → velocity setpoint gain (NAV, all 3 axes; also the pusher position loop). The velocity setpoint is limited to `pid_v_max_nav`. |
| `k_P` | 1.8 | — | 0.1 – 5 | Velocity error → acceleration gain. The acceleration command is limited to `pid_a_max`. |
| `k_I` | 0.4 | — | 0 – 1 | Velocity error integral gain. The integral is clamped (±0.4) and **is not reset when you change mode**. |
| `k_D` | 0.2 | — | 0.1 – 5 | Derivative term, computed on the filtered measurement. |
| `pid_a_max` | 0.6 g (RW3C: 0.12 g) | `PID_A_MAX` | 0.05 – 1 g | Max acceleration setpoint of the velocity PID, manual and NAV. **In nB modes this is the tilt limit** of the thrust vector: 0.12 g ≈ 7°, 0.6 g ≈ 31° (`max_phi`/`max_theta` do not apply there). |
| `pid_v_max_manual` | 3 m/s | `PID_V_MAX_MANUAL` | 0.1 – 10 m/s | Max velocity setpoint in manual, and the velocity at full stick. |
| `pid_v_max_nav` | 3 m/s (RW3C: 0.5 m/s) | `PID_V_MAX_NAV` | 0.1 – 10 m/s | Max velocity setpoint in NAV (position error × `k_K`, then limited). |
| `ec_k3` | 22 (PlusQuad: 29) | `EC_K3` | 1 – 100 rad/s | Third gain of the ANDI error controller, for roll/pitch (and vertical in NAV). It must equal the hover motors' actuator dynamics (`ACT_DYN`): then ANDI's actuator inversion cancels and nB ANDI commands the same motor inputs as nB INDI. Only ANDI and nB ANDI use it; INDI uses 1. |
| `max_phi` | 30° (PQ: 45°, RW3C: 5°) | `MAX_PHI` | 2 – 30 (rad setting shown in deg) | Scales the roll stick when it commands attitude, and limits the roll angle computed from the acceleration command. **It does not limit the tilt in the nB modes.** There the limit is the 0.6 g cap above. |
| `max_theta` | 30° (PQ: 45°, RW3C: 5°) | `MAX_THETA` | 2 – 30 | Same as `max_phi`, for pitch. |
| `oneloop_nB_Z_hold` | OFF | — | OFF/ON | Auto modes: the sticks set roll/pitch while altitude stays automatic. **It only works in MODULE (CTRL_ANDI).** In the nB modes the sticks are ignored. |

### Heading

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `heading_manual` (`take_heading`) | ON in the airframes | `HEADING_MANUAL` | OFF/ON | Auto modes. ON: the heading target is `psi_des`. OFF: the heading follows a coordinated-turn estimate. That is meant for forward flight: in hover it drifts, because the speed is clamped to 1 m/s. |
| `yaw_stick_in_auto` (`yaw_stick_on`) | ON in the airframes | `YAW_STICK_IN_AUTO` | OFF/ON | With `heading_manual` ON, the yaw stick moves `psi_des` at up to `MAX_R` (120 °/s in the airframes). |
| `psi_des_deg` (`psi_des`) | current heading | — | −180 – 180 | Target heading when `heading_manual` is ON. It is reset to the actual heading on the ground. |
| `fwd_sideslip_gain` | 0.25 in the airframes | `FWD_SIDESLIP_GAIN` (**no** `ONELOOP_NB_` prefix) | 0.01 – 20 | Lateral-acceleration correction in the coordinated-turn estimate. It only matters with `heading_manual` OFF. |

### Pusher (RW3C only)

These are only compiled when the airframe defines `ROTWING_EFF_SCHED_MP_dFdu`, i.e. on the RW3C and not on the PlusQuad.

| Setting | Default | Range | What it does |
|---|---|---|---|
| `use_push_PID` | OFF | OFF/ON | Pusher velocity control. It forces the thrust direction to level and drives `COMMAND_MOTOR_PUSHER` with a phase-modulated command. While it is OFF the pusher command is held at 0. |
| `use_push_Position` | OFF | OFF/ON | Manual modes, with `use_push_PID` ON: the pusher tracks the nav target position instead of the stick. NAV always tracks position. |
| `max_pusher_cmd` | 7500 | 0 – 9600 | Pusher command limit in the pusher PID. |

### Filters

| Setting | Default | Define | Range | What it does |
|---|---|---|---|---|
| `oneloop_nB_filt_cutoff` | 2.0 Hz (PQ: 12, RW3C: 2) | `FILT_CUTOFF` | 0.5 – 20 | Main 2nd-order Butterworth cutoff. It is used for the angular accelerations, the linear accelerations, the actuator-state filters and the nB filters. **This is the setting to use for all of those.** |
| `r_LP_freq` (`LP.r.freq_set`) | 15 Hz | — | 0.5 – 20 | Yaw-rate filter. It is the \|r\| used by the spin protection. A lower value gives less ripple but more lag. |
| `p_LP_freq`, `q_LP_freq` | 15 Hz | — | 0.5 – 20 | Roll/pitch rate filters (the rate feedback). |

### Gains (poles)

The gains are rebuilt every loop, so changes take effect immediately. Naming: `_e` is the error controller, `_rm` the reference model. `omega_n` is in rad/s.

| Setting | What it actually does |
|---|---|
| `p_head_e.omega_n`, `p_head_e.zeta` | **Yaw** error-controller gains, in every controller type. Also used by SpinQuad. |
| `p_att_rm.omega_n`, `p_att_rm.zeta` | Reference model for the **x/y of the thrust direction nI**. It only matters in the nB modes, because the attitude reference model is disabled (`OVERRIDE_ATT_RM`). |
| `p_head_rm.omega_n`, `p_head_rm.zeta` | Despite the name, this shapes the **z component of the nI reference** (nB modes). The heading reference is a direct step. |
| `p_att_e.*`, `p_pos_e.*`, `p_alt_e.*`, `p_alt_rm.*`, `p_pos_rm.*`, all `*.p3` | **No effect** on the control law (see [Settings with no effect](#settings-with-no-effect)). |

### Safety killer

| Setting | Default | Range | What it does |
|---|---|---|---|
| `use_safety_killer` | OFF | OFF/ON | When ON, the killer trips if any actuator's modelled state goes above `safety_killer_cutoff`, or its command goes above 8500 (hard-coded). On `RW3C_nB` the aileron is checked too. |
| `safety_killer_trigger` | OFF | OFF/ON | The trip flag. It latches, and while it is set **all** actuator commands are 0. It resets when you enter a mode, or when you set it to OFF here. |
| `safety_killer_cutoff` | 7500 | 1000 – 9600 | Trip threshold. |

### Other / test

| Setting | Default | What it does |
|---|---|---|
| `SpinQuad` | OFF | Yaw-rate tracking: the reference is AUX4 mapped to 0 – 40 rad/s. **In every fault case yaw is dropped, so SpinQuad only acts with no fault.** AUX4 is also mapped to `RADIO_CONTROL_THRUST_X`. |
| `state_compensation_on` | OFF | Subtracts gyroscopic cross-coupling from the roll/pitch pseudo-controls. It uses body-axis formulas, while the nB modes use swapped nB channels, so check it before relying on it. |
| `TestMotorIDX` | 0 | Only works in firmware built with `ONELOOP_NB_DEBUG_MODE TRUE`. That build sends throttle to this motor and 0 to all the others, bypassing the killer. |

### Settings with no effect

Leave these alone. Changing them in flight does nothing. The code line numbers are as of this writing.

| Setting | Why |
|---|---|
| `pdot/qdot/rdot/ax/ay/az_LP_freq` | Overwritten with `oneloop_nB_filt_cutoff` every loop. Use that setting instead. |
| `p_att_e.*` | The pitch/roll gains are hard-coded right after they are computed (`k1 = 4.19, k2 = 10.01`; `k3` is the `ec_k3` setting). |
| `p_pos_e.*`, `p_alt_e.*`, `p_alt_rm.*` | The gains they produce are never used. Position is controlled by the k_K/P/I/D PID, and the vertical k3 is also `ec_k3`. |
| `p_pos_rm.*` | Only overwrites `nav_hybrid_pos_gain`, which the controller does not use. |
| `*_rm.p3` | Overwritten every loop with `omega_n · zeta`. |
| `max_bank` | Only copied to `nav_hybrid_max_bank`. It does not limit the tilt. |
| `max_airspeed` (`max_as`) | Only used in `reshape_wind()`, which is never called. |
| `drop_yaw` | Overwritten every loop by the fault allocation logic. |
| `SpinQuadRate`, `ctrl_off` | Never read. |

---

## Which axes are kept during a fault

`set_WLS_settings()` chooses, for each fault combination, which pseudo-controls the allocator still tries to track. Yaw is always dropped when a fault is active.

**Axis names in nB modes.** The rows are swapped, which makes the flag names misleading:
- the row called `ap` is **nB_x**, driven by body **pitch** moment;
- the row called `aq` is **nB_y**, driven by body **roll** moment.

So `drop_roll` actually drops the pitch-driven channel, and `drop_pitch` drops the roll-driven channel.

**Quad vs forward.** `IN_QUAD` is false when the filtered RW3C skew is above **70°**. There is no hysteresis. On the PlusQuad `IN_QUAD` is always true.

| Faulted pair | `fault_ailerons` | IN_QUAD | Kept channel | Status |
|---|---|---|---|---|
| pitch (F/B) | ON | quad | `aq` (roll moment from R/L) | OK — **PlusQuad and RW3C quad case** |
| pitch (F/B) | ON | forward | `ap` | OK |
| roll (R/L) | ON | quad | `ap` (pitch from F/B) | OK (fixed: it used to keep `aq`, a channel the remaining F/B cannot control) |
| roll (R/L) | ON | forward | `ap` (pitch from F/B) | OK, ailerons not used |
| pitch (F/B) | OFF | quad | `aq` | OK |
| pitch (F/B) | OFF | forward | `ap` + `aq` (ailerons for roll) | OK (RW3C_nB) |
| roll (R/L) | OFF | quad | `ap` (pitch from F/B) | OK |
| roll (R/L) | OFF | forward | `ap` + `aq` (ailerons for roll) | OK — **RW3C forward case** |

---

## Guide 1 — PlusQuadV3, faulted pitch motors

FRONT and BACK get the static command. The drone spins, and RIGHT/LEFT keep controlling the thrust direction and altitude.

### Leave untouched
- `fault_roll_motors` OFF, and `fault_ailerons` **ON** (the default). With ON the allocator uses the correct quad case; the PlusQuad has no ailerons anyway.
- `SpinQuad` OFF, `auto_fault_cmd` OFF, `state_compensation_on` OFF.
- All the pole settings, and the settings with no effect.
- The spin protection and spin trim defaults (34 / 28 / 24 rad/s, max 4800, trim ON at 1000/1000 pprz/s).

### Check before flight
- `delta_fault` = 700 and `max_fault_mot` = 3000. The faulted motors get `throttle − 700`, capped at 3000.
- `max_fault_mot` (3000) < `spin_prot_max_cmd` (4800).
- Optional: `use_safety_killer`. It latches and cuts **all** motors, so decide beforehand whether you want that in a spinning test.

### In flight
1. Take off and hover in an **nB mode**. Manual: MODE 0 with AP switch 2 (`nB INDI`) or 1 (`nB_ANDI`). Auto: NAV.
2. Hold the throttle at the hover setting. Remember that from the moment the fault is ON, the throttle stick also sets the faulted motors' command.
3. Turn **`fault_pitch_motors` ON**. The front/back motors switch to the static command and the drone starts spinning.
4. Watch \|r\| and `spin_trim`:
   - **Spin too fast** (the envelope keeps acting, and `spin_trim` rises every flight): **lower `delta_fault`** to give the faulted motors more command.
   - **Spin too slow or stopping:** raise `delta_fault`.
   - **`spin_trim` rising slowly over the flight:** that is normal, it is compensating for the battery drain.
5. To end the test, turn **`fault_pitch_motors` OFF** at a safe altitude:
   - all four motors go back to the allocator, and yaw control returns while the drone is still spinning fast;
   - the stick frame switches back from NED to body.
   - The heading target is the current heading, so there is no heading jump, but the controller will brake a fast spin abruptly. Be ready for a yaw transient, or use `spin_man_down` instead.

### Alternative: use the spin transition maneuver
Instead of steps 3 and 5, hover in an nB mode and:
- set **`spin_man_up` ON** to spin up and fault the pitch motors automatically;
- later, set **`spin_man_down` ON** to slow down, un-fault, and stop the spin.

Both clear themselves when finished. If the healthy quad cannot reach `spin_trim_release_rate` (yaw has the lowest allocation priority), the spin-up waits at the highest rate it can reach. Clear `spin_man_up` to abort it.

### Optional: automatic test ramp
With the fault ON, set `auto_fault_cmd` ON. The faulted motors ramp from `max_fault_mot` down to 0 over 30 s, to find the spin rate at which control is lost. The trim is off during the ramp. The envelope still acts, so the spin cannot go past `spin_prot_max_rate`.

---

## Guide 2 — RW3C_nB, faulted pitch motors in quad, faulted roll motors in forward flight

On the RW3C, FRONT/BACK are fixed to the fuselage. RIGHT/LEFT rotate with the wing:
- at skew 0° they give roll;
- at skew 90° they give pitch.

In forward flight, roll can only come from the ailerons. The static fault command and the spin protection work the same in quad and in forward flight; only the allocation case changes with skew.

### Leave untouched
- `SpinQuad` OFF, `auto_fault_cmd` OFF, `state_compensation_on` OFF, all poles, the settings with no effect, and the spin protection/trim defaults.
- `use_push_*` as you normally fly.

### Check before flight
- `delta_fault` is **1000 on the RW3C**: the airframe has no `DELTA_FAULT`, so the C default applies. If you tune it, add `<define name="DELTA_FAULT" .../>` to `rotwing_v3c_nB_ail.xml`.
- `max_fault_mot` (3000) < `spin_prot_max_cmd` (4800).
- If you use `use_safety_killer`: the aileron is also checked, so an aileron command above 7500 or 8500 trips the killer.

### Part A — pitch motors faulted in quad
1. Hover in an nB mode. In the manual nB modes the wing is forced to 0°, so the RW3C is in quad automatically.
2. Leave `fault_ailerons` as it boots (**OFF** on `RW3C_nB`). Both settings give a correct quad case: the ailerons have no effect at low skew, and the servos are forced to 0 below 20°.
3. Turn **`fault_pitch_motors` ON**. FRONT/BACK get the static command, and RIGHT/LEFT control the roll-moment channel.
4. Tune the spin with `delta_fault` as in Guide 1, and watch `spin_trim`.
5. Turn **`fault_pitch_motors` OFF** before any transition to forward flight.

### Part B — roll motors faulted in forward flight
1. **First** get the skew above 70° in an nB mode, and keep it well away from 70°:
   - Use FORWARD (`nB_NAV_INDI`) or NAV, where the wing follows the airspeed schedule.
   - Or, in the manual nB modes, use `rotwing_state.force_skew` with `sp_skew_angle`.
2. Check that **`fault_ailerons` is OFF** (its boot value on `RW3C_nB`). The ailerons then keep controlling roll, and the allocator tracks both channels. If the skew dips below 70°, both settings fall into a correct quad case.
3. Turn **`fault_roll_motors` ON**. RIGHT/LEFT get the static command, and FRONT/BACK give pitch.
4. Watch \|r\| and `spin_trim`. The envelope and trim work the same as in quad.
5. Turn **`fault_roll_motors` OFF** **before** the wing rotates back to hover.

**Order rule for Part B:** skew above 70° first, fault ON second; fault OFF first, transition back second.

---

## Known issues (not fixed yet)

- **Allocation weights are reset every loop:** `set_WLS_settings()` starts from the nominal weights each time, so nothing carries over when switching from one faulted pair to the other. The drop flags are applied one loop late (`drop_axis()` runs before `set_WLS_settings()`).
- **No hysteresis** at the 70° skew threshold, so the allocation case can switch back and forth near 70°.
- **Comments don't match the code** in one allocation case (roll fault in quad without ailerons). The comment on `ONELOOP_NB_DEBUG_MODE` is also inverted.
- **Filters not updated in flight:** `accely_filt` and `airspeed_filt` keep their boot cutoff, and don't follow in-flight changes to `oneloop_nB_filt_cutoff`.
