# Cascade Lift Subsystem — Tuning Guide

A complete cascade (continuous-loop) lift controller integrated into the
RW-Template. It runs its own background task, so the same API works in both
driver control and autonomous.

## Files added / changed

| File | What |
|------|------|
| `custom/include/lift.h` | NEW — `LiftHeight` enum, `CascadeLift` class, `extern CascadeLift lift;` |
| `custom/src/lift.cpp` | NEW — control loop, PID, feedforward, motion profile, manual override, homing |
| `custom/include/robot-config.h` | EDITED — lift tuning `extern` params + forward-decl of `lift` |
| `custom/src/robot-config.cpp` | EDITED — lift parameter values + constructs `CascadeLift lift(arm_motor);` |
| `custom/src/user.cpp` | EDITED — driver controls (R1/R2 manual, presets), `lift.start()` in pre-auton |
| `custom/src/autonomous.cpp` | EDITED — comment block showing the auton API |
| `custom/include/user.h` | unchanged (`lift` is already `extern` in robot-config.h) |

No changes to the build system — the makefile auto-compiles everything in
`custom/src/*.cpp`, so the new `lift.cpp` is picked up automatically.

## Wiring & port allocation

The lift **reuses the template's arm motor group** (`arm_motor1`/`arm_motor2`
on `PORT16`/`PORT17`) because the V5 brain only has 21 smart ports and the
example config uses them all. The cascade lift replaces the arm, so the
`CascadeLift` controller now owns those motors.

- Both lift motors should spin the same direction mechanically. If one runs
  backwards, flip the `reversed` flag on that motor in `robot-config.cpp`
  (the `true`/`false` 3rd argument to `motor(...)`).
- Gear ratio: the template uses `ratio18_1` (green) for the arm. Use whatever
  your lift needs — `ratio36_1` (red) for more torque, `ratio6_1` (blue) for
  speed.

If you would rather use different ports: free two smart ports (e.g. drop an
unused example sensor), move the two `motor(...)` constructors in
`robot-config.cpp` onto those ports, and rebuild the `arm_motor` group. Nothing
else in the lift code references ports directly.

## API

```cpp
// Automatic moves (driver + autonomous)
lift.setHeight(LiftHeight::LOW);
lift.setHeight(LiftHeight::HIGH);
lift.setHeight(LiftHeight::MEDIUM);
lift.setHeight(LiftHeight::TOP);
lift.setHeight(LiftHeight::BOTTOM);
lift.setHeight(rawDegrees);                 // raw target, clamped to soft limits
lift.setHeight(rawDegrees, maxSpeedDegPerS); // ...with a custom max speed

// Manual driver control — ALWAYS takes priority over auto moves
lift.manual(100);   // full up  (maps to +12 V)
lift.manual(-100);  // full down (maps to -12 V)
lift.manual(0);     // release override -> hold wherever it is

// Stop + hold current position
lift.stopAndHold();

// PID enable/disable
lift.enablePID();
lift.disablePID();   // motors hold with feedforward only

// Homing (optional, call in pre-auton)
lift.home();                       // defaults: 3 V down, 2000 ms timeout
lift.home(2.5, 1500);              // custom voltage + timeout

// Queries
lift.isAtTarget();                  // true when the move has settled
lift.waitUntilAtTarget();           // blocks until settled (2 s default)
lift.waitUntilAtTarget(3000);       // ...custom timeout (msec)
lift.getPosition();                // current motor-degrees
lift.getMode();                     // LiftMode::{IDLE,POSITION,MANUAL,HOLD,HOMING}
lift.zeroPosition();               // reset encoder zero point
```

### Autonomous example

```cpp
void myAuton() {
  // ... drive somewhere ...
  lift.setHeight(LiftHeight::HIGH);
  lift.waitUntilAtTarget();
  // ... score ...
  lift.setHeight(LiftHeight::LOW);
  lift.waitUntilAtTarget();
}
```

## Driver controls (defaults, remap freely)

| Button | Action |
|--------|--------|
| R1 | Manual up (priority over presets) |
| R2 | Manual down (priority over presets) |
| Up arrow | Preset HIGH |
| Down arrow | Preset LOW |
| Y | Preset TOP |
| A | Preset MEDIUM |
| X | Preset BOTTOM |

Manual override always wins. When you release R1/R2 the lift re-engages a PID
hold at whatever height it's at. Presets use rising-edge detection, so holding
a button won't keep restarting the motion profile.

## How it works

The `controlLoop()` task runs every 10 ms and switches on `LiftMode`:

- **POSITION** — runs a trapezoidal motion profile (accelerate → cruise →
  brake) to generate a time-varying setpoint, then a PID follows that
  setpoint. Gravity feedforward is added on top.
- **HOLD** — PID to a fixed target with no profiling (used after manual
  release and `stopAndHold()`).
- **MANUAL** — direct voltage from the driver (still clamped to soft limits
  and still gets feedforward). Bypasses PID entirely.
- **HOMING** — drives down at low voltage until velocity stalls for 200 ms
  (or timeout), then zeros the encoder. PID is bypassed.
- **IDLE** — feedforward only.

The PID is implemented inside the class (not the template's `PID` class)
because the template's `PID::setTarget()` does not reset integral/derivative
state, which causes carry-over between moves.

Gravity feedforward (`lift_gravity_ff`) is a flat upward voltage. For a true
cascade lift the load is nearly constant across the travel, so one number
works. The feedforward tapers to zero within the bottom 2 degrees so the lift
rests fully on the stop instead of fighting it.

## Tuning procedure

All parameters live in `custom/src/robot-config.cpp` under
"CASCADE LIFT CONFIGURATION". All positions are **motor-degrees** (encoder
degrees at the motor shaft, before external gearing).

### 1. Zero the lift first

Either mechanically set the lift to the bottom and call `lift.zeroPosition()`,
or enable `lift.home()` in `runPreAutonomous()` (uncomment it in `user.cpp`).
Everything below assumes 0 = bottom.

### 2. Set soft limits

- `lift_min_position = 0` (bottom).
- `lift_max_position` = drive the lift to the top with R1, read
  `lift.getPosition()` (print to the brain screen or controller), and set it
  a few degrees below the hard stop.

### 3. Find gravity feedforward (`lift_gravity_ff`)

Disable PID (`lift.disablePID()`), raise the lift to mid-height, then slowly
increase `lift_gravity_ff` until the lift holds its height without dropping
and without drifting up. For a typical 2-motor cascade this lands around
1.0–2.5 V. Re-enable PID when done.

### 4. Tune proportional (`lift_pid_kp`)

Start with `ki = 0`, `kd = 0`. Command a move (e.g. `lift.setHeight(500)`).
Increase `kp` until the lift reaches the target briskly but doesn't slam or
oscillate badly. A starting point is ~0.1 V/deg.

### 5. Tune derivative (`lift_pid_kd`)

Add `kd` to dampen overshoot. Increase until moves settle cleanly with no
bounce. Typical: 0.3–1.0. If the lift feels "jerky" or noisy, `kd` is too high.

### 6. Tune integral (`lift_pid_ki`) — optional

Usually unnecessary for a lift because feedforward handles the steady-state
load. If the lift consistently settles a few degrees short, add a small `ki`
(0.001–0.01). The controller has anti-windup and clears the integral on sign
change, so it won't blow up.

### 7. Tune motion profile

- `lift_profile_speed` (deg/s) — cruise speed of automatic moves. Too high and
  the PID can't follow (overshoot); too low and moves feel sluggish.
- `lift_profile_accel` (deg/s²) — how fast it ramps to cruise. Lower = smoother
  but slower; higher = snappier but can slip belts/chain.

If moves overshoot, lower `lift_profile_speed` first, then `kp`.

### 8. Tune arrival detection

- `lift_arrival_tolerance` (deg) — how close counts as "there". 2–4 is typical.
- `lift_arrival_settle` (msec) — how long it must hold within tolerance before
  `isAtTarget()` returns true. 100–200 ms avoids false "arrived" signals.

### 9. Measure presets

With the lift zeroed, drive to each desired height with R1 and read
`lift.getPosition()`. Set `lift_preset_low/medium/high/top` to those values.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Lift drops when idle | Increase `lift_gravity_ff` |
| Lift drifts up when idle | Decrease `lift_gravity_ff` |
| Overshoots targets | Lower `lift_profile_speed` or `kp`; raise `kd` |
| Sluggish | Raise `lift_profile_speed`/`kp` |
| Jerky/noisy | Lower `kd` |
| Sits a few deg short | Add small `ki`, or raise `kp` |
| Won't move in auton | Make sure `lift.start()` runs in `runPreAutonomous()` |
| Motors fight each other | You're still running `armPIDLoop()`/direct `arm_motor` commands — remove them, the lift owns these motors now |
| One motor runs backwards | Flip the `reversed` flag on that motor in `robot-config.cpp` |
| Homing false-triggers | Raise the 200 ms stall duration in `lift.cpp`, or raise `down_voltage` |

## Safety

- Soft limits are enforced in every mode except homing. Set them correctly.
- Homing moves the lift. Leave it commented in pre-auton unless you've tested
  it on your robot — it can knock into things on a crowded field.
- `lift_max_output` clamps the final voltage. Default 12 V (full).

## A note on threading

The control loop runs in its own task while the driver loop and autonomous
write commands (`manual`, `setHeight`, etc.) from another task. The shared
state (`mode`, `target`, `manual_power`, ...) is plain `double`/`int`/`enum`,
not locked. On the V5 (32-bit ARM) a torn read of a double is possible but
harmless here — the worst case is one 10 ms iteration using a slightly stale
command. This is fine for a lift. If you ever see weird glitches, wrap command
updates in a `vex::mutex`.

## Autonomous

The default `auton_selected` was changed to `1` (`exampleAuton`) because the
old default `3` (`redGoalRush`) directly commands `arm_motor` / runs
`armPIDLoop()`, which would fight the lift controller. Rewrite those arm
sections to use the lift API before re-selecting case 3.
