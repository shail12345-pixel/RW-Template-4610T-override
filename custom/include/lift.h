#ifndef __LIFT_H__
#define __LIFT_H__

#include "vex.h"

using namespace vex;

/*
 * ===========================================================================
 *  CASCADE LIFT SUBSYSTEM
 *  ---------------------
 *  A complete, self-contained cascade (continuous-loop) lift controller for
 *  the RW-Template. It runs its own background control task, so the same API
 *  works in both driver control and autonomous.
 *
 *  Features
 *    - PID position control with gravity (feedforward) compensation
 *    - Trapezoidal motion profiling for smooth automatic moves
 *    - Soft limits (top + bottom) to protect the mechanism
 *    - Manual driver override that ALWAYS takes priority over automatic moves
 *    - Position hold (the lift holds wherever the driver lets go)
 *    - Named presets via the LiftHeight enum
 *    - Optional homing routine (stall / current detection)
 *    - Autonomous API: setHeight / waitUntilAtTarget
 *
 *  The lift reuses the template's arm motor group (PORT16 / PORT17) by default.
 *  See LIFT_TUNING.md for wiring, port allocation, and tuning instructions.
 * ===========================================================================
 */

/*
 * Named preset heights, in motor-degrees.
 * The actual degree values live in robot-config.cpp (lift_preset_*).
 * Add / rename entries here and add matching values in robot-config.cpp.
 */
enum class LiftHeight {
  BOTTOM,   // Fully lowered / resting
  LOW,      // e.g. ground pickup / low goal
  MEDIUM,   // e.g. middle height
  HIGH,     // e.g. high goal / wall stake
  TOP       // Fully raised
};

/*
 * Operating mode of the controller. The control loop switches on this each
 * iteration. Manual always wins; the driver code can drop into MANUAL at any
 * time and the loop will re-engage HOLD at the current position on release.
 */
enum class LiftMode {
  IDLE,     // No command yet (motors held)
  POSITION, // PID + profile to a target
  MANUAL,   // Direct voltage from the driver
  HOLD,     // Hold current position (PID, no profiling)
  HOMING    // Driving down to find the zero point
};

class CascadeLift {
 public:
  /*
   * Construct the lift around an existing motor group.
   * Pass the arm_motor group (PORT16/17) by default — see robot-config.cpp.
   */
  CascadeLift(motor_group& motors);

  /* Start the background control task. Idempotent (safe to call repeatedly). */
  void start();

  /* ---- Automatic API (driver + autonomous) ---- */

  // Move to a named preset height. Ignores repeated calls to the same preset.
  void setHeight(LiftHeight height);

  // Move to a raw target in motor-degrees (clamped to soft limits).
  void setHeight(double target_deg);

  // Move to a raw target with a custom max speed (deg/sec) for this move only.
  void setHeight(double target_deg, double max_speed_deg_s);

  /* ---- Manual / driver API ---- */

  // Direct manual control. power is -100..100 (mapped to -12..+12 V).
  // Calling manual(nonzero) immediately takes priority over any auto move.
  // manual(0) releases override and re-engages position hold at the
  // current position (it does NOT keep refreshing the override timer).
  void manual(double power);

  // Stop any motion and hold the current position (PID hold).
  void stopAndHold();

  /* ---- PID enable / disable ---- */

  void enablePID();   // Re-enable closed-loop position control (default).
  void disablePID();  // Disable PID; motors hold with feedforward only.

  /* ---- Homing ---- */

  // Drive the lift downward at low voltage until it stalls (low velocity +
  // current spike) or a timeout is reached, then zero the position. Disables
  // PID and ignores soft limits while homing. Safe to call in pre-auton.
  void home(double down_voltage = 3.0, double timeout_msec = 2000);

  // Manually reset the encoder zero point (e.g. after mechanical zeroing).
  void zeroPosition();

  /* ---- Queries ---- */

  // True once the current move has settled within tolerance.
  bool isAtTarget();

  // Block (from a thread) until the current move settles or times out.
  // Default timeout 2000 msec. Returns true if settled.
  bool waitUntilAtTarget(double timeout_msec = 2000);

  // Current lift position in motor-degrees.
  double getPosition();

  // Current controller mode.
  LiftMode getMode();

  // Last commanded target (motor-degrees).
  double getTarget();

  // Background control loop. Public so the free-function task wrapper
  // (liftControlTask in lift.cpp) can call it on the global lift instance.
  void controlLoop();

 private:
  // One trapezoidal-profile step: advances the setpoint toward the final
  // target and returns the profiled setpoint (motor-degrees).
  double motionProfileStep(double dt_sec);

  // Resolve a LiftHeight preset to a degree value.
  double resolvePreset(LiftHeight h);

  // Clamp a value to the configured soft limits.
  double clampToLimits(double value);

  // Reset internal PID state (call on every new target / mode change).
  void resetPID(double initial_error = 0);

  // Member references / state
  motor_group& lift_motors;

  bool started = false;
  LiftMode mode = LiftMode::IDLE;

  // Targets (motor-degrees)
  double target = 0;            // final target for this move
  double profile_setpoint = 0;  // current motion-profile setpoint
  double hold_position = 0;     // position captured for HOLD after manual

  // Manual override state
  double manual_power = 0;      // -100..100
  bool   manual_active = false;

  // PID state (self-contained)
  double kp = 0, ki = 0, kd = 0;
  double pid_error = 0, pid_prev_error = 0, pid_integral = 0;
  bool   pid_enabled = true;

  // Feedforward (gravity) compensation, in volts. Added whenever lift is up.
  double kG = 0;

  // Motion-profile limits
  double max_speed_deg_s = 0;    // cruise speed for current move
  double max_accel_deg_s2 = 0;   // acceleration for current move
  double profile_velocity = 0;   // current (signed) profile velocity
  bool   profiling = false;      // true while a profiled move is running

  // Arrival detection
  double arrival_tolerance = 0; // deg
  double arrival_settle_ms = 0; // time within tolerance to be "arrived"
  double arrival_timer = 0;
  bool   arrived = false;

  // Tuning parameters (populated from robot-config.cpp globals in start())
  double lift_kp = 0, lift_ki = 0, lift_kd = 0;
  double lift_kG = 0;
  double lift_max_speed = 0;       // default cruise speed (deg/s)
  double lift_max_accel = 0;       // default accel (deg/s^2)
  double lift_tolerance = 0;      // arrival tolerance (deg)
  double lift_settle_ms = 0;      // arrival settle time (msec)
  double lift_min_pos = 0;        // bottom soft limit (deg)
  double lift_max_pos = 0;        // top soft limit (deg)
  double lift_max_voltage = 12.0; // output clamp (V)

  // Preset degree values (copied from globals in start())
  double preset_bottom = 0, preset_low = 0, preset_medium = 0,
         preset_high = 0, preset_top = 0;

  // Remember last preset to ignore duplicate setHeight() calls.
  LiftHeight last_preset = LiftHeight::BOTTOM;
};

// Global instance, defined in custom/src/robot-config.cpp.
extern CascadeLift lift;

#endif // __LIFT_H__
