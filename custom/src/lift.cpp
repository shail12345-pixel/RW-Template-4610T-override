#include "vex.h"
#include "../custom/include/lift.h"
#include "../custom/include/robot-config.h"

#include <cmath>

using namespace vex;

// Forward declaration: free-function task wrapper (defined below). Needed so
// vex::thread can launch the member control loop on the global lift via a
// plain function pointer instead of a capturing lambda.
void liftControlTask();

// ============================================================================
//  CONSTRUCTION / STARTUP
// ============================================================================

CascadeLift::CascadeLift(motor_group& motors) : lift_motors(motors) {
  // Nothing else here; tuning is loaded in start() once globals are set.
}

void CascadeLift::start() {
  if (started) return; // idempotent

  // Pull tuning from the robot-config globals (see robot-config.cpp).
  lift_kp        = lift_pid_kp;
  lift_ki        = lift_pid_ki;
  lift_kd        = lift_pid_kd;
  lift_kG        = lift_gravity_ff;     // volts of upward feedforward
  lift_max_speed = lift_profile_speed;  // default cruise speed (deg/s)
  lift_max_accel = lift_profile_accel;  // default accel (deg/s^2)
  lift_tolerance = lift_arrival_tolerance;
  lift_settle_ms = lift_arrival_settle;
  lift_min_pos   = lift_min_position;
  lift_max_pos   = lift_max_position;
  lift_max_voltage = lift_max_output;

  preset_bottom = lift_preset_bottom;
  preset_low    = lift_preset_low;
  preset_medium = lift_preset_medium;
  preset_high   = lift_preset_high;
  preset_top    = lift_preset_top;

  // Seed state from current encoder reading so the lift does not jump on boot.
  hold_position = lift_motors.position(degrees);
  target = hold_position;
  profile_setpoint = hold_position;

  started = true;
  thread t = thread(liftControlTask);
  (void)t; // thread runs detached
}

// Free-function wrapper so vex::thread (which needs a plain function pointer,
// not a capturing lambda) can launch the member control loop on the global lift.
void liftControlTask() {
  lift.controlLoop();
}

// ============================================================================
//  PUBLIC API
// ============================================================================

void CascadeLift::setHeight(LiftHeight height) {
  // Ignore repeated calls to the same preset so held buttons do not keep
  // resetting the motion profile.
  if (mode == LiftMode::POSITION && last_preset == height && !manual_active) {
    return;
  }
  last_preset = height;
  setHeight(resolvePreset(height), lift_max_speed);
}

void CascadeLift::setHeight(double target_deg) {
  setHeight(target_deg, lift_max_speed);
}

void CascadeLift::setHeight(double target_deg, double max_speed_deg_s) {
  if (!started) start();

  target = clampToLimits(target_deg);
  profile_setpoint = getPosition();        // start profile from current pos
  this->max_speed_deg_s = (max_speed_deg_s > 0) ? max_speed_deg_s : lift_max_speed;
  this->max_accel_deg_s2 = lift_max_accel;  // REQUIRED: without this the profile never moves
  profile_velocity = 0;                     // start each move from rest
  profiling = (std::fabs(target - profile_setpoint) > lift_tolerance);
  resetPID(target - profile_setpoint);      // clear stale integral / derivative
  arrived = false;
  arrival_timer = Brain.timer(msec);
  manual_active = false;
  mode = LiftMode::POSITION;
}

void CascadeLift::manual(double power) {
  if (!started) start();

  // Clamp power to -100..100.
  if (power > 100) power = 100;
  if (power < -100) power = -100;

  if (power == 0) {
    // Releasing the stick: hand control back to a hold at the current spot.
    // Do NOT keep refreshing the manual timer (that would lock out auto).
    if (manual_active) {
      manual_active = false;
      hold_position = getPosition();
      // Re-engage hold with a fresh, non-profiled target.
      target = hold_position;
      profile_setpoint = hold_position;
      profiling = false;
      resetPID(0); // clear any integral/derivative carried from manual
      arrived = false;
      arrival_timer = Brain.timer(msec);
      mode = LiftMode::HOLD;
    }
    return;
  }

  manual_power = power;
  manual_active = true;
  mode = LiftMode::MANUAL;
}

void CascadeLift::stopAndHold() {
  if (!started) start();
  manual_active = false;
  hold_position = getPosition();
  target = hold_position;
  profile_setpoint = hold_position;
  profiling = false;
  resetPID(0); // clear state for a clean hold
  arrived = true; // holding counts as "at target" so chained calls proceed
  mode = LiftMode::HOLD;
}

void CascadeLift::enablePID()  { pid_enabled = true; }
void CascadeLift::disablePID() { pid_enabled = false; }

void CascadeLift::home(double down_voltage, double timeout_msec) {
  if (!started) start();

  mode = LiftMode::HOMING;
  pid_enabled = true; // re-enable PID after homing finishes
  double start_time = Brain.timer(msec);
  double prev_pos = getPosition();
  double stall_start = 0;
  bool stalled = false;

  // Drive downward until stalled (low velocity for a sustained period) or
  // timeout. Velocity-based stall detection is API-safe across VEX versions.
  while (true) {
    double now = Brain.timer(msec);
    if (now - start_time > timeout_msec) break;

    lift_motors.spin(fwd, -std::fabs(down_voltage), volt);

    double pos = getPosition();
    // Velocity over the last 50 msec (deg / 0.05 s). Downward is negative.
    double vel = (pos - prev_pos) / 0.05;
    prev_pos = pos;

    // Give it a moment to start moving before checking for a stall, so the
    // initial static state is not mistaken for hitting the stop.
    bool moving_now = (vel < -1.5); // moving downward noticeably
    if (now - start_time > 200) {
      if (!moving_now) {
        if (!stalled) { stalled = true; stall_start = now; }
        if (now - stall_start > 200) break; // sustained stall 200 msec
      } else {
        stalled = false;
      }
    }

    wait(50, msec);
  }

  lift_motors.stop(hold);
  zeroPosition();

  hold_position = 0;
  target = 0;
  profile_setpoint = 0;
  resetPID(0);
  arrived = true;
  mode = LiftMode::HOLD;
}

void CascadeLift::zeroPosition() {
  lift_motors.setPosition(0, degrees);
  hold_position = 0;
  target = 0;
  profile_setpoint = 0;
  resetPID(0);
}

bool CascadeLift::isAtTarget() {
  return arrived;
}

bool CascadeLift::waitUntilAtTarget(double timeout_msec) {
  double start_time = Brain.timer(msec);
  while (!arrived && (Brain.timer(msec) - start_time) < timeout_msec) {
    wait(10, msec);
  }
  return arrived;
}

double CascadeLift::getPosition() {
  return lift_motors.position(degrees);
}

LiftMode CascadeLift::getMode() { return mode; }
double CascadeLift::getTarget() { return target; }

// ============================================================================
//  INTERNAL: PRESETS / LIMITS / PID / PROFILE
// ============================================================================

double CascadeLift::resolvePreset(LiftHeight h) {
  switch (h) {
    case LiftHeight::BOTTOM: return preset_bottom;
    case LiftHeight::LOW:    return preset_low;
    case LiftHeight::MEDIUM: return preset_medium;
    case LiftHeight::HIGH:   return preset_high;
    case LiftHeight::TOP:    return preset_top;
  }
  return preset_bottom;
}

double CascadeLift::clampToLimits(double value) {
  if (value < lift_min_pos) return lift_min_pos;
  if (value > lift_max_pos) return lift_max_pos;
  return value;
}

void CascadeLift::resetPID(double initial_error) {
  // Clears carried-over integral and derivative so a new move starts clean.
  pid_error = initial_error;
  pid_prev_error = initial_error;
  pid_integral = 0;
}

double CascadeLift::motionProfileStep(double dt_sec) {
  // Proper trapezoidal (or triangular for short moves) profile:
  //   - accelerate at max_accel toward cruise speed max_speed
  //   - cruise once at max_speed
  //   - decelerate so velocity reaches ~0 exactly at the target
  // The decel point uses v^2 = 2*a*d from the CURRENT profile velocity.
  double remaining = target - profile_setpoint;
  double dir = (remaining >= 0) ? 1.0 : -1.0;
  double dist = std::fabs(remaining);
  double accel = std::fabs(max_accel_deg_s2) + 1e-9;

  double v_cur = std::fabs(profile_velocity);

  // Braking distance from the current velocity to zero.
  double braking_dist = (v_cur * v_cur) / (2.0 * accel);

  double desired_speed;
  if (dist <= braking_dist) {
    // Need to slow down: target speed for this remaining distance.
    desired_speed = std::sqrt(2.0 * accel * dist);
  } else if (v_cur < max_speed_deg_s) {
    // Still accelerating toward cruise.
    desired_speed = v_cur + accel * dt_sec;
    if (desired_speed > max_speed_deg_s) desired_speed = max_speed_deg_s;
  } else {
    // Cruising.
    desired_speed = max_speed_deg_s;
  }

  profile_velocity = dir * desired_speed;

  // Advance the setpoint, capped so we never overshoot the target.
  double step = profile_velocity * dt_sec;
  if (std::fabs(step) >= dist) {
    profile_setpoint = target;
    profile_velocity = 0;
  } else {
    profile_setpoint += step;
  }

  // Profile is done once we have reached the target.
  if (std::fabs(target - profile_setpoint) < lift_tolerance) {
    profiling = false;
    profile_velocity = 0;
  }
  return profile_setpoint;
}

// ============================================================================
//  CONTROL LOOP (background task, 10 msec)
// ============================================================================

void CascadeLift::controlLoop() {
  double last_time = Brain.timer(msec);

  while (true) {
    double now = Brain.timer(msec);
    double dt = (now - last_time) / 1000.0; // seconds
    if (dt <= 0) dt = 0.01;
    last_time = now;

    double output = 0;          // volts to send
    double current_pos = getPosition();

    switch (mode) {
      case LiftMode::MANUAL: {
        // Manual always wins. Map -100..100 to -12..+12 V, with feedforward
        // and soft limits applied AFTER so the stops are never driven past.
        double ff = lift_kG;
        // Taper feedforward near the bottom so the lift rests on the stop.
        if (current_pos <= lift_min_pos + 2.0) {
          ff *= std::fmax(0.0, (current_pos - lift_min_pos) / 2.0);
        }
        output = (manual_power / 100.0) * lift_max_voltage + ff;
        // Soft limits: do not push past the stops. At a limit, allow only the
        // feedforward (hold), never the opposing drive.
        if (current_pos <= lift_min_pos && output < ff) output = ff;
        if (current_pos >= lift_max_pos && output > ff) output = ff;
        break;
      }

      case LiftMode::HOMING:
        // Homing drives the motor itself in home(); loop just idles.
        output = 0;
        break;

      case LiftMode::POSITION:
      case LiftMode::HOLD: {
        double setpoint;
        if (mode == LiftMode::POSITION && profiling) {
          setpoint = motionProfileStep(dt);
        } else {
          setpoint = target;
          profiling = false;
        }

        if (pid_enabled) {
          // Self-contained PID (resets cleanly because we track state here).
          pid_error = setpoint - current_pos;
          pid_integral += pid_error * dt;
          // Anti-windup: cap integral contribution to the output limit.
          double integral_max = (std::fabs(lift_ki) > 1e-6)
              ? (lift_max_voltage / std::fabs(lift_ki)) : 0;
          if (integral_max > 0) {
            if (pid_integral * lift_ki >  integral_max) pid_integral =  integral_max / lift_ki;
            if (pid_integral * lift_ki < -integral_max) pid_integral = -integral_max / lift_ki;
          }
          // Clear integral on sign change (overshoot) for stability.
          if ((pid_integral > 0 && pid_error < 0) ||
              (pid_integral < 0 && pid_error > 0)) {
            pid_integral = 0;
          }
          double derivative = (pid_error - pid_prev_error) / dt;
          pid_prev_error = pid_error;

          output = lift_kp * pid_error
                 + lift_ki * pid_integral
                 + lift_kd * derivative;
        } else {
          // PID disabled: just feedforward.
          output = lift_kG;
        }

        // Gravity feedforward: a cascade lift load is ~constant, so a flat
        // upward voltage holds it. Reduce feedforward near the bottom.
        double ff = lift_kG;
        if (current_pos <= lift_min_pos + 2.0) {
          ff *= std::fmax(0.0, (current_pos - lift_min_pos) / 2.0);
        }
        output += ff;

        // Soft limits: do not drive past the stops.
        if (current_pos <= lift_min_pos && output < ff) output = ff;
        if (current_pos >= lift_max_pos && output > ff) output = ff;

        // Arrival detection for isAtTarget().
        // arrival_timer holds the last time we were OUT of tolerance, so
        // (now - arrival_timer) is how long we have been continuously in tol.
        bool profile_done = (mode != LiftMode::POSITION) || !profiling;
        bool in_tolerance = profile_done &&
                            (std::fabs(pid_error) <= lift_tolerance);
        if (in_tolerance) {
          if (now - arrival_timer >= lift_settle_ms) arrived = true;
          // (do not reset arrival_timer while in tolerance)
        } else {
          arrived = false;
          arrival_timer = now; // just left tolerance: restart the settle clock
        }
        break;
      }

      case LiftMode::IDLE:
      default:
        output = lift_kG; // hold against gravity
        break;
    }

    // Final output clamp.
    if (output >  lift_max_voltage) output =  lift_max_voltage;
    if (output < -lift_max_voltage) output = -lift_max_voltage;

    if (mode != LiftMode::HOMING) {
      lift_motors.spin(fwd, output, volt);
    }

    wait(10, msec);
  }
}
