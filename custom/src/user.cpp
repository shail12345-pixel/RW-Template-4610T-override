#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"
#include "../custom/include/lift.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  // NOTE: arm_motor (PORT16/17) is now owned by the CascadeLift controller.
  // The old redGoalRush() routine (case 3) directly commands arm_motor and
  // runs armPIDLoop(), which would fight the lift controller. Do NOT select
  // case 3 until you have rewritten its arm sections to use the lift API:
  //   lift.setHeight(LiftHeight::HIGH); lift.waitUntilAtTarget();
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      redGoalRush();
      break;
    case 4:
      break; 
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// --- Lift driver-control edge detection (previous-button states) ---
bool lift_up_prev = false, lift_down_prev = false;
bool lift_low_prev = false, lift_med_prev = false;
bool lift_high_prev = false, lift_top_prev = false;
bool lift_home_prev = false;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;

  // Start the lift background control task (idempotent). It also runs during
  // autonomous because we call lift.start() in runPreAutonomous().
  lift.start();
  // Re-engage a hold at the current position so the lift doesn't drop.
  lift.stopAndHold();

  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // ---- DRIVE (default tank drive) ----
    driveChassis(ch3 * 0.12, ch2 * 0.12);

    // ---- LIFT: MANUAL OVERRIDE (always highest priority) ----
    // R1 = up, R2 = down. Stick value scaled to -100..100.
    if (r1 || r2) {
      // R1 up is positive, R2 down is negative. Both held cancels out.
      double power = (r1 ? 100 : 0) + (r2 ? -100 : 0);
      lift.manual(power);
    } else {
      // No manual buttons: release override -> hold wherever it is.
      lift.manual(0);
    }

    // ---- LIFT: PRESETS (rising-edge triggered so held buttons don't spam) ----
    // Remap these buttons to whatever your drivers prefer.
    if (button_up_arrow && !lift_high_prev) lift.setHeight(LiftHeight::HIGH);
    lift_high_prev = button_up_arrow;

    if (button_down_arrow && !lift_low_prev) lift.setHeight(LiftHeight::LOW);
    lift_low_prev = button_down_arrow;

    if (button_y && !lift_top_prev) lift.setHeight(LiftHeight::TOP);
    lift_top_prev = button_y;

    if (button_a && !lift_med_prev) lift.setHeight(LiftHeight::MEDIUM);
    lift_med_prev = button_a;

    if (button_x && !lift_home_prev) lift.setHeight(LiftHeight::BOTTOM);
    lift_home_prev = button_x;

    wait(10, msec);
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }

  // ---- CASCADE LIFT ----
  // Start the background control task so the lift API works in auton AND driver.
  lift.start();

  // OPTIONAL: auto-home the lift at startup. Disabled by default because
  // homing moves the lift, which can be unsafe on a crowded field. Uncomment
  // to home (drive down until stall, then zero the encoder):
  // lift.home(3.0, 2000);
  // lift.setHeight(LiftHeight::BOTTOM);
}