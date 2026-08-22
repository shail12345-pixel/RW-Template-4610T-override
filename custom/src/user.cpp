#include "vex.h"
#include "motor-control.h"
#include "../include/autonomous.h"
#include "../include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

// =============================================================================
// Driver Control
// =============================================================================







void intakeManager(){
  // rian from 4610R is the goat
  while(1){
    if(controller_1.ButtonL1.pressing()){
        intake.spin(fwd,12,volt);
        claw.spin(fwd,12,volt);
    }else if(controller_1.ButtonL2.pressing()/*&& liftDown()*/){
      intake.spin(reverse,12,volt);
      claw.spin(reverse,12,volt);
   /* }else if(controller_1.ButtonR2.pressing()&& !liftDown()){
        claw.spin(reverse,12,volt);
        wait(200,msec);
        // cascadePlusHeight(2);
        */
    }else{
      intake.stop(coast);
      claw.stop(hold);
    }
    
  }
}

void liftManager(){
  while(1){
    if(controller_1.ButtonR1.pressing()&&liftHeight.angle(deg)<110){
        lift.spin(fwd,12,volt);
    }else if(controller_1.ButtonR2.pressing()&&liftHeight.angle(deg)+1>0){
      lift.spin(reverse,12,volt);
    }else{
      lift.stop(hold);
    }
    
  }
}

void stressTest(){
    while(1){
        controller_1.Screen.clearScreen();
        controller_1.Screen.setCursor(1,1);
        controller_1.Screen.print("%f",intake.temperature(celsius));
        controller_1.Screen.setCursor(2,1);
        controller_1.Screen.print("%f",intake.velocity(pct));
    }
}

// =============================================================================
// RW Stuff
// =============================================================================

void runAutonomous() {
  int auton_selected = 2;
  switch(auton_selected) {
    case 1:
      tunePid();
      break;
    case 2:
      exampleAuton();
      break;  
    case 3:
      autonOne();
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
  liftHeight.setPosition(0,deg);
  thread s(stressTest);
  thread l(liftManager);
  thread i(intakeManager);
  stopChassis(coast);
  heading_correction = false;

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

      // default split arcade drive while not running the full controlNormalized thread
  //    if (!drive_code) {
        // split arcade drive (Axis3 fwd/back, Axis1 turn)
      //controlNormalized();
      double forwardSpeed = ch3 * 0.12;
      double turn = ch1 * 0.12;
      driveChassis(forwardSpeed + turn, forwardSpeed - turn);
      //}

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

}