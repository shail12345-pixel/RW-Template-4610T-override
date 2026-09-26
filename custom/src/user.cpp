#include "vex.h"
#include "motor-control.h"
#include "../include/autonomous.h"
#include "../include/robot-config.h"
#include "../include/lift.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>
#include <string>
#include "pid.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include "../include/wrist.h"




// Modify autonomous, driver, or pre-auton code below

// =============================================================================
// Driver Control
// =============================================================================



bool liftOverride = false;
bool wristL1Handled = false;




void intakeManager(){
  // rian from 4610R is the goat
  while(1){
    if(controller_1.ButtonR1.pressing()){
      intake.spin(fwd,6,volt);
      claw.spin(fwd,12,volt);
    }else if(controller_1.ButtonL1.pressing()){
      liftOverride = false;
          claw.spin(fwd,12,volt);
          intake.spin(fwd,12,volt);
          
        
    }else if(controller_1.ButtonL2.pressing() && getLiftHeight()<3){
      liftOverride = false;
      intake.spin(reverse,12,volt);
      claw.spin(reverse,12,volt);
      
    }else if(controller_1.ButtonL2.pressing()){
        liftOverride = true;

        claw.spin(reverse,12,volt);
        
        liftPlusHeight(10);

        if(getLiftHeight()<36)moveWristTo(110);

        
        claw.stop();

        // moveWristTo(20);
        


        liftOverride = false;
    }else if(controller_1.ButtonX.pressing()){
      release();
      while(controller_1.ButtonX.pressing())wait(10,msec);
    }else{
      liftOverride = false;
      intake.stop(coast);
      claw.stop(coast);
    }
    wait(10,msec);
  }
}

void liftManager(){
  //i cant pt down
  while(1){
    if(controller_1.ButtonL1.pressing()){
      
      liftToAngle(5,12,12);
      lift.spin(reverse,6,volt);


    }else if(controller_1.ButtonR1.pressing()){
      printText("liftStarted");
         lift.spin(fwd,12,volt);
    }else if(controller_1.ButtonR2.pressing()){
      lift.spin(reverse,12,volt);
    }else if(controller_1.ButtonUp.pressing()){
      lift.spin(reverse,12,volt);
      wait(1,sec);
      liftHeight.setPosition(0,deg);
      liftToAngle(5,12,12);

    }else if(!liftOverride){
      lift.stop(hold);
    }
    wait(10,msec);
  }
}


// void wristManager(){
//   while(1){

//     if(controller_1.ButtonL1.pressing()){
//       printText("started)");
//       if(!((fabs(wristPosition.position(deg)-(4.0*85.0))<=2))){
//               printText("in)");
//         while(getLiftHeight()<3)wait(10,msec);
//         moveWristTo(-90);
//       }
//             printText("out)");
//     }else if(controller_1.ButtonR1.pressing()){
//       printText("wristStarted");
//       moveWristTo(5.0); 
//       if(getLiftHeight()>38)moveWristTo(34.0);
//     }
//     wait(10,msec);
//   }
// }
//i kant put down the cup
void wristManager(){
  while(1){
    if(controller_1.ButtonL1.pressing()){
      moveWristTo(-85);
    }else if(controller_1.ButtonR1.pressing()){
      printText("wrist triggered");
      moveWristTo(5);
    }
    wait(10,msec);
  }
}

void conDisplay(){
    while(1){
        controller_1.Screen.clearScreen();
        controller_1.Screen.setCursor(1,1);
        controller_1.Screen.print("%f",wristPosition.position(deg)/4);
        controller_1.Screen.setCursor(2,1);
        controller_1.Screen.print("%f",getLiftHeight());
    }
}

// void print(){
//     while(1){
//       if(controller_1.ButtonX.pressing()){
//         std::cout << horizontal_tracker.position(degrees);

//       }
//     }
// }


// void testButton(){
//   while(1){
//     if(controller_1.ButtonA.pressing()){

//       moveWristTo(5);
//     }
//   }
// }


void clawReverseOrForward(){
  while(1){
    if(controller_1.ButtonA.pressing()){
       claw.spin(fwd,12,volt);
    }
    else if(controller_1.ButtonB.pressing()){
        claw.spin(reverse,12,volt);
    }
  }
}

void intakeReverseOrForward(){
  while(1){
    if(controller_1.ButtonRight.pressing()){
       intake.spin(fwd,12,volt);
    }
    else if(controller_1.ButtonLeft.pressing()){
      intake.spin(reverse,12,volt);
    }
  }
}

float deadband(float input, float width){
  if (std::fabs(input)<width){
    return(0);
  }
  return(input);
}


void controlNormalized() {

    double forward = deadband(controller_1.Axis3.value(), 10);
    double turn = deadband(controller_1.Axis1.value(), 10);
    double normal = fabs(turn) + fabs(forward);
    double fResult = 0, tResult = 0;
    if (normal != 0) {
      fResult = forward / normal, tResult = turn / normal;
      if (fabs(turn) > fabs(forward)) {
        fResult *= fabs(turn);
        tResult *= fabs(turn);
      } else {
        fResult *= fabs(forward);
        tResult *= fabs(forward);
      }
    }
    left_chassis.spin(fwd, (fResult + tResult) * 0.12, volt);
    right_chassis.spin(fwd, (fResult - tResult) * 0.12, volt);
    wait(10, msec);

}





// =============================================================================
// RW Stuff
// =============================================================================

void runAutonomous() {
  int auton_selected = 2;
  thread a(brainD);
  switch(auton_selected) {
    case 1:
      qual1();
      break;
    case 2:
      NA_1pin();
      break;  
    case 3:
      A_1pin();
      break;
    case 4: 
      skills(); 
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

  wristPosition.setPosition(-310,deg);
  liftHeight.setPosition(0,deg);

  lift.setStopping(hold);
  intake.setStopping(coast);
  claw.setStopping(coast);
  wrist.setStopping(hold);
  thread s(conDisplay);
  thread w(wristManager);
 thread l(liftManager);
 thread i(intakeManager);


  thread t(clawReverseOrForward);
  thread p(intakeReverseOrForward);

  stopChassis(coast);
  heading_correction = false;

  // wait(3,sec);

  // controller_1.rumble("---");
  

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

      
      controlNormalized();

      wait(10, msec);
    }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

   wristPosition.setPosition(-280,deg);
  liftHeight.setPosition(0,deg);

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

  liftHeight.setPosition(0,deg);

}