#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <thread>
#include <string>



#include "../include/autonomous.h"
#include "../include/lift.h"
#include "motor-control.h"

double kP = .75;
double kI = 0;
double kD = 2;

double degToIn(double degrees){
  return 50*sin((M_PI*degrees)/360);
}

double inToDeg(double inches){
  return (360/M_PI)*asin(inches/50);
  //f(x)=(360/\pi )*\arcsin (x/50)
}

double getLiftHeight(){
  return degToIn(liftHeight.position(deg));
}
void liftToAngle(double targetAngle, double maxSpeed, double minSpeed) {

  targetAngle = clamp(targetAngle, 0, inToDeg(42));

  double current = liftHeight.position(deg);
  if(degToIn(targetAngle) > 42.25) targetAngle = inToDeg(42);

  double error = targetAngle - current;
  double previousError = error;
  double integral = 0;
  double derivative = 0;
  double speed = 0;

  const double threshold = 1.5;
  const int stableMs = 200;
  const int loopDelayMs = 10;

  const int timeoutMs = 1000;  // 1 second timeout
  int elapsedMs = 0;

  int stableCount = 0;
  int requiredStableCount = stableMs / loopDelayMs;

  while (true) {

    // Check timeout
    if (elapsedMs >= timeoutMs) {
      break;
    }

    if (fabs(error) <= threshold) {
      stableCount++;
    } else {
      stableCount = 0;
    }

    if (stableCount >= requiredStableCount) break;

    // 1. Accumulate integral
    integral += error;

    // Reset integral if target is crossed
    if ((error > 0 && previousError < 0) ||
        (error < 0 && previousError > 0)) {
      integral = 0;
    }

    // 2. Calculate derivative
    derivative = error - previousError;

    // 3. Compute output speed
    speed = (error * kP) +
            (integral * kI) +
            (derivative * kD);

    // Cap speed
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    // 4. Apply output
    lift.spin(fwd, speed, volt);

    // 5. Update values
    previousError = error;
    current = liftHeight.position(deg);
    error = targetAngle - current;

    vex::wait(loopDelayMs, msec);
    elapsedMs += loopDelayMs;
  }

  lift.stop(hold);
  vex::wait(50, msec);


}



double heightConsideredDown = .5;

bool liftDown(){
  if(getLiftHeight() <= heightConsideredDown){
    return true;
  }else{
    return false;
  }
}



void liftTo(double height, double maxSpeed, double minSpeed){
  liftToAngle(inToDeg(height),maxSpeed);
  

}

double alliance = 3;
double neutral = 6;
double midfield = 12;

double cup = 7;
double pin = 3.25;

void liftTo(bool pinInClaw, bool cupInClaw, bool pinPresent, double cupCount, const char* goal, double maxSpeed, double minSpeed, double buffer){
    double goalHeight = 0;
    // strcmp returns 0 when the strings MATCH
    if(strcmp(goal,"alliance") == 0){
      goalHeight = alliance;
    }else if(strcmp(goal,"neutral") == 0){
      goalHeight = neutral;
    }else if(strcmp(goal,"midfield") == 0){
      goalHeight = midfield;
    }
    double target = goalHeight + (cup * cupCount);

    if(pinPresent){
      target = target + pin;
    }

    if(cupInClaw){
      target = target + cup-1;
    }else if(pinInClaw){
      target = target+pin-1;
    }

    target = target + buffer -5;

    liftTo(target,maxSpeed, minSpeed);
}

void liftToState(const char* position, double maxSpeed, double minSpeed, double buffer){
  double target=0;
  if(strcmp(position,"vertical") == 0){
    target = 0;
  }else if(strcmp(position,"intake") == 0){
    target = 0.0;
    
  }
  target = target+buffer;

  liftTo(target,maxSpeed, minSpeed);
}



void liftPlusHeight(double height, double maxSpeed, double minSpeed) {
    double currentHeight = getLiftHeight();
    double targetHeight = currentHeight + height;

    liftTo(targetHeight, maxSpeed, minSpeed);
}

// =============================================================================
// Driver-control STAGES (non-blocking)
//
// Pressing the stage button sets a target and returns immediately. liftManager
// calls liftStageUpdate() every 10 ms to drive toward it with a PID whose state
// persists between ticks. When the lift arrives (or times out) the stage is
// released and the motor just holds where it is. Any manual lift input calls
// liftStageCancel(), so you can go up or down from a stage and it never pulls
// you back to the last selected one.
// =============================================================================

static bool   stageActive     = false;
static double stageTargetDeg  = 0;
static double stagePrevError  = 0;
static int    stageElapsedMs  = 0;
static int    stageStableMs   = 0;

static const int    STAGE_LOOP_MS    = 10;
static const int    STAGE_STABLE_MS  = 200;
static const int    STAGE_TIMEOUT_MS = 3000;
static const double STAGE_THRESHOLD  = 1.5;   // degrees

// Stage heights in inches, lowest to highest. Edit these to taste.
static const int NUM_LIFT_STAGES = 4;
double liftStageHeight(int i){
  switch(i){
    case 0:  return heightConsideredDown; // bottom
    case 1:  return alliance;
    case 2:  return neutral;
    default: return midfield;
  }
}

bool liftStageActive(){ return stageActive; }

void liftStageCancel(){ stageActive = false; }

void liftStageGoTo(double inches){
  inches = clamp(inches, 0, 42);
  stageTargetDeg = inToDeg(inches);
  stagePrevError = stageTargetDeg - liftHeight.position(deg);
  stageElapsedMs = 0;
  stageStableMs  = 0;
  stageActive    = true;
}

// Go to the next stage ABOVE wherever the lift currently is (wraps to the
// bottom stage from the top). Works from any height, including after manual moves.
void liftStageNext(){
  double h = getLiftHeight();
  for(int i = 0; i < NUM_LIFT_STAGES; i++){
    if(liftStageHeight(i) > h + 1.0){
      liftStageGoTo(liftStageHeight(i));
      return;
    }
  }
  liftStageGoTo(liftStageHeight(0));
}

// Call once per STAGE_LOOP_MS while no manual input is active.
void liftStageUpdate(){
  if(!stageActive) return;

  double error = stageTargetDeg - liftHeight.position(deg);

  if(fabs(error) <= STAGE_THRESHOLD) stageStableMs += STAGE_LOOP_MS;
  else stageStableMs = 0;

  stageElapsedMs += STAGE_LOOP_MS;

  if(stageStableMs >= STAGE_STABLE_MS || stageElapsedMs >= STAGE_TIMEOUT_MS){
    stageActive = false;
    lift.stop(hold);   // stay here; manual control is free from this point
    return;
  }

  double derivative = error - stagePrevError;
  double speed = (error * kP) + (derivative * kD);
  stagePrevError = error;

  if(speed > 12) speed = 12;
  if(speed < -12) speed = -12;

  lift.spin(fwd, speed, volt);
}
