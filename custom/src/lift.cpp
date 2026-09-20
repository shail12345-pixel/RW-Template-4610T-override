#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <thread>
#include <string>



#include "../include/autonomous.h"
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
void liftToAngle(double targetAngle, double maxSpeed = 12) {

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



void liftTo(double height, double maxSpeed = 12){
  liftToAngle(inToDeg(height),maxSpeed);
  

}

double alliance = 3;
double neutral = 6;
double midfield = 12;

double cup = 7;
double pin = 3.25;

void liftTo(bool pinInClaw, bool cupInClaw, bool pinPresent, double cupCount, const char* goal, double maxSpeed = 12,double buffer = 2){
    double goalHeight = 0;
    if(strcmp(goal,"alliance")){
      goalHeight = alliance;
    }else if(strcmp(goal,"neutral")){
      goalHeight = neutral;
    }else if(strcmp(goal,"midfield")){
      goalHeight = midfield;
    }
    double target = goalHeight + (cup * cupCount);

    
    if(pinPresent){
      target = target + pin;


    if(cupInClaw){
      target = target + cup-1;
    }else if(pinInClaw){
      target = target+pin-1;
    }

    
    target = target + buffer -5;


    liftTo(target,maxSpeed);
  }
}

void liftToState(const char* position, double maxSpeed = 12,double buffer = 0){
  double target=0;
  if(position=="vertical"){
    target = 0;
  }else if(position=="intake"){
    target = 3;
  }
  target = target+buffer;

  liftTo(target,maxSpeed);
}



void liftPlusHeight(double height, double maxSpeed = 12) {
    double currentHeight = getLiftHeight();
    double targetHeight = currentHeight + height;

    liftTo(targetHeight, maxSpeed);
}


// 