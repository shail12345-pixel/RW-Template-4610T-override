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
    double current = liftHeight.position(deg);
    if(degToIn(targetAngle>42.25))targetAngle=inToDeg(42);
    double error = targetAngle - current;
    double previousError = error;
    double integral = 0;
    double derivative = 0;
    double speed = 0;

    // Use absolute value so the loop works for moving both up and down
    // Require error to be within threshold for a short stable period before exiting
    const double threshold = 1.5;
    const int stableMs = 200; // must be within threshold for this many milliseconds
    const int loopDelayMs = 10; // matches wait(10, msec)
    int stableCount = 0;
    int requiredStableCount = stableMs / loopDelayMs;

    while (true) {
      if (fabs(error) <= threshold) {
        stableCount++;
      } else {
        stableCount = 0;
      }

      if (stableCount >= requiredStableCount) break;
        // 1. Accumulate integral
        integral += error;

        // Reset integral if target is crossed to prevent windup
        if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
            integral = 0;
        }

        // 2. Calculate derivative
        derivative = error - previousError;

        // 3. Compute output speed
        speed = (error * kP) + (integral * kI) + (derivative * kD);

        // Cap speed to maxSpeed constraint (voltage scaled)
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        // 4. Apply output to motors (.12 converts PCT to Volts)
        lift.spin(fwd, speed, volt);
    //  std::cout << "\n" << liftHeight.position(deg) << ", " << error << ", " << speed << "\n";

        // 5. Update values for next loop iteration
        previousError = error;
        current = liftHeight.position(deg);
        error = targetAngle - current;

        wait(10, msec);
    }

    // Stop motors once target threshold is reached
    lift.stop(hold);
    wait(50,msec);
    std::cout << "Done";
    
  
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
  
  std::cout << "\n True height (in deg): " << getLiftHeight << "/n";
}

double alliance = 3;
double neutral = 6;
double midfield = 12;

double cup = 7;
double pin = 3.25;

void liftTo(bool pinInClaw, bool cupInClaw, bool pinPresent, double cupCount, const char* goal, double maxSpeed = 12,double buffer = 2){
    double goalHeight = 0;
    if(goal=="alliance"){
      goalHeight = alliance;
    }else if(goal=="neutral"){
      goalHeight = neutral;
    }else if(goal =="midfield"){
      goalHeight = midfield;
    }
    double target = goalHeight + (cup * cupCount);
    std::cout << "/n" << cupCount * cup << "\n";
    
    if(pinPresent){
      target = target + pin;
    }
    std::cout << "/n" << target << "\n";

    if(cupInClaw){
      target = target + cup-1;
    }else if(pinInClaw){
      target = target+pin-1;
    }
    std::cout << "/n" << target << "  \n";
    
    target = target + buffer -5;
    std::cout << "/n" << target << "\n";

    liftTo(target,maxSpeed);
}

void liftToState(const char* position, double maxSpeed = 12,double buffer = 0){
  double target;
  if(position=="vertical"){
    target = 0;
  }else if(position=="intake"){
    target = 4;
  }
  target = target+buffer;
  liftTo(target,maxSpeed);
}

void liftPlusHeight(double height, double maxSpeed = 12){
  double target = height + degToIn(liftHeight.position(deg));
  liftTo(target,maxSpeed); 
}


