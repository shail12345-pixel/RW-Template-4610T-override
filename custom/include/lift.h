#pragma once

extern double kP;
extern double kI;
extern double kD;

extern void liftToAngle(double targetAngle, double maxSpeed = 12, double minSpeed = 0);

extern double degToIn(double degrees);

extern double inToDeg(double inches);


extern double heightConsideredDown;

extern bool liftDown();



extern void liftTo(double height, double maxSpeed = 12, double minSpeed = 0);

extern double alliance;
extern double neutral;
extern double midfield;

extern double cup;
extern double pin;

extern void liftTo(bool pinInClaw, bool cupInClaw, bool pinPresent, double cupCount, const char* goal, double maxSpeed = 12, double minSpeed = 0, double buffer = 2);

extern void liftToState(const char* position, double maxSpeed = 12, double minSpeed = 0, double buffer = 0);

extern void liftPlusHeight(double height, double maxSpeed = 12, double minSpeed = 0);
