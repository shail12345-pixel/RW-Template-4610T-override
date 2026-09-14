#include "vex.h"
#include "pid.h"
#include "../include/autonomous.h"

using namespace vex;

PID wristPID(0.75, 0.0, 6.0);
// wristPID.setSmallBigErrorTolerance(2,6);
// wristPID.setSmallBigErrorDuration(50,250);
// wristPID.setDerivativeTolerance(10);



void moveWristTo(double target) {
    target = target;

    const double WRIST_MIN = -360;
    const double WRIST_MAX = 136;
    const double LIMIT_MARGIN = 8.0;
    const int TIMEOUT_MS = 600;

    target = clamp(
        target,
        WRIST_MIN + LIMIT_MARGIN,
        WRIST_MAX - LIMIT_MARGIN
    );

    wristPID.setTarget(target);

    timer wristTimer;
    wristTimer.clear();

    while (!wristPID.targetArrived() &&
           wristTimer.time(msec) < TIMEOUT_MS) {

        double wristAngle = wristPosition.position(deg);

        double output = wristPID.update(wristAngle);

        if (wristAngle <= WRIST_MIN && output < 0) {
            output = 0;
        }

        if (wristAngle >= WRIST_MAX && output > 0) {
            output = 0;
        }

        output = clamp(output, -100.0, 100.0);

        wrist.spin(fwd, output, pct);

        wait(10, msec);
    }

    wrist.stop(hold);
}