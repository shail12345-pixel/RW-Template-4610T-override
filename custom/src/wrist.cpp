#include "vex.h"
#include "pid.h"

using namespace vex;

PID wristPID(0.75, 0.0, 6.0);

double clamp(double value, double min, double max) {
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}

void moveWristTo(double target) {
    target = target * 4.0;
    const double WRIST_MIN = -278;
    const double WRIST_MAX = 136;
    const double LIMIT_MARGIN = 8.0;

    
    target = clamp(
        target,
        WRIST_MIN + LIMIT_MARGIN,
        WRIST_MAX - LIMIT_MARGIN
    );

    wristPID.setTarget(target);

    while (!wristPID.targetArrived()) {

        double wristAngle =
            (wristPosition.position(deg));

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