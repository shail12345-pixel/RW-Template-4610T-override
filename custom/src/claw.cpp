#include "vex.h"
#include "pid.h"
#include "../include/autonomous.h"



namespace {
constexpr double pinDetectionDistanceMm = 70.0;
constexpr double clawClosedPositionDegrees = 90.0;
}

void distanceWrit() {
    claw.setPosition(0, vex::rotationUnits::deg);
    claw.setStopping(vex::brakeType::hold);
    bool isClosed = false;

    while (true) {
        double distance = front_sensor.objectDistance(vex::distanceUnits::mm);
        bool pinDetected = (distance > 0 && distance <= pinDetectionDistanceMm);

        if (pinDetected != isClosed) {
            isClosed = pinDetected;
            claw.spinToPosition(
                isClosed ? clawClosedPositionDegrees : 0.0,
                vex::rotationUnits::deg,
                50, // velocity in %
                vex::velocityUnits::pct,
                false
            );
        }
        vex::wait(20, vex::msec);
    }
}
