#pragma once

#include "config.hpp"
#include <functional>

struct MoveToPointParams {
        bool reversed = false;
        Number maxLateralSpeed = 1;
        Number minLateralSpeed = 0;
        Number maxAngularSpeed = 1;
        Number lateralSlew = 1.0;
        Number angularSlew = 1.0;
        Length earlyExitRange = 0_in;
};

struct MoveToPointSettings {
        PID angularPID;
        PID lateralPID;
        ExitConditionGroup<Length> exitConditions;
        std::function<units::Pose()> poseGetter;
        MotorGroup& leftMotors;
        MotorGroup& rightMotors;
};

void moveToPoint(units::V2Position target, Time timeout, MoveToPointParams params, MoveToPointSettings settings);