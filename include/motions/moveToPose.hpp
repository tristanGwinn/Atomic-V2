#pragma once

#include "config.hpp"
#include <functional>



struct MoveToPoseParams {
        bool reversed = false;
        Number driftCompensation = 2.0;
        Number lead = 0.6;
        Number maxLateralSpeed = 1;
        Number minLateralSpeed = 0;
        Number maxAngularSpeed = 1;
        Number lateralSlew = 1.0;
        Number angularSlew = 1.0;
        Length earlyExitRange = 0_in;
};

struct MoveToPoseSettings {
        PID angularPID;
        PID lateralPID;
        ExitConditionGroup<Length> lateralExitConditions;
        ExitConditionGroup<AngleRange> angularExitConditions;
        std::function<units::Pose()> poseGetter = pose_getter;
        MotorGroup& leftMotors = left_motors;
        MotorGroup& rightMotors = right_motors;
};

void moveToPose(units::Pose target, Time timeout, MoveToPoseParams params, MoveToPoseSettings settings);