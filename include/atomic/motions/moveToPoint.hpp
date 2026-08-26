#pragma once

#include "atomic/motionConfig.hpp"
#include <functional>

namespace atomic {

struct MoveToPointParams {
        bool reversed = false;
        Number maxLateralSpeed = 1;
        Number minLateralSpeed = 0;
        Number maxAngularSpeed = 1;
        Number lateralSlew = lateral_slew;
        Number angularSlew = angular_slew;
        Length earlyExitRange = 0_in;
};

struct MoveToPointSettings {
        PID angularPID = angular_pid;
        PID lateralPID = lateral_pid;
        ExitConditionGroup<Length> exitConditions = lateral_exit_conditions;
        std::function<units::Pose()> poseGetter = pose_getter;
        atomic::MotorGroup& leftMotors = left_motors;
        atomic::MotorGroup& rightMotors = right_motors;
};

void moveToPoint(units::V2Position target, Time timeout, MoveToPointParams params, MoveToPointSettings settings);

}; // namespace atomic