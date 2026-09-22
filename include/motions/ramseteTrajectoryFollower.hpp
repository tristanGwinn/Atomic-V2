#pragma once

#include "config.hpp"

struct RameseteFollowSettings {
        RamseteController ramseteController;
        Length trackWidth;
        std::function<units::Pose()> poseGetter;
        MotorGroup& leftMotors;
        MotorGroup& rightMotors;
};

void followTrajectory(const Trajectory& t, RameseteFollowSettings settings);