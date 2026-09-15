#pragma once

#include "config.hpp"

struct RameseteFollowSettings {
        RamseteController ramseteController = ramsete_controller;
        Length trackWidth = track_width;
        std::function<units::Pose()> poseGetter = pose_getter;
        MotorGroup& leftMotors = left_motors;
        MotorGroup& rightMotors = right_motors;
};

void followTrajectory(const Trajectory& t, RameseteFollowSettings settings);