#pragma once

#include "atomic/motionConfig.hpp"

namespace atomic {

struct RameseteFollowSettings {
        RamseteController ramseteController = ramsete_controller;
        Length trackWidth = track_width;
        std::function<units::Pose()> poseGetter = pose_getter;
        atomic::MotorGroup& leftMotors = left_motors;
        atomic::MotorGroup& rightMotors = right_motors;
};

void followTrajectory(const atomic::Trajectory& t, RameseteFollowSettings settings);

}   // namespace atomic