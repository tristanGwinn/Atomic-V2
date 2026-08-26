#pragma once

#include "atomic/exitCondition.hpp"
#include "atomic/pid.hpp"
#include "atomic/ramsete/ramseteController.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "units/Pose.hpp"
#include <functional>

extern const atomic::PID angular_pid;
extern const atomic::PID lateral_pid;

extern const atomic::RamseteController ramsete_controller;

extern const std::function<units::Pose()> pose_getter;

extern atomic::MotorGroup left_motors;
extern atomic::MotorGroup right_motors;

extern const atomic::ExitConditionGroup<AngleRange> angular_exit_conditions;
extern const atomic::ExitConditionGroup<Length> lateral_exit_conditions;

extern const Length track_width;
extern const Length wheel_diameter;
extern const AngularVelocity max_rpm;

extern const Number drift_compensation;

extern const Number angular_slew;
extern const Number lateral_slew;