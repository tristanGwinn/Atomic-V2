#pragma once

#include "config.hpp"

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "controllers/pid.hpp"
#include <iostream>

/**
 * Turn on point code
 */
class Rotate : public Command {
private:
    DriveSubsystem *drivetrain;
    double static_voltage;
    bool finish{true};

    std::function<Angle()> updateAngle;

public:
    Rotate(DriveSubsystem *drivetrain, const Angle angle, const bool flip, const double static_voltage = 0.0,
           const bool finish = true)
        : drivetrain(drivetrain),
          static_voltage(static_voltage),
          finish(finish), updateAngle([flip, angle]() { return (flip ? -1.0f : 1.0f) * angle; }) {
    }

    Rotate(DriveSubsystem *drivetrain, const std::function<Angle()> &updateAngle, const bool flip,
           const double static_voltage = 0.0,
           const bool finish = true)
        : drivetrain(drivetrain),
          static_voltage(static_voltage),
          finish(finish), updateAngle([flip, updateAngle] () { return (flip ? -1.0f : 1.0f) * updateAngle(); }) {
    }

    void initialize() override {
        config::angular_pid.reset();
        config::angular_pid.setTarget(updateAngle().internal());
    }

    void execute() override {
        const auto output = units::clamp(config::angular_pid.update(drivetrain->getPose().orientation.internal()), -1.0, 1.0);
        drivetrain->setPct(static_voltage - output, static_voltage + output);
    }

    bool isFinished() override {
        return finish && this->drivetrain->getPose().orientation - config::angular_pid.getTarget()*rad <
               config::angle_tolerance;
    }

    std::vector<Subsystem *> getRequirements() override {
        return {drivetrain};
    }

    ~Rotate() override = default;
};
