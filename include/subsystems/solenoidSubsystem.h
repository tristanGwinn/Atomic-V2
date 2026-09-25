#pragma once

#include <utility>
#include <vector>

#include "pros/adi.hpp"

#include "command/runCommand.h"
#include "command/subsystem.h"

// May want to make this into an abstract class or smth, idk
class SolenoidSubsystem : public Subsystem {
    std::vector<pros::adi::DigitalOut> solenoids;

    bool lastValue = false;

public:
    explicit SolenoidSubsystem(pros::adi::DigitalOut solenoid) : solenoids({std::move(solenoid)}) {
    }

    explicit SolenoidSubsystem(const std::vector<pros::adi::DigitalOut> &solenoids) : solenoids(solenoids) {
    }

    void periodic() override {
        // no - op
    }

    void setLevel(const bool value) {
        for (auto &&solenoid: solenoids) {
            solenoid.set_value(value);
        }
        lastValue = value;
    }

    RunCommand *levelCommand(bool value) {
        return new RunCommand([this, value]() { this->setLevel(value); }, {this});
    }

    [[nodiscard]] bool getLastValue() const { return lastValue; }

    ~SolenoidSubsystem() override = default;
};
