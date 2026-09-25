#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "controllers/pid.hpp"

#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/adi.hpp"

#include "hardware/Motor/MotorGroup.hpp"

// todo

// this could probably be simplified by making an abstract class for a solenoid subsystem,
// but this works so its not important. (simplifying would likely reduce compile time)
class ClawSubsystem : public Subsystem {
    private:
        pros::adi::DigitalOut solenoid;
        pros::v5::Distance distance;

        // maybe put this in config, idk
        // currently, this is an untested placeholder
        Length threshold = 1.5_in;

        bool lastValue = false;
        bool isCupDetected = false;

    public:
        
        explicit ClawSubsystem(pros::adi::DigitalOut &solenoid, pros::v5::Distance &dist)
        : solenoid(solenoid), distance(dist) {
            // prolly dont need anything here
        }
        
        void periodic() override {
            // check if cup is in range
            (to_mm(threshold) > distance.get_distance()) ?
            isCupDetected = true : isCupDetected = false;
        }

        void setLevel(const bool value) {
            solenoid.set_value(value);
            lastValue = value;
        }

        bool getCupStatus() { return isCupDetected; }

        RunCommand *levelCommand(bool value) {
            return new RunCommand([this, value]() { this->setLevel(value); }, {this});
        }

        [[nodiscard]] bool getLastValue() const { return lastValue; }

        ~ClawSubsystem() override = default;
};
