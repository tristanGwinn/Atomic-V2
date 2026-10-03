#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "controllers/pid.hpp"

#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/adi.hpp"

#include "hardware/Motor/MotorGroup.hpp"

// this could probably be simplified by making an abstract class for a solenoid subsystem,
// but this works so its not important. (simplifying would likely reduce compile time)
class ClawSubsystem : public Subsystem {
    private:
        pros::adi::DigitalOut solenoid;
        pros::v5::Distance distance;

        // maybe put this in config, idk
        // currently, this is an untested placeholder
        Length threshold = 2.0_in;

        bool lastValue = false;
        bool isCupDetected = false;

        bool primedToClamp = false;
        bool isClawClamped = false;

        void allowClamp(const bool value) { primedToClamp = value; }

        void setLevel(const bool value) {
            solenoid.set_value(value);
            lastValue = value;
        }

    public:
        
        explicit ClawSubsystem(pros::adi::DigitalOut &solenoid, pros::v5::Distance &dist)
        : solenoid(solenoid), distance(dist) {
            // prolly dont need anything here
        }

        void periodic() override {
            // auto distance_value = distance.get_distance();

            // check if cup is in range
            (threshold >= from_mm(distance.get_distance())) ?
            isCupDetected = true : isCupDetected = false;
        }


        bool getCupStatus() { printf("Is a cup detected? %s", isCupDetected ? "Yes.\n" : "No.\n"); printf("cup distance sensor: %d\n", distance.get_distance()); return isCupDetected; }

        bool getClampStatus() { return isClawClamped; }
        
        bool isPrimed() { return primedToClamp; }


        RunCommand *primeClampCommand() {
            return new RunCommand([this]() { this->allowClamp(true); }, {this});
        }

        RunCommand *tryClampCommand() {
            return new RunCommand(
                [this] ()
                {
                    if (primedToClamp) this->setLevel(true);
                    else this->setLevel(false);
                },
                {this}
            );
        }

        RunCommand *levelCommand(bool value) {
            return new RunCommand([this, value]() {
                    this->setLevel(value);
                    this->isClawClamped = value;
                }, 
                {this}
            );
        }

        [[nodiscard]] bool getLastValue() const { return lastValue; }

        ~ClawSubsystem() override = default;
};
