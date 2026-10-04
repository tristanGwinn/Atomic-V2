#pragma once

#include "command/command.h"
#include "command/runCommand.h"
#include "command/waitUntilCommand.h"

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

    public:
        
        explicit ClawSubsystem(pros::adi::DigitalOut &solenoid, pros::v5::Distance &dist)
        : solenoid(solenoid), distance(dist) {
            // prolly dont need anything here
        }

        void periodic() override {
            // auto distance_value = distance.get_distance();

            // check if cup is in range
            if (threshold >= from_mm(distance.get_distance())){
                // if (!isCupDetected) printf("Cup has been detected!\n");
                isCupDetected = true;

            }else{
                // if (isCupDetected) printf("Cup has moved from view :(\n");
                isCupDetected = false;
            }
        }

        // void allowClamp(const bool value) { primedToClamp = value; }

        void setLevel(const bool value) {
            if(isClawClamped != value) 
                printf("%s\n", (value) ? "The claw has been clamped!" : "The claw has been opened!");
            solenoid.set_value(value);
            lastValue = value;
        }


        bool getCupStatus() { return isCupDetected; }

        bool getClampStatus() { return isClawClamped; }
        
        bool isPrimed() { return primedToClamp; }


        /*FunctionalCommand *waitForCupCommand() {
            printf("Claw is waiting on cup ... ");
            return new FunctionalCommand(
                [this]() {}, [this]() {}, [this](bool _) {}, [this]() {return this->getCupStatus(); }, {this});
        }*/

        /*RunCommand *primeClampCommand() {
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
        }*/

        FunctionalCommand *clampWhenReadyCommand(){ 
            return new FunctionalCommand(
                [this]() { this->setLevel(false); printf("\nWaiting for cup ...\n"); }, 
                [this]() {},
                [this](bool _) { this->setLevel(true); printf("Cup detected, clamp activated!\n\n"); },
                [this]() { return getCupStatus(); },
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
