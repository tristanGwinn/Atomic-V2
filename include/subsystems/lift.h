#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "hardware/Motor/MotorGroup.hpp"

class LiftSubsystem : public Subsystem {
    private:
        MotorGroup motor;

        std::optional<double> pct;

        // other components here

    public:
        explicit LiftSubsystem(MotorGroup &motors) : motor(motors) {
            motor.setAngle(0_stDeg);    // tare output upon initialization
        }

        /**
         * This function ca
         */
        void periodic() override {
            // auto position = this->getPosition();
        }

        /**
         * Move the lift motors at a signed percentage of voltage
         */
        void setPct(const double pct) {
            this->motor.move(pct);
        }

        /**
         * this command was take from the annotated example provided in echo's documentation
         * I left the annotations in for future reference in writing other commnads
         */
        RunCommand* pctCommand(const double pct) {
            // Create a new RunCommand
            // The lambda body is called at every update, in this case setting the lift percentage
            return new RunCommand(
                [this, pct] () // Capture "this" and the percentage request
                {
                    this->setPct(pct); // Set the percentage of the lift to the request
                },
                {this}  // Add "this", the pointer to this subsystem that is currently running.
                        // It is important to ensure that all subsystems that are being utilized in a command are properly
                        // Freed to allow that command to run.
            );
        }

        ~LiftSubsystem() override = default;
};
