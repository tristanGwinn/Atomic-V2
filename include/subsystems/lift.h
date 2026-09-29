#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "hardware/Motor/MotorGroup.hpp"

// This is a subsystem class for the cascade lift
class LiftSubsystem : public Subsystem {
    private:
        const double max_lift_height = 12.0;

        MotorGroup motor;
        
        double winch_diameter;
        bool isWinchWound = false;

        double position = 0;
        double prev_position = 0;

        PID pid;

        std::optional<double> voltage;
        std::optional<double> target;

    public:
        explicit LiftSubsystem(MotorGroup &motors, double diameter, const PID &pid) : motor(motors), winch_diameter(diameter), pid(pid) {
            motor.setAngle(0_stDeg);    // tare encoder upon initialization
        }

        /**
         * This function executes every fram of the command scheduler
         */
        void periodic() override {
            position = getPosition();

            if (voltage) isWinchWound = checkWinchStatus(voltage.value());

            // if the motor is not powered and a target is set, move with the PID controller 
            if (!voltage && target) {
                const auto control_out = pid.update(position);
                printf("lift error: %f \n", position - target.value());
                printf("lift control output: %f \n", control_out);
                
                if(control_out < 0 && isWinchWound) // if attempting to overwind the lift, coast motors
                {
                    this->brakeMotors(BrakeMode::COAST);
                    printf("The winch seems to be wound, to avoid uneeded motor stress, skipping movement.");
                }
                else motor.move(control_out);       // otherwise, we chillin'

                isWinchWound = checkWinchStatus(control_out);
            }

            prev_position = position;
        }

        double getPosition() {
            return to_stRot(motor.getAngle()) * M_PI * winch_diameter;
        }

        /**
         * Move the lift motors at a signed percentage of voltage
         */
        void setPct(const double pct) {
            this->motor.move(pct);
            voltage = pct;
        }

        void setTarget(double target) {
            this->target = clamp(target, 0, max_lift_height);
            pid.setTarget(
                this->target.value()
            );
            voltage = std::nullopt;
        }

        void brakeMotors(BrakeMode brake_mode) {
            motor.setBrakeMode(brake_mode);
            motor.brake();
        }

        void stopAndHold() {
            voltage = std::nullopt;
            target = std::nullopt;
            brakeMotors(BrakeMode::HOLD);
        }

        bool checkWinchStatus(double voltage){
            // if the motor is reversed and is not moving, the winch is likely wound up
            if(voltage < 0 && prev_position == position)
                return true;
            else
                return false;
        }

        void moveToBottom() {
            if (!isWinchWound) this->setPct(-0.25);
            else brakeMotors(BrakeMode::COAST);     // if winch is wound, let go of lift
        }


        FunctionalCommand *positionCommand(double height, double threshold = 1.2) {
        return new FunctionalCommand(
            [this, height]() { this->setTarget(height);
                                     }, [this, height]() { this->setTarget(height); }, [](bool _) {
                                     }, [this, threshold, height]() {
                                         return abs(this->getPosition() - height) < threshold;
                                     }, {this});
        }

        FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->stopAndHold();
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
        }

        FunctionalCommand *lowerLift() {
        return new FunctionalCommand([this]() {
                                         this->moveToBottom();
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
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
