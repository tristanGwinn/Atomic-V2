#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "hardware/Motor/MotorGroup.hpp"

// This is a subsystem class for the cascade lift
class LiftSubsystem : public Subsystem {
    private:
        const Length max_lift_height = 12.0_in;

        MotorGroup motor;
        
        Length winch_diameter;
        bool isWinchWound = false;

        Length position = 0_cm;
        Length prev_position = 0_cm;

        PID pid;

        std::optional<double> voltage;
        std::optional<Length> target;

    public:
        explicit LiftSubsystem(MotorGroup &motors, Length diameter, const PID &pid) : motor(motors), winch_diameter(diameter), pid(pid) {
            motor.setAngle(0_stDeg);    // tare encoder upon initialization
        }

        /**
         * This function executes every fram of the command scheduler
         */
        void periodic() override {
            position = getPosition();

            // if the motor is not powered and a target is set, move with the PID controller 
            if (!voltage && target) {
                const auto control_out = pid.update(to_cm(position));
                // printf("lift error: %f inches\n", to_in(position) - to_in(target.value()));
                // printf("lift control output: %f \n", control_out.internal());
                motor.move(control_out); 
            }

            prev_position = position;
        }

        Length getPosition() {
            return to_stRot(motor.getAngle()) * M_PI * winch_diameter;
        }

        Length getTarget() {
            return target.value_or(0_m);
        }

        /**
         * Move the lift motors at a signed percentage of voltage
         */
        void setPct(const double pct) {
            this->motor.move(pct);
            voltage = pct;
        }

        void setTarget(Length target) {
            this->target = units::clamp(target, 0_m, max_lift_height);
            pid.setTarget(
                to_cm(this->target.value())
            );
            voltage = std::nullopt;
        }

        void brakeMotors(BrakeMode brake_mode) {
            voltage = std::nullopt;
            target = std::nullopt;
            motor.setBrakeMode(brake_mode);
            motor.brake();
        }

        void stopAndHold() {
            brakeMotors(BrakeMode::HOLD);
        }

        // this function is pretty much useless
        /*bool checkWinchStatus(double voltage){
            // if the motor is reversed and is not moving, the winch is likely wound up
            if(voltage < 0 && prev_position == position)
                return true;
            else
                return false;
        }*/

        // void moveToBottom() {
        //     // if (!isWinchWound) this->setPct(-0.25);  // im dumb we dont need this
        //     brakeMotors(BrakeMode::COAST);
        // }


        FunctionalCommand *positionCommand(Length height, Length threshold = 2.3_cm) {
            std::cout << "The lift target position is set to " << height 
                      << " with a tolerance of " << threshold << std::endl;
            return new FunctionalCommand(
                [this, height]() { this->setTarget(height);
                                         }, [this, height]() { this->setTarget(height);
                                         }, [this](bool _) {
                                             printf("The lift is within tolerence of its target.\n");
                                             this->target = std::nullopt;
                                         }, [this, threshold, height]() {
                                             return units::abs(this->getPosition() - target.value()) < threshold;
                                         }, {this}
            );
        }

        FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->stopAndHold();
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
        }

        FunctionalCommand *dropLiftCommand() {
        return new FunctionalCommand([this]() {
                                         this->brakeMotors(BrakeMode::COAST);
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
