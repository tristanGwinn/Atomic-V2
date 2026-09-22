#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "controllers/pid.hpp"

#include "hardware/Motor/MotorGroup.hpp"
#include "pros/imu.hpp"

class ArmSubsystem : public Subsystem {
    private:
        MotorGroup motor;
        pros::Imu imu;

        PID pid;

        std::optional<double> voltage;
        std::optional<double> target;

    public:
        explicit ArmSubsystem(MotorGroup &motors, pros::Imu &inertial, const PID &pid)
         : motor(motors), imu(inertial), pid(pid) {
            motor.setAngle(0_stDeg);
            imu.reset();
        }

        void periodic() override {
            auto position = this->getPosition();
            
            if (!voltage.has_value() && target.has_value()) {
                const auto command = pid.update(position.convert(rad));
                motor.move(command);
            }
        }

        Angle getPosition() const {
            // to-do: consider the imu rotation of the chassis

            // convert the roll into rotation relative to initial rotation (tuned constant)
            // 35 should be replaced with a tuned constant defined elsewhere
            auto pos = (imu.get_roll() < 0) ?
                       180*2 + imu.get_roll() + 35 : imu.get_roll() - 35; 
            return from_stDeg(pos);    
        }

        void setTarget(Angle target) {
            pid.setTarget(target.convert(rad));
            voltage = std::nullopt;
        }

        FunctionalCommand *positionCommand(Angle angle, Angle threshold = 8_stDeg) {
        return new FunctionalCommand(
            [this, angle]() { this->setTarget(angle);
                                     }, [this, angle]() { this->setTarget(angle); }, [](bool _) {
                                     }, [this, threshold, angle]() {
                                         return abs(this->getPosition() - angle) < threshold;
                                     }, {this});
        }

        FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->setTarget(
                                             this->getPosition() + 5 *
                                             1_stDeg);
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
        }

        ~ArmSubsystem() override = default;
};
