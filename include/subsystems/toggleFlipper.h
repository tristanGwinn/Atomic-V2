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
class ToggleSubsystem : public Subsystem {
    private:
        pros::adi::DigitalOut left;
        pros::adi::DigitalOut right;
        pros::adi::DigitalOut swiper;

    public:
        
        explicit ToggleSubsystem(pros::adi::DigitalOut &left, pros::adi::DigitalOut &right, pros::adi::DigitalOut& swiper)
        : left(left), right(right), swiper(swiper) {
            // prolly dont need anything here
        }

        void periodic() override {
            // no-op
        }


        void setLevelLeft(const bool value) {
            left.set_value(value);
        }

        void setLevelRight(const bool value) {
            right.set_value(value);
        }

        void setLevelSwiper(const bool value) {
            swiper.set_value(value);
        }

        FunctionalCommand *levelCommand(bool value) {
            return new FunctionalCommand(
                [this, value]() { 
                    this->setLevelLeft(value); 
                    this->setLevelRight(value);  
                    this->setLevelSwiper(value);
                }, 
                [this, value]() {
                    this->setLevelLeft(value); 
                    this->setLevelRight(value);  
                    this->setLevelSwiper(value);
                }, [this](bool _) { },
                [this]() { return true; },
                {this}
            );
        }

        FunctionalCommand *mainLevelCommand(bool value) {
            return new FunctionalCommand(
                [this, value]() { 
                    this->setLevelLeft(value); 
                    this->setLevelRight(value);  
                }, 
                [this, value]() {
                    this->setLevelLeft(value); 
                    this->setLevelRight(value);
                }, [this](bool _) { },
                [this]() { return true; },
                {this}
            );
        }

        FunctionalCommand *swiperAndRightCommand(bool value, Time startTime) {
            return new FunctionalCommand(
                [this, value]() { 
                    this->setLevelSwiper(value); 
                }, 
                [this, value, startTime]() {
                    this->setLevelSwiper(value); 
                    if (from_msec(pros::millis()) - startTime > 1500_msec)
                        this->setLevelRight(value); 
                }, [this](bool _) { },
                [this]() { return true; },
                {this}
            );
        }

        FunctionalCommand *swiperLevelCommand(bool value) {
            return new FunctionalCommand(
                [this, value]() { 
                    this->setLevelSwiper(value); 
                }, 
                [this, value]() {
                    this->setLevelSwiper(value); 
                }, [this](bool _) { },
                [this]() { return true; },
                {this}
            );
        }

        FunctionalCommand *rightLevelCommand(bool value) {
            return new FunctionalCommand(
                [this, value]() { 
                    this->setLevelRight(value); 
                }, 
                [this, value]() {
                    this->setLevelRight(value); 
                }, [this](bool _) { },
                [this]() { return true; },
                {this}
            );
        }

        ~ToggleSubsystem() override = default;
};
