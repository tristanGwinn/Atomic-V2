#pragma once

#include "subsystems/solenoidSubsystem.h"
#include "command/runCommand.h"

#include "controllers/pid.hpp"

#include "hardware/Motor/MotorGroup.hpp"
#include "pros/imu.hpp"

// todo

class ClawSubsystem : public Subsystem {
    private:


    public:
        
        void periodic() override {

        }

        ~ClawSubsystem() override = default;

};
