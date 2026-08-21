#pragma once

#include "lemlog/logger/sinks/terminal.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/llemu.hpp"

logger::Terminal terminal;

// other subsytems

    // this is just an example
    // inline pros::Motor intake(-14);
    // using the subsystems and sensors defined in ~/include/hardware is best practice where possible

// chassis setup 
atomic::MotorGroup rightDrive({8, 10}, 360_rpm);
atomic::MotorGroup leftDrive({-1, 11, -12, 13}, 360_rpm);

atomic::V5InertialSensor imu(1);

pros::Controller controller(pros::E_CONTROLLER_MASTER);