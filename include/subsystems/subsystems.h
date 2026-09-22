#pragma once

#include "config.hpp"
#include "controllers/pid.hpp"
#include "chassis/trackingWheel.hpp"

#include "command/commandController.h"

#include "subsystems/lift.h"
#include "subsystems/drivetrain.h"
#include "subsystems/arm.h"


CommandController primary(pros::E_CONTROLLER_MASTER);   // set the controller for command triggers

MotorGroup left_motors({-10, -9}, 450_rpm);
MotorGroup right_motors({3, 2}, 450_rpm);
pros::Imu imu(2);


TrackingWheel horizontal_tracker(
                                ReversibleSmartPort(18),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );


MotorGroup lift_motors({-13, 7}, 600_rpm);  // lift motors

MotorGroup arm_motors({6, -4}, 600_rpm);
pros::Imu arm_imu(10);
PID arm_pid(0.0, 0.0, 0.0, 0.0, false);

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;
ArmSubsystem *arm;

/**
 * @brief This function runs the update scheduler at each frame with a consistent schedule
 *
 * @warning This function or alternative similar to it must be running to ensure the \refitem CommandScheduler is run
 */
[[noreturn]] void update_loop() {
	// Loop forever
	while (true) {
		// Store the start time
		auto start_time = pros::millis();

		// Run the command scheduler
		// This might be an expensive(Time wise) computation
		CommandScheduler::run();

		// Use delay until if this computation ends up being expensive, keeping loop time in check
		pros::c::task_delay_until(&start_time, 10);
	}
}

void initializeSubsystems(){
    lift = new LiftSubsystem(lift_motors);
    drivetrain = new DriveSubsystem(left_motors, right_motors, imu, horizontal_tracker);
    arm = new ArmSubsystem(arm_motors, arm_imu, arm_pid);
    
    CommandScheduler::registerSubsystem(drivetrain, drivetrain->arcade(primary));
    CommandScheduler::registerSubsystem(lift, lift->pctCommand(0.0));
    
    // Move lift up on R1 and down on L1
    primary.getTrigger(DIGITAL_R1)->whileTrue(lift->pctCommand(1.0));
    primary.getTrigger(DIGITAL_L1)->whileTrue(lift->pctCommand(-1.0));
}