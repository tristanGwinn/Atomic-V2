#pragma once

#include "config.hpp"
#include "controllers/pid.hpp"
#include "chassis/trackingWheel.hpp"

#include "command/sequence.h"
#include "command/commandController.h"

#include "subsystems/lift.h"
#include "subsystems/drivetrain.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

#include "commands/score.h"
#include "commands/resetArmLift.h"
#include "commands/ramsete.h"

#include "trajectory/trajectory.h"
#include "trajectory/path.hpp"
#include "trajectory/kinematics.hpp"
#include "trajectory/trajectoryGenerator.hpp"

CommandController primary(pros::E_CONTROLLER_MASTER);   // set the controller for command triggers

MotorGroup left_motors({-10, -9}, 450_rpm);
MotorGroup right_motors({3, 1}, 450_rpm);
pros::Imu imu(20);

TrackingWheel horizontal_tracker(
                                ReversibleSmartPort(13),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );

MotorGroup lift_motors({-11, 21}, 600_rpm); // lift motors
constexpr Length lift_winch_diameter = 20_mm;

PID lift_pid(0.15, 0.0, 0.0, 0.0, false);

MotorGroup arm_motors({6, -4}, 600_rpm);
pros::Imu arm_imu(8);
PID arm_pid(0.35, 0.0, 0.0, 0.0, false);

pros::adi::DigitalOut claw_solenoid('A');
pros::Distance claw_distance(16);

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;
ArmSubsystem *arm;
ClawSubsystem *claw;

ResetArmLift *resetArmLift;
Score *scorePos1;


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
    imu.tare();

    lift = new LiftSubsystem(lift_motors, lift_winch_diameter, lift_pid);
    arm = new ArmSubsystem(arm_motors, arm_imu, imu, arm_pid);
    drivetrain = new DriveSubsystem(left_motors, right_motors, imu, horizontal_tracker);
    claw = new ClawSubsystem(claw_solenoid, claw_distance);
    
    CommandScheduler::registerSubsystem(drivetrain, drivetrain->arcade(primary));
    CommandScheduler::registerSubsystem(lift, lift->holdPositionCommand()); 
    CommandScheduler::registerSubsystem(arm, arm->holdPositionCommand());
    CommandScheduler::registerSubsystem(claw, claw->levelCommand(false));
    
    claw->levelCommand(true)->schedule();

    resetArmLift = new ResetArmLift(lift, arm);
    scorePos1 = new Score(lift, arm, claw, {180.0, 6.0});

    primary.getTrigger(DIGITAL_A)
        ->onTrue(
            resetArmLift
            ->andThen(claw->clampWhenReadyCommand())
            ->andThen(scorePos1)
            ->andThen(resetArmLift)
        );

    primary.getTrigger(DIGITAL_UP)
        ->onTrue(
                arm->positionCommand(180.0)
                   ->andThen(arm->holdPositionCommand())
            );

    primary.getTrigger(DIGITAL_LEFT)
        ->onTrue(
                lift->positionCommand(10_in)
                    ->andThen(lift->holdPositionCommand())
            );

    primary.getTrigger(DIGITAL_B)->onTrue(resetArmLift);

    
    printf("\nSubsystems have been set up!\n\n");
}
