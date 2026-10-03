#pragma once

#include "config.hpp"
#include "controllers/pid.hpp"
#include "chassis/trackingWheel.hpp"

#include "command/commandController.h"

#include "subsystems/lift.h"
#include "subsystems/drivetrain.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

#include "commands/grabAndScore.h"

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

PID lift_pid(0.10, 0.0, 0.0, 0.0, false);

MotorGroup arm_motors({6, -4}, 600_rpm);
pros::Imu arm_imu(8);
PID arm_pid(0.15, 0.0, 0.0, 0.0, false);

pros::adi::DigitalOut claw_solenoid('A');
pros::Distance claw_distance(16);

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;
ArmSubsystem *arm;
ClawSubsystem *claw;

GrabAndScore *scorePos1;

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
    CommandScheduler::registerSubsystem(lift, lift->pctCommand(0.0));   // also temporary
    CommandScheduler::registerSubsystem(arm, arm->pctCommand(0.0));
    CommandScheduler::registerSubsystem(claw, claw->primeClampCommand());
    claw->levelCommand(true);

    scorePos1 = new GrabAndScore(lift, arm, claw, {180.0, 6.0});

    primary.getTrigger(DIGITAL_A)->onTrue(scorePos1);
    primary.getTrigger(DIGITAL_UP)
        ->onTrue(
                arm->positionCommand(180.0)
                   ->andThen(arm->holdPositionCommand())
            );
    primary.getTrigger(DIGITAL_LEFT)
        ->onTrue(
                lift->positionCommand(6_in)
                    ->andThen(lift->holdPositionCommand())
            );

    primary.getTrigger(DIGITAL_B)->onTrue(
        new ParallelCommandGroup({
            lift->positionCommand(0_in)
                ->andThen(lift->dropLiftCommand()),
            arm->positionCommand(40)
               ->andThen(arm->pctCommand(-1))
               ->withTimeout(1_sec)
               ->andThen(arm->dropArmCommand())
        })
    );

    
    // Move lift up on R1 and down on L1
    // primary.getTrigger(DIGITAL_R1)->whileTrue(lift->pctCommand(0.85));
    // primary.getTrigger(DIGITAL_L1)->whileTrue(lift->pctCommand(-0.6));

    // primary.getTrigger(DIGITAL_B)->whileTrue(arm->positionCommand(2.0));
    // primary.getTrigger(DIGITAL_L2)->whileTrue(arm->positionCommand(180.0));

    
    printf("Subsystems have been set up!\n");
}
