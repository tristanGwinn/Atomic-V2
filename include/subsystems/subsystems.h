#pragma once

#include "config.hpp"
#include "controllers/pid.hpp"
#include "chassis/trackingWheel.hpp"

#include "command/sequence.h"
#include "command/commandController.h"

#include "subsystems/solenoidSubsystem.h"

#include "subsystems/lift.h"
#include "subsystems/drivetrain.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

#include "commands/score.h"
#include "commands/resetArmLift.h"

#include "commands/ramsete.h"
#include "commands/rotate.h"

#include "trajectory/trajectory.h"
#include "trajectory/path.hpp"
#include "trajectory/kinematics.hpp"
#include "trajectory/trajectoryGenerator.hpp"

CommandController primary(pros::E_CONTROLLER_MASTER);   // set the controller for command triggers

MotorGroup left_motors({-17, -9}, 450_rpm);
MotorGroup right_motors({3, 1}, 450_rpm);
pros::Imu imu(20);

TrackingWheel horizontal_tracker(
                                ReversibleSmartPort(13),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );

MotorGroup lift_motors({-11, 21}, 600_rpm); // lift motors
constexpr Length lift_winch_diameter = 20_mm;

PID lift_pid(9, 0.0, 0.0, 0.0, false);

MotorGroup arm_motors({6, -4}, 600_rpm);
pros::Imu arm_imu(8);
PID arm_pid(1.97, 0.0, 0.28, 0.0, false);

pros::adi::DigitalOut claw_solenoid('A');
pros::Distance claw_distance(16);

pros::adi::DigitalOut left_solenoid('C');
pros::adi::DigitalOut right_solenoid('D');
pros::adi::DigitalOut swiper_solenoid('E');

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;
ArmSubsystem *arm;
ClawSubsystem *claw;
// ToggleSubsystem *toggleFlipper;

ResetArmLift *resetArmLift;

Score *scorePosT;

Score *scorePos0;
Score *scorePos1;
Score *scorePos2;
Score *scorePos3;

Command* lastScoreCommand;

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
    CommandScheduler::registerSubsystem(lift, lift->dropLiftCommand()); 
    CommandScheduler::registerSubsystem(arm, arm->dropArmCommand());
    CommandScheduler::registerSubsystem(claw, claw->maintainCommand());
    
    // claw->levelCommand(false)->schedule();

    resetArmLift = new ResetArmLift(lift, arm, claw);

    //scorePos0 = new Score(lift, arm, {180.0, 0.0});
    scorePosT = new Score(lift, arm, {180.0, 0.0});

    scorePos0 = new Score(lift, arm, {275.5, 1.5});
    scorePos1 = new Score(lift, arm, {286.0, 6.5});
    scorePos2 = new Score(lift, arm, {173.0, 2.8});
    scorePos3 = new Score(lift, arm, {178.0, 6.0});
    // scorePos4 = new Score(lift, arm, claw, {171.0, 11.2});

    primary.getTrigger(DIGITAL_UP)
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnFalse(
            claw->levelCommand(true)
            ->andThen(scorePosT)
            ->andThen(arm->holdPositionCommand()->with(lift->holdPositionCommand()))
            ->andThen(new WaitCommand(150_msec))
        );

    primary.getTrigger(DIGITAL_B)
        ->onTrue(resetArmLift);

    primary.getTrigger(DIGITAL_RIGHT)
        ->toggleOnFalse(
            arm->positionCommand(18.65, 0.5)
            ->andThen(arm->holdPositionCommand())
            ->andThen(claw->levelCommand(false))
        );

    primary.getTrigger(DIGITAL_Y)
        ->toggleOnTrue(
            claw->levelCommand(true)->andThen(new WaitCommand(150_msec))
        )
        ->toggleOnFalse(
            arm->positionCommand(25, 1.0)
        );

    primary.getTrigger(DIGITAL_DOWN)
        ->onTrue(
            claw->levelCommand(false)
            ->andThen( new WaitCommand(2.5_sec) )
            ->andThen(claw->levelCommand(true))
        );

    primary.getTrigger(DIGITAL_R1)
    ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnFalse(
            claw->levelCommand(true)
            ->andThen(scorePos0)
            ->andThen(arm->holdPositionCommand()->with(lift->holdPositionCommand()))
            ->andThen(new WaitCommand(150_msec))
        );
    
    
    /*primary.getTrigger(DIGITAL_R1)->andOther(primary.getTrigger(DIGITAL_R2))
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnTrue(

        );*/
    

    primary.getTrigger(DIGITAL_R2)
    ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnFalse(
            claw->levelCommand(true)
            ->andThen(scorePos1)
            ->andThen(arm->holdPositionCommand()->with(lift->holdPositionCommand()))
            ->andThen(new WaitCommand(150_msec))
        );

    primary.getTrigger(DIGITAL_L1)
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnFalse(
            claw->levelCommand(true)
            ->andThen(scorePos2)
            ->andThen(arm->holdPositionCommand()->with(lift->holdPositionCommand()))
            ->andThen(new WaitCommand(150_msec))
        );
        
    primary.getTrigger(DIGITAL_L2)
    ->andOther(primary.getTrigger(DIGITAL_B)->negate())
        ->toggleOnFalse(
            claw->levelCommand(true)
            ->andThen(scorePos3)->andThen(new WaitCommand(50_msec))
            ->andThen(lift->holdPositionCommand())//->andThen(arm->holdPositionCommand())
            ->andThen(new WaitCommand(150_msec))
        );

    resetArmLift->schedule();

    printf("\nSubsystems have been set up!\n\n");
}