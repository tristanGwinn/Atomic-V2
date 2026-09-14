/** 
 * 
 * 45434A VEX Atomic Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 9/14/26
 * Updated By: Tristan Gwinn
 * 
 * Current Task: Clean up code and directory
 * 
 * What's new:
 *  - Removed namespace "atomic" and "include/atomic/"
 *  - moved lose files to "deprecate/" in both "src/" and "include/"
 * 
 * Task to-do:
 *  - Create dedicated file for Command and subsystem initalization
 *  - Move robot config variables out of main.cpp
 *  - Remove lemlib style motion handling and replace with command based motions
 *  - Implement Commands where possible and update/remove files in "deprecate/"
 * 
 * Main to-do: 
 *  - Add functionality to read json files as trajectories (High Priority)
 * 	- Test RAMSETE functionality (Med Priority)
 *  - Update Atomic logo to new pink version (Low Priority)
 * 	- Driver control recording/playback (Low Priority)
 * 
 */

/////
//
//  To download code on linux, run sudo chmod a+rw after connecting brain / controller
//
/////

#include "main.h"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "driveCurve.hpp"
#include "motionConfig.hpp"
#include "chassis/odom.hpp"

#include "motions/ramseteTrajectoryFollower.hpp"

#include "trajectory/kinematics.hpp"
#include "trajectory/trajectoryGenerator.hpp"

#include "command/commandController.h"

#include "subsystems/lift.h"
#include "subsystems/drivetrain.h"

#include "pros/llemu.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
ExpoDriveCurve driveCurve(0.0, 1.0, 1.12);

// Physical robot variables
const Length track_width = 11.50_in;
const Length wheel_diameter = 2.75_in;
const AngularVelocity max_rpm = 450_rpm;

const LinearVelocity max_vel = 64.8_inps;           // max_vel = wheel_diameter * PI * max_rpm / 60_sec
const LinearAcceleration max_accel = 3.40_mps2;     // max_accel = drivetrain force at max rpm / robot mass

DifferentialKinematics robot_kinematics(
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

MotorGroup left_motors({-11, -12}, 450_rpm);
MotorGroup right_motors({20, 19}, 450_rpm);

V5InertialSensor imu(2);

TrackingWheel vertical_tracker(
                                ReversibleSmartPort(20),    // tracking port
                                wheel_diameter,                     // diameter
                                track_width / 2                     // offset
                            );

TrackingWheel horizontal_tracker(
                                ReversibleSmartPort(18),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );

Odometry odom({&imu}, {&vertical_tracker}, {&horizontal_tracker});

// PID variables
extern const PID angular_pid(0.05, 0, 0);
extern const PID lateral_pid(0.05, 0, 0);

const ExitConditionGroup<AngleRange> angular_exit_conditions({ExitCondition<AngleRange>(0.5_cDeg, 1000_msec)});
const ExitConditionGroup<Length> lateral_exit_conditions({ExitCondition(1.0_in, 2000_msec)});

const Number angular_slew = 1.0;
const Number lateral_slew = 1.0;

CommandController primary(pros::E_CONTROLLER_MASTER);   // set the controller for command triggers

MotorGroup lift_motors({-13, 7}, 600_rpm);  // lift motors

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;

// brain image stuff
LV_IMAGE_DECLARE(logo);
bool logoOnBrain = false;


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


void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}
    else pros::lcd::initialize();   // to get rid of pros screen run: pros::lcd::shutdown();

    // Start the command scheduler task
    pros::Task commandSchedulerTask(update_loop);

    // add components to subsystems
    lift = new LiftSubsystem(lift_motors);
    drivetrain = new DriveSubsystem(left_motors, right_motors);

    // register subsystems
    CommandScheduler::registerSubsystem(drivetrain, drivetrain->arcade(primary));
    CommandScheduler::registerSubsystem(lift, lift->pctCommand(0.0));

    // primary.getTrigger(ANALOG_LEFT_Y)->whileTrue(lift->pctCommand(1.0));

    primary.getTrigger(DIGITAL_R1)->whileTrue(lift->pctCommand(1.0));
    primary.getTrigger(DIGITAL_L1)->whileTrue(lift->pctCommand(-1.0));

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
