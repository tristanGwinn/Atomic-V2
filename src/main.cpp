/** 
 * 
 * 45434A VEX Atomic Framework and Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 9/8/26
 * Updated By: Tristan Gwinn
 * 
 * ToDo: 
 *  - Add a subsystem / action command scheduler to handle subsystem use (Current Focus)
 *  - Add functionality to read json files as trajectories (High Priority)
 * 	- Test RAMSETE functionality (Med Priority)
 *  - Update Atomic logo to new pink version (Low Priority)
 * 	- Driver control recording/playback (Low Priority)
 * 	
 * What's New:
 *  - Robot is drivable
 *  - Temporary operator control of the lift has been added
 * 
 */

#include "main.h"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "atomic/driveCurve.hpp"
#include "atomic/motionConfig.hpp"
#include "atomic/chassis/odom.hpp"

#include "atomic/motions/ramseteTrajectoryFollower.hpp"

#include "atomic/trajectory/kinematics.hpp"
#include "atomic/trajectory/trajectoryGenerator.hpp"

#include "pros/llemu.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
atomic::ExpoDriveCurve defaultDriveCurve(0.0, 1.0, 1.12);

// Physical robot variables
const Length track_width = 11.50_in;
const Length wheel_diameter = 2.75_in;
const AngularVelocity max_rpm = 450_rpm;

const LinearVelocity max_vel = 64.8_inps;           // max_vel = wheel_diameter * PI * max_rpm / 60_sec
const LinearAcceleration max_accel = 3.40_mps2;     // max_accel = drivetrain force at max rpm / robot mass

atomic::DifferentialKinematics robot_kinematics(
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

atomic::MotorGroup left_motors({-11, -12}, 450_rpm);
atomic::MotorGroup right_motors({20, 19}, 450_rpm);

atomic::V5InertialSensor imu(2);

atomic::TrackingWheel vertical_tracker(
                                atomic::ReversibleSmartPort(20),    // tracking port
                                wheel_diameter,                     // diameter
                                track_width / 2                     // offset
                            );

atomic::TrackingWheel horizontal_tracker(
                                atomic::ReversibleSmartPort(18),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );

atomic::Odometry odom({&imu}, {&vertical_tracker}, {&horizontal_tracker});

// PID variables
extern const atomic::PID angular_pid(0.05, 0, 0);
extern const atomic::PID lateral_pid(0.05, 0, 0);

const atomic::ExitConditionGroup<AngleRange> angular_exit_conditions({atomic::ExitCondition<AngleRange>(0.5_cDeg, 1000_msec)});
const atomic::ExitConditionGroup<Length> lateral_exit_conditions({atomic::ExitCondition(1.0_in, 2000_msec)});

const Number angular_slew = 1.0;
const Number lateral_slew = 1.0;

LV_IMAGE_DECLARE(logo);


bool logoOnBrain = true;
bool showDebug = true;

void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}

    if(imu.isConnected()){
        imu.calibrate();
	    odom.startTask();
    }else{
        showDebug = false;
    }

	if (showDebug) pros::lcd::initialize(); // initialize brain screen if needed

    while (true) {
        if (showDebug){
            // get current pose of the robot
            const units::Pose pose = odom.getPose();
            // print pros to the brain screen
            pros::lcd::print(0, "x: %f", (pose.x));
            pros::lcd::print(1, "y: %f", (pose.y));
            pros::lcd::print(2, "theta: %f", (pose.orientation));
        }
        
        // delay to let other tasks run
        pros::delay(10);
	}

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

// this is here temporarily due to testing needs

// stage right lift motor = 13, reversed : (-13)
// stage left lift motor = 7, forward    : (7)
atomic::MotorGroup lift_motors({-13, 7}, 600_rpm);
float lift_percent = 1;

void opcontrol() {
    // temp. code
	lift_motors.setBrakeMode(atomic::BrakeMode::HOLD);

	while (true) {
        float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    	float left = defaultDriveCurve.curve(leftY + rightX);
    	float right = defaultDriveCurve.curve(leftY - rightX);
    	left_motors.move(left / 127);
    	right_motors.move(right / 127);

        // temp. code start
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)))
		{
			lift_motors.move(lift_percent);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)))
		{
			lift_motors.move(-lift_percent);
		}
		else
		{
			lift_motors.brake();
		}
        // temp. code end

    	pros::delay(10);
  	}

}