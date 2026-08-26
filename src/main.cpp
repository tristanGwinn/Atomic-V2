/** 
 * 
 * 45434A VEX Atomic Framework and Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 8/25/26
 * Updated By: Tristan Gwinn
 * 
 * Note:
 * 	As of now, nothing has been tested,
 *  and some functionality may be broken/or unintentionally left unfinished.
 * 
 * ToDo: 
 * 	It's okay to reference old code and other templates/frameworks to implement features
 * 	- Create a formatting guide
 * 	- Driver control recording/playback
 *  - Verify driver control functionality
 * 	- Review RAMSETE controller code
 *  - Review and potentially remove atomic::Pose in favor of units::pose
 * 	
 * What's New:
 * 	- Most of the code now uses proper units and is formatted consistently
 * 	- Ramsete controller (untested)
 *  - Realtime motion-profiled trajectory generation
 * 	- Trajectory following
 * 	- Turn to
 *  - Move to Pose
 *  - Move to Point
 *  - Chassis class was removed
 *  - toAngularVelocity() was added to units class to handle calculating angular velocity
 * 
 */

#include "main.h"
#include "lemlog/logger/sinks/terminal.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "atomic/driveCurve.hpp"
#include "atomic/motionConfig.hpp"
#include "pros/llemu.hpp"
#include "subsystems.hpp"

atomic::ExpoDriveCurve defaultDriveCurve;


logger::Terminal terminal;

atomic::MotorGroup rightDrive({8, 10}, 360_rpm);
atomic::MotorGroup leftDrive({-1, 11, -12, 13}, 360_rpm);

atomic::V5InertialSensor imu(1);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

bool logoOnBrain = false;
LV_IMAGE_DECLARE(logo);

void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}



}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}