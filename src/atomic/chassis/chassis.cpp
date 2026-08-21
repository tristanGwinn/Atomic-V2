#include <math.h>
#include "pros/motors.h"
#include "pros/rtos.h"
#include "atomic/logger/logger.hpp"
#include "atomic/util.hpp"
#include "atomic/chassis/chassis.hpp"
#include "atomic/chassis/odom.hpp"
#include "pros/rtos.hpp"

atomic::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu) {}

atomic::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

atomic::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                         OdomSensors sensors, DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : drivetrain(drivetrain),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      throttleCurve(throttleCurve),
      steerCurve(steerCurve),
      lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD, linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(atomic::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        atomic::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
        atomic::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
    }
}

void atomic::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new atomic::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new atomic::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void atomic::Chassis::setPose(float x, float y, float theta, bool radians) {
    atomic::setPose(atomic::Pose(x, y, theta), radians);
}

void atomic::Chassis::setPose(Pose pose, bool radians) { atomic::setPose(pose, radians); }

atomic::Pose atomic::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = atomic::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = rad_to_deg(pose.theta);
    return pose;
}

void atomic::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void atomic::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}



atomic::Trajectory atomic::Chassis::loadTrajectory(const asset& path) {
    // TODO
}

void atomic::Chassis::followTrajectory(const atomic::Trajectory& t, bool async = true) {
    std::vector<Trajectory::State> states = t.states();
    
    // if (states.size() == 0) {
    //     logHelper.error("No were detected in trajectory. Skipping motion");
    //     return;
    // }
    
    atomic::MotionCancelHelper helper(20_msec); // cancel helper

    // this value will let us convert linear velocity to motor voltage
    // meters per second * velToVoltage -->  new movement voltage
    const auto velToVoltage = (39.37 * 60 * 127) / (M_PI * this->drivetrain.wheelDiameter * this->drivetrain.rpm);

    for(int i = 1; i < states.size(); ++i) {
        if(helper.wait()) {
            // get the current position of the robot
            const Pose curState = this->getPose();

            DriveVelocities driveVel = ramseteController.calculate(curState, states[i]);

            // side velocity = linear velocity +/- angular velocity * track_width/2
            const auto leftSideVelocity = (driveVel.v / mps) - (driveVel.omega / rps) * this->drivetrain.trackWidth / 2;
            const auto rightSideVelocity = (driveVel.v / mps) + (driveVel.omega / rps) * this->drivetrain.trackWidth / 2;

            this->drivetrain.leftMotors->move(velToVoltage * leftSideVelocity);
            this->drivetrain.rightMotors->move(velToVoltage * rightSideVelocity);

            Time timeToWait = states[i].t - states[i - 1].t;
            pros::delay(timeToWait/sec);
        }else{
            break;
        }
    } 

    // stop the robot
    this->drivetrain.leftMotors->brake();
    this->drivetrain.rightMotors->brake();
}

void atomic::Chassis::followTrajectory(const asset& t, bool async = true) {
    // TODO
}


void atomic::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}
 
void atomic::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void atomic::Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void atomic::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool atomic::Chassis::isInMotion() const { return this->motionRunning; }

void atomic::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    atomic::setPose(atomic::Pose(0, 0, theta), false);
}

void atomic::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}