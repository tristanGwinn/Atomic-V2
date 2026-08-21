#pragma once

#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "atomic/chassis/trackingWheel.hpp"
#include "atomic/pose.hpp"
#include "atomic/pid.hpp"
#include "atomic/exitcondition.hpp"
#include "atomic/driveCurve.hpp"
#include "atomic/ramsete/ramseteController.hpp"
#include "hot-cold-asset/asset.hpp"

#include "atomic/MotionCancelHelper.hpp"

// much of this file was directly inspired by or copied from lemlib
namespace atomic{

/**
 * @brief class containing the sensors used for odometry
 */
class OdomSensors {
    public:
        /**
         * The sensors are stored in a class so that they can be easily passed to the chassis class
         * The variables are pointers so that they can be set to nullptr if they are not used
         * Otherwise the chassis class would have to have a constructor for each possible combination of sensors
         *
         * @param vertical1 pointer to the first vertical tracking wheel
         * @param vertical2 pointer to the second vertical tracking wheel
         * @param horizontal1 pointer to the first horizontal tracking wheel
         * @param horizontal2 pointer to the second horizontal tracking wheel
         * @param imu pointer to the IMU
         *
         */
        OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                    TrackingWheel* horizontal2, pros::Imu* imu);
        TrackingWheel* vertical1;
        TrackingWheel* vertical2;
        TrackingWheel* horizontal1;
        TrackingWheel* horizontal2;
        pros::Imu* imu;
};

/**
 * @brief class containing constants for a chassis controller
 */
class ControllerSettings {
    public:
        /**
         * @brief ControllerSettings constructor
         *
         * The constants are stored in a class so that they can be easily passed to the chassis class
         * Set a constant to 0 and it will be ignored
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param antiWindup integral anti windup range. If error is within this range, integral is set to 0
         * @param smallError range of error at which the chassis controller will exit if the error is within this range
         * for an amount of time determined by smallErrorTimeout
         * @param smallErrorTimeout the time the chassis controller will wait before exiting if error is within a
         * certain range determined by smallError
         * @param largeError range of error at which the chassis controller will exit if the error is within this range
         * for an amount of time determined by largeErrorTimeout
         * @param largeErrorTimeout the time the chassis controller will wait before exiting if error is within a
         * certain range determined by largeError
         * @param slew maximum acceleration
         *
         */
        ControllerSettings(float kP, float kI, float kD, float windupRange, float smallError, float smallErrorTimeout,
                           float largeError, float largeErrorTimeout, float slew)
            : kP(kP),
              kI(kI),
              kD(kD),
              windupRange(windupRange),
              smallError(smallError),
              smallErrorTimeout(smallErrorTimeout),
              largeError(largeError),
              largeErrorTimeout(largeErrorTimeout),
              slew(slew) {}

        float kP;
        float kI;
        float kD;
        float windupRange;
        float smallError;
        float smallErrorTimeout;
        float largeError;
        float largeErrorTimeout;
        float slew;
};

/**
 * @brief class containing constants for a drivetrain
 */
class Drivetrain {
    public:
        /**
         * @brief Drivetrain constructor
         *
         * The constants are stored in a class so that they can be easily passed to the chassis class
         * Set a constant to 0 and it will be ignored
         *
         * @param leftMotors pointer to the left motors
         * @param rightMotors pointer to the right motors
         * @param trackWidth the track width of the robot, in inches. This is the distance from the left wheels to the
         * right wheels
         * @param wheelDiameter the diameter of the wheel used on the drivetrain, in inches
         * @param rpm the rpm of the wheels
         * @param horizontalDrift higher values make the robot move faster but causes more overshoot on turns.
         * Recommended value of 2 if not using traction wheels, 8 if using traction wheels
         *
         */
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter,
                   float rpm, float horizontalDrift);
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        float trackWidth;
        float wheelDiameter;
        float rpm;
        float horizontalDrift;
};

/**
 * @brief AngularDirection
 *
 * When turning, the user may want to specify the direction the robot should turn in.
 * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 */
enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

/**
 * @brief Parameters for Chassis::turnToPoint
 *
 */
struct TurnToPointParams {
        /** whether the robot should turn to face the point with the front of the robot. True by default */
        bool forwards = true;
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        int maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        int minSpeed = 0;
        /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Parameters for Chassis::turnToHeading
 *
 */
struct TurnToHeadingParams {
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        int maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        int minSpeed = 0;
        /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Enum class DriveSide to specify the locked side in swing turns
 * 
 */
enum class DriveSide {
    LEFT, /** lock the left side of the drivetrain */
    RIGHT /** lock the right side of the drivetrain */
};

/**
 * @brief Parameters for Chassis::swingToPoint
 *
 */
struct SwingToPointParams {
        /** whether the robot should turn to face the point with the front of the robot. True by default */
        bool forwards = true;
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Parameters for Chassis::swingToHeading
 *
 */
struct SwingToHeadingParams {
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Parameters for Chassis::moveToPose
 *
 */
struct MoveToPoseParams {
        /** whether the robot should move forwards or backwards. True by default */
        bool forwards = true;
        /** how fast the robot will move around corners. Recommended value 2-15. 0 means use horizontalDrift set in
         * chassis class. 0 by default. */
        float horizontalDrift = 0;
        /** carrot point multiplier. value between 0 and 1. Higher values result in curvier movements. 0.6 by default */
        float lead = 0.6;
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
         * less accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Parameters for Chassis::moveToPoint
 *
 */
struct MoveToPointParams {
        /** whether the robot should move forwards or backwards. True by default */
        bool forwards = true;
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
         * less accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

extern ExpoDriveCurve defaultDriveCurve;

/**
 * @brief Chassis class
 */
class Chassis {
    public:

        /** 
         * These components are buit into lemlib for driver control settings, but as of now are commented out
         * 
         * 
         * lateralSettings settings for the lateral controller
         * angularSettings settings for the angular controller
         * sensors sensors to be used for odometry
         * throttleCurve curve applied to throttle input during driver control
         * turnCurve curve applied to steer input during driver control
         */
        /**
         * @brief Chassis constructor
         *
         * @note Currently missing driver control customization
         * 
         * @param drivetrain drivetrain to be used for the chassis
         * @param sensors sensors to be used for odometry 
         * 
         */
        Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                OdomSensors sensors, DriveCurve* throttleCurve = &defaultDriveCurve,
                DriveCurve* steerCurve = &defaultDriveCurve);

        /**
         * @brief Calibrate the chassis sensors. This should be called in the initialize function
         *
         * @param calibrateIMU whether the IMU should be calibrated. true by default
         *
         */
        void calibrate(bool calibrateIMU = true);

        /**
         * @brief Set the pose of the chassis
         *
         * @param x new x value
         * @param y new y value
         * @param theta new theta value
         * @param radians true if theta is in radians, false if not. False by default
         *
         */
        void setPose(float x, float y, float theta, bool radians = false);

        /**
         * @brief Set the pose of the chassis
         *
         * @param pose the new pose
         * @param radians whether pose theta is in radians (true) or not (false). false by default
         *
         */
        void setPose(Pose pose, bool radians = false);

        /**
         * @brief Get the pose of the chassis
         *
         * @param radians whether theta should be in radians (true) or degrees (false). false by default
         * @return Pose
         * 
         */
        Pose getPose(bool radians = false, bool standardPos = false);
        
        /**
         * @brief Wait until the robot has traveled a certain distance along the path
         *
         * @note Units are in inches or degrees
         *
         * @param dist the distance the robot needs to travel before returning
         *
         */
        void waitUntil(float dist);

        /**
         * @brief Wait until the robot has completed the path
         *
         */
        void waitUntilDone();

        /**
         * @brief Sets the brake mode of the drivetrain motors
         *
         * @param mode Mode to set the drivetrain motors to
         * 
         */
        void setBrakeMode(pros::motor_brake_mode_e mode);

        // TODO: simple movements; maybe move into their own classes
        
        /**
         * @brief Turn the chassis so it is facing the target point
         *
         * @param x x location
         * @param y y location
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        // void turnToPoint(float x, float y, int timeout, TurnToPointParams params = {}, bool async = true);
        
        /**
         * @brief Turn the chassis so it is facing the target heading
         *
         * @param theta heading location
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        // void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);
        
        /**
         * @brief Turn the chassis so it is facing the target heading, but only by moving one half of the drivetrain
         *
         * @param theta heading location
         * @param lockedSide side of the drivetrain that is locked
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        // void swingToHeading(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params = {},
        //                     bool async = true);
        
        /**
         * @brief Turn the chassis so it is facing the target point, but only by moving one half of the drivetrain
         *
         * @param x x location
         * @param y y location
         * @param lockedSide side of the drivetrain that is locked
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         * @endcode
         */
        // void swingToPoint(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params = {},
        //                   bool async = true);
        
        /**
         * @brief Move the chassis towards the target pose
         *
         * Uses the boomerang controller
         *
         * @param x x location
         * @param y y location
         * @param theta target heading in degrees.
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        // void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params = {}, bool async = true);
        
        /**
         * @brief Move the chassis towards a target point
         *
         * @param x x location
         * @param y y location
         * @param timeout longest time the robot can spend moving
         * @param params struct to simulate named parameters
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        // void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {}, bool async = true);


        /**
         * @brief Load a trajectory path from an asset
         * 
         * @note TODO: write code for this
         * 
         * @param path the path asset to follow
         * @return the trajectory read from the asset
         * 
         */
        Trajectory loadTrajectory(const asset& path);

        /**
         * @brief Move the chassis along a trajectory path
         *
         * @param t the trajectory path to follow
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        void followTrajectory(const Trajectory& t, bool async = true);

        /**
         * @brief Move the chassis along a trajectory path directly from an asset
         *
         * @param t the trajectory path to follow
         * @param async whether the function should be run asynchronously. true by default
         *
         */
        void followTrajectory(const asset& t, bool async = true);

        /**
         * @brief Control the robot during the driver using the tank drive control scheme. In this control scheme one
         * joystick axis controls the left motors' forward and backwards movement of the robot, while the other joystick
         * axis controls right motors' forward and backward movement.
         * @param left speed to move left wheels forward or backward. Takes an input from -127 to 127.
         * @param right speed to move right wheels forward or backward. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         *
         */
        void tank(int left, int right, bool disableDriveCurve = false);

        /**
         * @brief Control the robot during the driver using the arcade drive control scheme. In this control scheme one
         * joystick axis controls the forwards and backwards movement of the robot, while the other joystick axis
         * controls the robot's turning
         *
         * @param throttle speed to move forward or backward. Takes an input from -127 to 127.
         * @param turn speed to turn. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         * @param desaturateBias how much to favor angular motion over lateral motion or vice versa when motors are
         * saturated. A value of 0 fully prioritizes lateral motion, a value of 1 fully prioritizes angular motion
         *
         */
        void arcade(int throttle, int turn, bool disableDriveCurve = false, float desaturateBias = 0.5);
        
        /**
         * @brief Control the robot during the driver using the curvature drive control scheme. This control scheme is
         * very similar to arcade drive, except the second joystick axis controls the radius of the curve that the
         * drivetrain makes, rather than the speed. This means that the driver can accelerate in a turn without changing
         * the radius of that turn. This control scheme defaults to arcade when forward is zero.
         *
         * @param throttle speed to move forward or backward. Takes an input from -127 to 127.
         * @param turn speed to turn. Takes an input from -127 to 127.
         * @param disableDriveCurve whether to disable the drive curve or not. If disabled, uses a linear curve with no
         * deadzone or minimum power
         *
         */
        void curvature(int throttle, int turn, bool disableDriveCurve = false);

        /**
         * @brief Cancels the currently running motion.
         * If there is a queued motion, then that queued motion will run.
         *
         */
        void cancelMotion();

        /**
         * @brief Cancels all motions, even those that are queued.
         * After this, the chassis will not be in motion.
         *
         */
        void cancelAllMotions();

        /**
         * @return whether a motion is currently running
         *
         */
        bool isInMotion() const;

        /**
         * @brief Resets the x and y position of the robot
         * without interfering with the heading.
         *
         */
        void resetLocalPosition();

        // this part was included with lemlib, the warnings are hyping it up more than it needs to be lmao
        // commented out until PID is implemented
        /**
         * PIDs are exposed so advanced users can implement things like gain scheduling
         * Changes are immediate and will affect a motion in progress
         *
         * @warning Do not interact with these unless you know what you are doing
         */
        PID lateralPID;
        /**
         * PIDs are exposed so advanced users can implement things like gain scheduling
         * Changes are immediate and will affect a motion in progress
         *
         * @warning Do not interact with these unless you know what you are doing
         */
        PID angularPID;

    protected:
        /**
         * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
         */
        void requestMotionStart();
        /**
         * @brief Dequeues this motion and permits queued task to run
         */
        void endMotion();

        bool motionRunning = false;
        bool motionQueued = false;

        float distTraveled = 0;

        ControllerSettings lateralSettings;
        ControllerSettings angularSettings;
        Drivetrain drivetrain;
        OdomSensors sensors;
        DriveCurve* throttleCurve = nullptr;
        DriveCurve* steerCurve = nullptr;

        RamseteController ramseteController;

        ExitCondition lateralLargeExit;
        ExitCondition lateralSmallExit;
        ExitCondition angularLargeExit;
        ExitCondition angularSmallExit;
    private:
        pros::Mutex mutex;
};

}   // namespace atomic