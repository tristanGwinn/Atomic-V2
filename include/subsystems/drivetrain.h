#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "util.hpp"
#include "chassis/trackingWheel.hpp"
#include "hardware/Motor/MotorGroup.hpp"

class DriveSubsystem : public Subsystem {
    private:
        // DifferentialKinematics *drive_kinematics;

        MotorGroup left_motors;
        MotorGroup right_motors;

        TrackingWheel horizontal_tracker;
        V5InertialSensor imu;

        float deadband = 0;     // no deadband by default
        float minOutput = 0;    // no min output by default
        float curve = 1.0;      // no curve by default

        std::optional<double> pct;

        Length left_previous = 0_m;
        Length right_previous = 0_m;

        Angle theta_offset = -90_stDeg;

        bool track_odom = false;
        Pose pose = {0_m, 0_m, 0_stDeg};

        std::optional<double> target_angle;
        std::optional<Length> pid_target;

        PID pid = config::lateral_pid;

        Pose start_pose = pose;
        Time start_time = from_msec(pros::millis());
        Time current_time = from_msec(pros::millis());

    public:
        explicit DriveSubsystem(MotorGroup &leftmotors, MotorGroup &rightmotors, 
                                pros::Imu &inertial, TrackingWheel &tracker,
                                const float deadband, const float minOutput, const float curve)
        : left_motors(leftmotors), right_motors(rightmotors),
          imu(V5InertialSensor::from_pros_imu(inertial)), horizontal_tracker(tracker),
          deadband(deadband), minOutput(minOutput), curve(curve) 
        {
	        printf("calibrating odom position ...");
	        this->calibrateTracking();
	        printf("DONE");
	        this->sendOdomDebug();
        }

        explicit DriveSubsystem(MotorGroup &leftmotors, MotorGroup &rightmotors, 
                                pros::Imu &inertial, TrackingWheel &tracker)
        : left_motors(leftmotors), right_motors(rightmotors),
          imu(V5InertialSensor::from_pros_imu(inertial)), horizontal_tracker(tracker)
        {
	        printf("calibrating odom position ...");
	        this->calibrateTracking();
	        printf("DONE");
	        this->sendOdomDebug();
        }

        void periodic() override {
            if(track_odom) {
                updateOdom();
	            this->sendOdomDebug();
                
                /*
                if (pid_target.has_value()){
                    const auto error = this->getPose().distanceTo( 
                            this->start_pose + units::V2Position::fromPolar(this->start_pose.orientation + 90_stDeg, pid_target.value()) 
                        );
                    const auto control_out = pid.update(to_in(error)) / 100;
                    setPct(control_out, control_out);
                }*/
            }
        }

        // for debugging purposes
        void sendOdomDebug(){
            printf("Sideways Tracker: %f in\n", to_in(horizontal_tracker.getDistanceTraveled()));
            printf("Drivetrain Position: %f in\n", to_in(left_previous + right_previous) / 2);
            printf("x (sideways): %f \n", to_in(pose.x));
            printf("y (forward): %f \n", to_in(pose.y));
            printf("theta: %f \n", to_stDeg(pose.orientation));
        }

        void updateOdom() {
            // get sideways delta and offset
            const Length sideways_delta = horizontal_tracker.getDistanceDelta();
            const Length sideways_offset = horizontal_tracker.getOffset();

            const auto left_delta = getLeftDelta();
            const auto right_delta = getRightDelta();

            // get forward drivetrain delta (avg of both sides)
            const Length forward_delta = (left_delta + right_delta) / 2;
            
            // get heading (from imu, if imu is messed up, get the heading from wheels)
            const Angle theta = theta_offset + 
                          ((imu.getRotation().internal() == INFINITY)
                          ? from_stDeg((left_delta + left_previous - right_delta - right_previous) / (config::track_width)) + 90_stDeg
                          : imu.getRotation());

            const Angle deltaTheta = theta - pose.orientation;

            // calculate local change vector
            const V2Position localChange = [&] {
                if(deltaTheta == 0_stRad) return V2Position(forward_delta, sideways_delta);
                const double twosinehalftheta = 2 * sin(deltaTheta / 2);
                return V2Position(twosinehalftheta * (sideways_delta / to_stRad(deltaTheta) + sideways_offset),
                                  twosinehalftheta * (forward_delta / to_stRad(deltaTheta)));
            }();

            // set global position, V2Position makes this super easy
            // pose += localChange.rotatedBy((pose.orientation + deltaTheta) / 2);    // rotate by avg theta

            // Instead of the rotateBy() function of V2Position, this is done manually here for debugging purposes.
            // get the magnitude of the local change vector
            const Length magnitude = sqrt(localChange.x*localChange.x + localChange.y*localChange.y);
            // angle of the local change vector plus the angle of rotation (global frame average)
            const Angle rotate_theta = atan2(localChange.y, localChange.x) + (pose.orientation + deltaTheta) / 2;
            // add rotated local change vector to global position vector
            pose.x += magnitude * cos(rotate_theta);
            pose.y += magnitude * sin(rotate_theta);

            pose.orientation = theta;   // set pose orientation to measured robot anlge
        }

        Length getLeftDelta() {
            const auto pos = to_stRad(left_motors.getAngle()) * config::wheel_diameter;
            const auto delta = pos - left_previous;
            left_previous = pos;
            return delta;
        }

        Length getRightDelta() {
            const auto pos = to_stRad(right_motors.getAngle()) * config::wheel_diameter;
            const auto delta = pos - right_previous;
            right_previous = pos;
            return delta;
        }


        void calibrateTracking(){
            track_odom = false;
            resetEncoders();
            calibrateIMU();
            pose = {0_m, 0_m, 0_stDeg};
            track_odom = true;
        }

        void resetEncoders() {
            left_motors.setAngle(0_stDeg);
            right_motors.setAngle(0_stDeg);
            horizontal_tracker.reset();
            left_previous = 0_m;
            right_previous = 0_m;
        }

        void calibrateIMU() {
            theta_offset = -M_PI_2 * rad;
            imu.calibrate();
            WAIT_UNTIL(imu.isCalibrated());
        }

        
        Pose getPose(){
            return this->pose;
        }

        void setDriveVelocities(DriveVelocities targets) {
            // side velocity = linear velocity +/- angular velocity * track_width/2
            const LinearVelocity leftSideVelocity = targets.v - toLinear<AngularVelocity>(targets.omega, config::track_width);
            const LinearVelocity rightSideVelocity = targets.v + toLinear<AngularVelocity>(targets.omega, config::track_width);

            const double left_out = leftSideVelocity / config::max_vel;
            const double right_out = rightSideVelocity / config::max_vel;
            const double sum = abs(left_out) + abs(right_out);

            left_motors.move( sum <= 1.0 ? left_out : left_out / sum );
            right_motors.move( sum <= 1.0 ? right_out : right_out / sum );
        }

        void setPidTarget(Length distance) {
            this->pid_target = distance;
            pid.setTarget(to_in(pid_target.value()));
        }

        /**
         * based on the expo curve from lemlib.
         * see https://www.desmos.com/calculator/umicbymbnl for more info
         */
        float expCurve(float input) {
            // return 0 if input is within deadzone
            if (fabs(input) <= deadband) return 0;

            // g is the output of g(x) as defined in the Desmos graph
            const float g = fabs(input) - deadband;

            // g127 is the output of g(127) as defined in the Desmos graph
            const float g127 = 127 - deadband;

            // i is the output of i(x) as defined in the Desmos graph
            const float i = pow(curve, g - 127) * g * sgn(input);

            // i127 is the output of i(127) as defined in the Desmos graph
            const float i127 = pow(curve, g127 - 127) * g127;

            return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * sgn(input);
        }

        /**
         * Move the drivetrain motors at a signed percentage of voltage
         */
        void setPct(const double pct_left, const double pct_right) {
            this->left_motors.move(pct_left);
            this->right_motors.move(pct_right);
        }

        FunctionalCommand* lateralPidCommand(Length distance, Time timeout = 5_sec, Length tolerance = config::default_tolerance){

        return new FunctionalCommand(
            [this, distance, tolerance]() { 
                         this->start_time = from_msec(pros::millis());
                         this->start_pose = this->getPose();
                         this->setPidTarget(distance);
                                     std::cout << "The drivetrain target position is set to " << units::V2Position::fromPolar(this->start_pose.orientation + 90_stDeg, distance).x << ", "
                      <<  units::V2Position::fromPolar(this->start_pose.orientation + 90_stDeg, distance).y
                      << " forward/reverse with a tolerance of " << to_in(tolerance) << std::endl;

                     }, [this, distance]() { 
                         this->current_time = from_msec(pros::millis());
                         this->setPidTarget(distance);

                     }, [this](bool _) {
                         this->pid_target = std::nullopt;
                         this->pct = std::nullopt;
                         printf("The drivetrain is within tolerence of its target.\n");

                     }, [this, distance, timeout, tolerance]() {
                         const auto error = this->getPose().distanceTo( this->start_pose + units::V2Position::fromPolar(this->start_pose.orientation + 90_stDeg, distance) );
                         std::cout << "Drivetrain error (inches): " << to_in(error) << std::endl;
                         return 
                            (error < tolerance) || (this->current_time - this->start_time >= timeout);
                     }, {this});
        }

        RunCommand* pctCommand(const double left, const double right) {
            // Create a new RunCommand
            // The lambda body is called at every update, in this case setting the intake percentage
            return new RunCommand(
                [this, left, right] () // Capture "this" and the percentage request
                {
                    this->setPct(left, right);
                },
                {this}
            );
        }

        RunCommand *arcade(pros::Controller &controller) {
            return new RunCommand(
                [this, controller]() mutable {
                    this->setPct(
                            this->expCurve((controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X)) / 127.0),
                            this->expCurve((controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X)) / 127.0));
                },
                {this});
        }

        ~DriveSubsystem() override = default;
};
