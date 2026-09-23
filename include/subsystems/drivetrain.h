#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "chassis/trackingWheel.hpp"
#include "hardware/Motor/MotorGroup.hpp"

class DriveSubsystem : public Subsystem {
    private:
        MotorGroup left_motors;
        MotorGroup right_motors;

        TrackingWheel horizontal_tracker;
        pros::Imu imu_pros;
        V5InertialSensor imu;

        float deadband = 0;     // no deadband by default
        float minOutput = 0;    // no min output by default
        float curve = 1.0;      // no curve by default

        std::optional<double> pct;

        bool track_odom = false;

        Length left_previous = 0_m;
        Length right_previous = 0_m;

        Length left_delta = 0_m;
        Length right_delta = 0_m;

        Angle theta_offset = 0_stDeg;
        Pose pose = {0_m, 0_m, 0_stDeg};

    public:
        explicit DriveSubsystem(MotorGroup &leftmotors, MotorGroup &rightmotors, 
                                pros::Imu inertial, TrackingWheel &tracker,
                                float deadband, float minOutput, float curve)
        : left_motors(leftmotors), right_motors(rightmotors),
          imu_pros(inertial), imu(V5InertialSensor::from_pros_imu(inertial)), horizontal_tracker(tracker),
          deadband(deadband), minOutput(minOutput), curve(curve) 
        {
            calibrateTracking();
        }

        explicit DriveSubsystem(MotorGroup &leftmotors, MotorGroup &rightmotors, 
                                pros::Imu inertial, TrackingWheel &tracker)
        : left_motors(leftmotors), right_motors(rightmotors),
          imu_pros(inertial),  imu(V5InertialSensor::from_pros_imu(inertial)), horizontal_tracker(tracker)
        {
            calibrateTracking();
        }

        void periodic() override {
            if(track_odom) {
                updateOdom();
                sendOdomDebug();
            }

            // printf("Horizontal Tracker (Deg): %d\n", horizontal_tracker.getRotations() * 180.0 / M_PI);
            // printf("Horizontal Tracker (Rotations): %d\n", horizontal_tracker.getRotations() / M_TWOPI);

            // printf("Wrapper is valid: %d\n", imu.isConnected());  // If this method exists

            
            // printf("POSE \n");
        }

        // for debugging purposes ... will likely remove later
        void sendOdomDebug(){
            printf("PROS IMU heading: %f deg\n", imu_pros.get_heading());
            printf("Wrapper Rotation: %f rad\n", imu.getRotation().internal());
            printf("Horizontal Tracker: %f rad\n", horizontal_tracker.getTrackerAngle().internal());
            printf("Left Drivetrain Tracker: %f in\n", to_in(to_stRad(left_motors.getAngle()) * config::wheel_diameter));
            printf("x (sideways): %f \n", pose.x.internal());
            printf("y (horizontal): %f \n", pose.y.internal());
            printf("theta: %f \n", pose.orientation.internal());
        }

        void updateOdom() {
            // get sideways delta and offset
            const Length sideways_delta = horizontal_tracker.getDistanceDelta();
            const Length sideways_offset = horizontal_tracker.getOffset();

            updateLeftDelta(); updateRightDelta();
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
                return V2Position(twosinehalftheta * (sideways_delta / to_stRad(deltaTheta)) + sideways_offset,
                                  twosinehalftheta * (forward_delta / to_stRad(deltaTheta)));
            }();

            // set global position, V2Position makes this super easy
            // pose += localChange.rotatedBy(pose.orientation + deltaTheta / 2);    // rotate by avg theta


            // Instead of the rotateBy() function of V2Position, this is done manually here for debugging purposes.

            // get the magnitude of the local change vector
            const Length magnitude = sqrt(localChange.x*localChange.x + localChange.y*localChange.y);

            // angle of the local change vector plus the angle of rotation (global frame average)
            const Angle rotated_theta = atan2(localChange.y, localChange.x) + (pose.orientation + deltaTheta / 2);

            // add rotated local change vector to global position vector
            pose.x += magnitude * cos(rotated_theta);
            pose.y += magnitude * sin(rotated_theta);

            pose.orientation = theta;   // set pose orientation to measured robot angle
        }

        void updateLeftDelta() {
            const auto pos = to_stRad(left_motors.getAngle()) * config::wheel_diameter;
            left_delta = pos - left_previous;
            left_previous = pos;
        }

        void updateRightDelta() {
            const auto pos = to_stRad(right_motors.getAngle()) * config::wheel_diameter;
            right_delta = pos - right_previous;
            right_previous = pos;
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
            right_previous= 0_m;
        }

        void calibrateIMU() {
            imu.calibrate();
            WAIT_UNTIL(imu.isCalibrated());
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
            pros::lcd::print(5, "DRIVETRAIN VOLTAGES (%)");
            pros::lcd::print(6, "left: %f", pct_left);
            pros::lcd::print(7, "right: %f", pct_right);
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
