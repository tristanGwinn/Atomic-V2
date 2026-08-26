#pragma once

#include "atomic/trajectory/path.hpp"
#include "atomic/trajectory/trajectory.h"
#include <vector>

namespace atomic {

class Kinematics {
    public:
        virtual LinearVelocity getMaxSpeed(Path *path, Trajectory::State lastState, Length deltaD, Time t) const = 0;
        virtual std::vector<LinearVelocity> getWheelVelocities(LinearVelocity linearVel, AngularVelocity angularVel) const = 0;
        virtual ~Kinematics() = default;
};

class DifferentialKinematics : public Kinematics {
    public:
        DifferentialKinematics(Length trackWidth, LinearVelocity maxVel, LinearAcceleration maxAccel, Number friction)
            : m_track_width(trackWidth), m_max_vel(maxVel), m_max_accel(maxAccel),
              m_friction(friction) {}
  
        LinearVelocity getMaxSpeed(Path *path, Trajectory::State lastState, Length deltaD, Time t) const override;

        std::vector<LinearVelocity> getWheelVelocities(LinearVelocity linearVel, AngularVelocity angularVel) const override;

    private:
        Length m_track_width;
        LinearVelocity m_max_vel;
        LinearAcceleration m_max_accel;
        Number m_friction;
};

}   // namespace atomic