#pragma once

#include "Eigen/Dense"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "trajectory.h"

namespace atomic {

// I would rather use the V2Position vector for this.
// This should be phased out for V2Position,
// or a similar vector object when units are properly decided and handled
// class Point2D {
//     public:
//         Point2D() : x(0), y(0) {}
//         Point2D(Number x, Number y) : x(x), y(y) {}
//         Number x;
//         Number y;
// };

class Path {
    public:
        virtual units::V2Position getPoint(Time t) = 0;
        virtual units::V2Velocity getDerivative(Time) = 0;
        virtual units::V2Acceleration getSecondDerivative(Time t) = 0;
        virtual Time GetMaxT() const = 0;
        virtual ~Path() = default;
};

class CubicBezier : public Path {
    public:
        CubicBezier(units::V2Position p0, units::V2Position p1,
                    units::V2Position p2, units::V2Position p3);
        units::V2Position getPoint(Time t) override;
        units::V2Velocity getDerivative(Time t) override;
        units::V2Acceleration getSecondDerivative(Time t) override;
        Time GetMaxT() const override;

    private:
        Eigen::Matrix<Number, 4, 2> points;

        Eigen::Matrix<Number, 4, 4> matCoefficients;
        Eigen::Matrix<Number, 3, 4> derivativeCoefficients;
        Eigen::Matrix<Number, 2, 4> secondDerivativeCoefficients;
};

class MultiPath : public Path {
    public:
        MultiPath(std::vector<Path *> paths) : paths(paths) {};
        MultiPath(std::initializer_list<Path *> paths) {
            for (auto p : paths) {
                this->paths.push_back(p);
            }
        };

        units::V2Position getPoint(Time t) override;
        units::V2Velocity getDerivative(Time t) override;
        units::V2Acceleration getSecondDerivative(Time t) override;
        Time GetMaxT() const override;

    private:
        std::vector<Path *> paths;
};

}   // namespace atomic
