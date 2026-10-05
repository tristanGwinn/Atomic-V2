#pragma once

#include "Eigen/Dense"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "trajectory.h"

class Path {
    public:
        virtual units::V2Position getPoint(double t) = 0;
        virtual units::V2Velocity getDerivative(double) = 0;
        virtual units::V2Acceleration getSecondDerivative(double t) = 0;
        virtual double GetMaxT() const = 0;
        virtual Length GetLength() const = 0;
        virtual ~Path() = default;
};

class CubicBezier : public Path {
    public:
        CubicBezier(units::V2Position p0, units::V2Position p1,
                    units::V2Position p2, units::V2Position p3);
        units::V2Position getPoint(double t) override;
        units::V2Velocity getDerivative(double t) override;
        units::V2Acceleration getSecondDerivative(double t) override;
        double GetMaxT() const override;
        Length GetLength() const override;

    private:
        Length path_length = 0_m;

        Eigen::Matrix<double, 4, 2> points;

        Eigen::Matrix<double, 4, 4> matCoefficients;
        Eigen::Matrix<double, 3, 4> derivativeCoefficients;
        Eigen::Matrix<double, 2, 4> secondDerivativeCoefficients;
};

class MultiPath : public Path {
    public:
        MultiPath(std::vector<Path *> paths) : paths(paths) {};
        MultiPath(std::initializer_list<Path *> paths) {
            for (auto p : paths) {
                this->paths.push_back(p);
            }
        };

        units::V2Position getPoint(double t) override;
        units::V2Velocity getDerivative(double t) override;
        units::V2Acceleration getSecondDerivative(double t) override;
        double GetMaxT() const override;
        Length GetLength() const override;

    private:
        std::vector<Path *> paths;
};
