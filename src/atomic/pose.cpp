#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "atomic/pose.hpp"

atomic::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

bool atomic::Pose::operator==(const Pose& other) const{
    return this->x == other.x && this->y == other.y &&
           this->theta == other.theta;
}

atomic::Pose atomic::Pose::operator+(const atomic::Pose& other) const {
    return atomic::Pose(this->x + other.x, this->y + other.y, this->theta);
}

atomic::Pose atomic::Pose::operator-(const atomic::Pose& other) const {
    return atomic::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float atomic::Pose::operator*(const atomic::Pose& other) const { return this->x * other.x + this->y * other.y; }

atomic::Pose atomic::Pose::operator*(const float& other) const {
    return atomic::Pose(this->x * other, this->y * other, this->theta);
}

atomic::Pose atomic::Pose::operator/(const float& other) const {
    return atomic::Pose(this->x / other, this->y / other, this->theta);
}

atomic::Pose atomic::Pose::lerp(atomic::Pose other, float t) const {
    return atomic::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float atomic::Pose::distance(atomic::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float atomic::Pose::angle(atomic::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

atomic::Pose atomic::Pose::rotate(float angle) const {
    return atomic::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string atomic::format_as(const atomic::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("atomic::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}