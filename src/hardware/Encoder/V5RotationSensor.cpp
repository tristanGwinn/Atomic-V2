#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Port.hpp"
#include "hardware/util.hpp"
#include "pros/rotation.hpp"
#include <limits.h>
#include <mutex>

namespace atomic {
V5RotationSensor::V5RotationSensor(ReversibleSmartPort port)
    : m_port(abs(port)),
      m_reversed(port < 0) {
    pros::c::rotation_set_reversed(m_port, m_reversed);
}

V5RotationSensor::V5RotationSensor(const V5RotationSensor& other)
    : m_port(other.m_port),
      m_reversed(other.m_reversed),
      m_offset(other.m_offset) {}

V5RotationSensor V5RotationSensor::from_pros_rot(pros::Rotation encoder) {
    if (encoder.get_reversed()) return V5RotationSensor {{-encoder.get_port(), runtime_check_port}};
    else return V5RotationSensor {{encoder.get_port(), runtime_check_port}};
}

int32_t V5RotationSensor::isConnected() const {
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return 0;
    if (pros::c::rotation_get_angle(m_port) == INT_MAX) return 0;
    else return 1;
}

Angle V5RotationSensor::getAngle() const {
    std::lock_guard lock(m_mutex);
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return from_stRot(INFINITY);
    const int32_t raw = pros::c::rotation_get_position(m_port);
    if (raw == INT_MAX) return from_stRot(INFINITY);
    // the rotation sensor returns centidegrees, so we have to convert to degrees
    const Angle angle = from_stDeg(double(raw) / 100);
    return angle + m_offset;
}

int32_t V5RotationSensor::setAngle(Angle angle) {
    std::lock_guard lock(m_mutex);
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return INT_MAX;
    // requestedAngle = pos + offset
    // offset = requestedAngle - raw
    const int32_t raw = pros::c::rotation_get_position(m_port);
    if (raw == INT_MAX) return INT_MAX;
    // the rotation sensor returns centidegrees, so we have to convert to degrees
    const Angle position = from_stDeg(double(raw) / 100);
    m_offset = angle - position;
    return 0;
}

int32_t V5RotationSensor::isReversed() const {
    std::lock_guard lock(m_mutex);
    return m_reversed;
}

int32_t V5RotationSensor::setReversed(bool reversed) {
    std::lock_guard lock(m_mutex);
    m_reversed = reversed;
    return convertStatus(pros::c::rotation_set_reversed(m_port, m_reversed));
}
} // namespace atomic