#include "atomic/chassis/trackingWheel.hpp"
#include "atomic/util.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Encoder/ADIEncoder.hpp"

atomic::TrackingWheel::TrackingWheel(Encoder* encoder, Length diameter, Length offset, Number ratio)
    : m_encoder(encoder),
      m_diameter(diameter),
      m_offset(offset),
      m_ratio(ratio),
      m_lastTotal(to_stRot(encoder->getAngle()) * M_PI * diameter * m_ratio) {}

atomic::TrackingWheel::TrackingWheel(ReversibleSmartPort port, Length diameter, Length offset, Number ratio)
    : m_encoder(new V5RotationSensor(port)),
      m_diameter(diameter),
      m_offset(offset),
      m_ratio(ratio),
      m_lastTotal(to_stRot(m_encoder->getAngle()) * M_PI * diameter * m_ratio) {}
    
int atomic::TrackingWheel::reset() { return m_encoder->setAngle(0_stDeg); }

Length atomic::TrackingWheel::getDistanceDelta() {
    // calculate delta
    const Length total = this->getDistanceTraveled();
    const Length delta = total - m_lastTotal;
    m_lastTotal = total;
    // return the delta
    return delta;
}

Length atomic::TrackingWheel::getDistanceTraveled() { return to_stRot(m_encoder->getAngle()) * M_PI * m_diameter * m_ratio; }

Length atomic::TrackingWheel::getOffset() { return m_offset; }
