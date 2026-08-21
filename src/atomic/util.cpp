#include <vector>
#include "atomic/pose.hpp"
#include "atomic/util.hpp"

// wait until a condition is met
#define waitUntil(condition)                                                   \
  do {                                                                         \
    pros::delay(5);                                                            \
  } while (!(condition))

namespace atomic{

float atomic::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float atomic::random_float(float min, float max){
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = max - min;
    float r = random * diff;
    return min + r;
}

float atomic::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

float atomic::ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}


// i mostly stole this from echo cause i dont know a better way to calculate this
float atomic::cheap_norm_pdf(const float x){        // Approximation of the standard normal PDF
    // Coefficients for the rational approximation
    const float a = 0.3989422804014337; // 1 / sqrt(2 * PI)
    const float e = 0.59422804014337;   // magic number

    // Compute the approximate normal PDF using a rational polynomial
    const float pdfApprox = a / (1.0 + e * x * x * x * x);

    return pdfApprox;
}


constexpr float atomic::clamp(float input, float min, float max){
    if(input > max){
        return max;
    }else if (min > input)
    {
        return min;
    }else{
        return input;
    }
}

constexpr float atomic::clamp_min_voltage(float drive_output, float drive_min_voltage){
    if(drive_output < 0 && drive_output > -drive_min_voltage){
        return -drive_min_voltage;
    }
    if(drive_output > 0 && drive_output < drive_min_voltage){
        return drive_min_voltage;
    }
    return drive_output;
}

constexpr float atomic::percent_to_volts(float percent){
   return (127 * percent);
}

constexpr float atomic::volts_to_percent(float volts){
   return (volts / 127);
}


float atomic::angleError(float target, float position, bool radians, AngularDirection direction) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}

constexpr float atomic::sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

constexpr float atomic::reduce_0_to_360(float angle){
    while(!(angle >= 0 && angle < 360)) {
        if( angle < 0 ) {
            angle += 360; 
        }else if(angle >= 360) {
            angle -= 360; 
        }
    }
    
    return angle;
}

constexpr float atomic::reduce_negative_pi_to_pi(float angle){
    while(!(angle >= -M_PI && angle < M_PI)) {
        if( angle < -M_PI )
        { 
            angle += 2*M_PI; 
        }else if(angle >= M_PI) { 
            angle -= 2*M_PI; 
        }
    }

    return angle;
}

constexpr float atomic::reduce_negative_180_to_180(float angle){
    while(!(angle >= -180 && angle < 180)) {
        if( angle < -180 )
        { 
            angle += 360; 
        }else if(angle >= 180) { 
            angle -= 360; 
        }
    }

    return angle;
}

constexpr float atomic::reduce_negative_90_to_90(float angle){
    while(!(angle >= -90 && angle < 90)) {
        if( angle < -90 )
        { 
            angle += 180; 
        }else if(angle >= 90) { 
            angle -= 180; 
        }
    }

    return angle;
}

}   // namespace atomic
