#include <vector>
#include "atomic/pose.hpp"
#include "atomic/util.hpp"

namespace atomic{

Number atomic::slew(Number target, Number current, Number maxChangeRate, Time deltaTime, SlewDirection restrictDirection) {
    if (maxChangeRate == 0) return target;

    const Number change = target - current;

    // only restrict change for specified directions
    if (restrictDirection == SlewDirection::INCREASING && change < 0) return target;
    if (restrictDirection == SlewDirection::DECREASING && change > 0) return target;

    // check if the change is within the limit
    if (abs(change) > abs(maxChangeRate * to_sec(deltaTime)))
        return current + (maxChangeRate * to_sec(deltaTime) * sgn(change));

    // return the target if no restriction is necessary
    return target;
}

DriveOutputs desaturate(Number lateralOutput, Number angularOutput) {
    const Number left = lateralOutput - angularOutput;
    const Number right = lateralOutput + angularOutput;
    const Number sum = abs(left) + abs(right);
    if (sum <= 1.0) return {left, right};
    else return {left / sum, right / sum};
}

Curvature getSignedTangentArcCurvature(units::Pose start, V2Position end) {
    // whether the pose is on the left or right side of the arc
    const V2Position delta = end - start;
    const Number side = sgn(sin(start.orientation) * delta.x - cos(start.orientation) * delta.y).internal();
    // calculate center point and radius
    const Number a = -tan(start.orientation);
    const Length c = tan(start.orientation) * start.x - start.y;
    const Length x = abs(a * end.x + end.y + c) / sqrt(a * a + 1);
    const Length d = start.distanceTo(end);
    // return the curvature
    return side * ((2 * x) / (d * d));
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


constexpr Number atomic::clamp(Number input, Number min, Number max){
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


Angle angleError(Angle target, Angle position, std::optional<AngularDirection> direction) {
    // Wrap the angle to be within 0pi and 2pi radians
    target = mod(mod(target, 1_stRot) + 1_stRot, 1_stRot);

    Angle error = target - position;
    if (!direction) return from_stDeg(std::remainder(to_stDeg(error), 360));
    if (direction == AngularDirection::CW_CLOCKWISE) return error < 0_stRot ? error + 1_stRot : error;
    else return error > 0_stRot ? error - 1_stRot : error;
}

constexpr Angle atomic::reduce_0_to_360(Angle angle){
    while(!(angle.internal() >= 0 && angle.internal() < 360)) {
        if( angle.internal() < 0 ) {
            angle += 360 * deg; 
        }else if(angle.internal() >= 360) {
            angle -= 360 * deg; 
        }
    }
    
    return angle;
}

constexpr Angle atomic::reduce_negative_pi_to_pi(Angle angle){
    while(!(angle.internal() >= -M_PI && angle.internal() < M_PI)) {
        if( angle.internal() < -M_PI )
        { 
            angle += 2*M_PI*rad; 
        }else if(angle.internal() >= M_PI) { 
            angle -= 2*M_PI*rad; 
        }
    }

    return angle;
}

constexpr Angle atomic::reduce_negative_180_to_180(Angle angle){
    while(!(angle.internal() >= -180 && angle.internal() < 180)) {
        if( angle.internal() < -180 )
        { 
            angle += 360 * deg; 
        }else if(angle.internal() >= 180) { 
            angle -= 360 * deg; 
        }
    }

    return angle;
}

constexpr Angle atomic::reduce_negative_90_to_90(Angle angle){
    while(!(angle.internal() >= -90 && angle.internal() < 90)) {
        if( angle.internal() < -90 )
        { 
            angle += 180 * deg; 
        }else if(angle.internal() >= 90) { 
            angle -= 180 * deg; 
        }
    }

    return angle;
}

}   // namespace atomic
