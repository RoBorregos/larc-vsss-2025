#include "Kinematic.h"

Kinematic::Kinematic(): transform(*new Transform()) {
    CIRCUMFERENCE = 2 * 3.14f * RADIUS;
}

Kinematic::Kinematic(Transform &t) : transform(t) {
    CIRCUMFERENCE = 2 * 3.14f * RADIUS;
}

Output Kinematic::GetVelocities(Transform target) {
    Output output;
    //Get the difference between the two transforms and multiply by the constants
    Transform t = target - transform;
    float FrontVel = t.position.Magnitude() * LINEAR_CONSTANT;
    float RotationVel = t.rotation * ANGULAR_CONSTANT;
    //Get the velocities of the wheels in rad/s
    float leftVel = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    float rightVel = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    
    //Convert to RPM
    output.a = leftVel * 60 / CIRCUMFERENCE;
    output.b = rightVel * 60 / CIRCUMFERENCE;

    return output;
}