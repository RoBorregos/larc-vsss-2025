#include "Kinematic.h"


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

Output Kinematic::GetVelocities(Vector2 target) {
    Output output;
    //Get the target magnitude of the target and get the angular 
    //difference and multiply by the constants
    Transform t = Transform(target, 0);
    t.SetAngule();
    t = t - transform;
    float FrontVel = target.Magnitude() * LINEAR_CONSTANT;
    float RotationVel = t.rotation * ANGULAR_CONSTANT;
    //Get the velocities of the wheels in rad/s
    float leftVel = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    float rightVel = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    
    //Convert to RPM
    output.a = leftVel * 60 / CIRCUMFERENCE;
    output.b = rightVel * 60 / CIRCUMFERENCE;

    return output;
}

Kinematic::Kinematic(const Kinematic& other) : transform(other.transform) {
    CIRCUMFERENCE = other.CIRCUMFERENCE;
    ANGULAR_CONSTANT = other.ANGULAR_CONSTANT;
    LINEAR_CONSTANT = other.LINEAR_CONSTANT;
    RADIUS = other.RADIUS;
    WHEEL_DISTANCE = other.WHEEL_DISTANCE;
    transform = other.transform;
}

Kinematic& Kinematic::operator=(const Kinematic& other) {
    if (this != &other) {
        transform = other.transform;
        CIRCUMFERENCE = other.CIRCUMFERENCE;
        ANGULAR_CONSTANT = other.ANGULAR_CONSTANT;
        LINEAR_CONSTANT = other.LINEAR_CONSTANT;
        RADIUS = other.RADIUS;
        WHEEL_DISTANCE = other.WHEEL_DISTANCE;
        transform = other.transform;
    }
    return *this;
}