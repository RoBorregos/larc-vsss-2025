#include "Kinematic.h"

// Sets the Transform's position and rotation.
// Ensures the rotation is within the valid range [0, 2π].
Kinematic::Kinematic(Transform &t) : transform(t) {
    CIRCUMFERENCE = 2 * 3.14f * RADIUS;
}



// Function: GetVelocities
// Computes the wheel velocities (in RPM) required to move the robot from its current transform
// to the target transform.
// Parameters:
// - target: The target Transform object.
// Returns:
// - Output object containing the left and right wheel velocities in RPM.
Output Kinematic::GetVelocities(Transform target) {
    Output output;
    //Get the difference between the two transforms.
    Transform t = target - transform;
    // Calculate the forward velocity and rotational velocity.
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

// Function: GetVelocities
// Computes the wheel velocities (in RPM) required to move the robot toward a target global velocity.
// Parameters:
// - target: The target velocity as a Vector2 object.
// Returns:
// - Output object containing the left and right wheel velocities in RPM.
Output Kinematic::GetVelocities(Vector2 target) {
    Output output;
    //Get the target magnitude of the target and get the angular 
    //difference and multiply by the constants

    // Calculate the forward velocity and rotational velocity.
    float FrontVel = target.Magnitude() * LINEAR_CONSTANT;
    float RotationVel = transform.GetRotationalDifference(target.GetAngle()) * ANGULAR_CONSTANT;
    //Get the velocities of the wheels in rad/s
    output.a = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    output.b = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    
    //Convert to RPM
    
    output.a = output.a * 60 / CIRCUMFERENCE;
    output.b = output.b * 60 / CIRCUMFERENCE;

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