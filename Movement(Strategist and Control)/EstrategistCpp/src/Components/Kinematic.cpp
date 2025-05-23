#include "Kinematic.h"
#include <cmath>
constexpr float pi = 3.141592653589793f;

// Sets the Transform's position and rotation.
// Ensures the rotation is within the valid range [0, 2π].
Kinematic::Kinematic(Transform &t) : transform(t) {
    CIRCUMFERENCE = 2 * 3.14f * RADIUS;
}

/*
 * This function computes the wheel velocities (in RPM) required to rotate the robot to a target orientation,
 * without considering translational movement.
 * It takes the following parameter:
 *   - target: a Transform object representing the desired orientation.
 * Returns an Output object with the left and right wheel velocities.
 */
Output Kinematic::GetVelocitiesForRotation(Transform target){
    Output output;
    target.rotation = transform.GetRotationalDifference(target.rotation); 
    bool changeReference = abs(target.rotation) > pi/2;
    if (changeReference){
        target.rotation =( signbit(target.rotation) ? -1 : 1 ) * pi - target.rotation;
    }
    float RotationalVel = target.rotation * ANGULAR_CONSTANT*0.5;
    float leftVel = - WHEEL_DISTANCE / (2*RADIUS) * RotationalVel; 
    float rightVel = + WHEEL_DISTANCE / (2*RADIUS) * RotationalVel; 

    output.a = leftVel * 60 / CIRCUMFERENCE;
    output.b = rightVel * 60 / CIRCUMFERENCE;

    if(changeReference){
        output.a = -output.a;
        output.b = -output.b;
    }
    return output;


}

/*
 * This function computes the wheel velocities (in RPM) required to move the robot toward a target Transform.
 * It does not consider the current orientation of the robot.
 * It takes the following parameter:
 *   - target: a Transform object representing the target position and orientation.
 * Returns an Output object with the left and right wheel velocities.
 */
Output Kinematic::GetVelocities(Transform target) {
    Output output;
    //Get the difference between the two transforms.
    Transform t = target - transform;
    bool changeReference = abs(t.rotation) > pi/2+0.5;
    if (changeReference){
        t.rotation =( signbit(t.rotation) ? -1 : 1 ) * pi - t.rotation;
        cout<<"rotation: "<<t.rotation<<endl;
    }
    // Calculate the forward velocity and rotational velocity.
    float FrontVel = t.position.Magnitude() * LINEAR_CONSTANT*0.08f;
    float RotationVel = t.rotation * ANGULAR_CONSTANT * 2;
    //Get the velocities of the wheels in rad/s
    float leftVel = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    float rightVel = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    //Convert to RPM
    output.a = leftVel * 60 / CIRCUMFERENCE;
    output.b = rightVel * 60 / CIRCUMFERENCE;
    if(changeReference){
        output.a = -output.a;
        output.b = -output.b;
    }

    return output;
}
/*
 * This function computes the wheel velocities (in RPM) required to move the robot toward a target global velocity vector.
 * It takes the following parameter:
 *   - target: a Vector2 object representing the desired global velocity.
 * Returns an Output object with the left and right wheel velocities.
 */
Output Kinematic::GetVelocities(Vector2 target) {
    Output output;
    // Calculate the forward velocity and rotational velocity.
    float FrontVel = target.Magnitude() * LINEAR_CONSTANT *1.2;
    float RotationVel = transform.GetRotationalDifference(target.GetAngle()) * ANGULAR_CONSTANT * 1.5 ; 
    //Get the velocities of the wheels in rad/s
    output.a = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    output.b = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    
    //Convert to RPM
    output.a = output.a * 60 / CIRCUMFERENCE;
    output.b = output.b * 60 / CIRCUMFERENCE;

    return output;
}




//Functions to cpy only the valuable objects
Kinematic::Kinematic(const Kinematic& other) : transform(other.transform) {
    CIRCUMFERENCE = other.CIRCUMFERENCE;
    ANGULAR_CONSTANT = other.ANGULAR_CONSTANT;
    LINEAR_CONSTANT = other.LINEAR_CONSTANT;
    RADIUS = other.RADIUS;
    WHEEL_DISTANCE = other.WHEEL_DISTANCE;
    transform = other.transform;
}
// Copies all relevant member variables and returns a reference to this object.
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