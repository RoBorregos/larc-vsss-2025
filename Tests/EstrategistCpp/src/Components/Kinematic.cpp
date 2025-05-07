#include "Kinematic.h"
#include <cmath>
constexpr float pi = 3.141592653589793f;

// Sets the Transform's position and rotation.
// Ensures the rotation is within the valid range [0, 2π].
Kinematic::Kinematic(Transform &t) : transform(t) {
    CIRCUMFERENCE = 2 * 3.14f * RADIUS;
}



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

// Function: GetVelocities
// Computes the wheel velocities (in RPM) required to move the robot from its current transform
// to the target transform.

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
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    float RotationVel = t.rotation * ANGULAR_CONSTANT * 2;
    //     Constante angular de robot bidireccional   ^   cambiar para que si se mueva bien
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
// Function: GetVelocities
// Computes the wheel velocities (in RPM) required to move the robot toward a target global velocity.
Output Kinematic::GetVelocities(Vector2 target) {
    Output output;
    //Get the target magnitude of the target and get the angular 
    //difference and multiply by the constants
    // Calculate the forward velocity and rotational velocity.
    cout<<"                     Angle Dif:"<<transform.GetRotationalDifference(target.GetAngle()) <<endl;
    float FrontVel = target.Magnitude() * LINEAR_CONSTANT *1.5;
                                                           // 2.2
    float RotationVel = transform.GetRotationalDifference(target.GetAngle()) * ANGULAR_CONSTANT * 0.5 ; //0.8
    //Get the velocities of the wheels in rad/s
    output.a = FrontVel/ RADIUS - WHEEL_DISTANCE / (2*RADIUS) * RotationVel; 
    output.b = FrontVel/ RADIUS + WHEEL_DISTANCE / (2*RADIUS) * RotationVel;
    
    //Convert to RPM
    
    output.a = output.a * 60 / CIRCUMFERENCE;
    output.b = output.b * 60 / CIRCUMFERENCE;

    return output;
}

Output Kinematic::GetVelocitiesForMagn(Vector2 target) {
    Output output;
    //Get the target magnitude of the target and get the angular 
    //difference and multiply by the constants
    // Calculate the forward velocity and rotational velocity.
    cout<<"                     Angle Dif:"<<transform.GetRotationalDifference(target.GetAngle()) <<endl;
    float FrontVel = target.Magnitude() * LINEAR_CONSTANT *1.5;
    float RotationVel = transform.GetRotationalDifference(target.GetAngle()) * ANGULAR_CONSTANT * 0.5;
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