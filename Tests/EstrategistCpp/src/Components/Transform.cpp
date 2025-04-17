#include "Transform.h"
#define _USE_MATH_DEFINES
#include <cmath>


using namespace std;
Transform::Transform() {
    position = Vector2();
    rotation = 0.0f;
}

Transform::Transform(float x, float y, float r) {
    position = Vector2(x, y);
    rotation = r;
    CheckAngle();  // Ensure the rotation is within the valid range [0, 2π].
}

Transform::Transform(Vector2 p, float r) {
    position = p;
    rotation = r;
}
// This function is to get the data Diference Between two Transforms
Transform Transform::operator-(const Transform& b) const {

    //Verify both angles are within the valid range
    this->CheckAngle();
    b.CheckAngle();
    //Get Vector between the two origins
    Transform t;
    t.position =  b.position - this->position ;
    t.SetAngule();     // Set the angle of the resulting Transform based on its position.
    t.CheckAngle();    // Enzure the resulting angle is valid
    t.rotation = GetRotationalDifference(b.rotation); // compute the rotational difference between the actual rotation and the b.rotation
    return t;
}
// Ensures the rotation angle is within the range [0, 2π].
void Transform::CheckAngle() const {
    if (rotation > 2*M_PI) {
        rotation -= 2*M_PI;
    }
    if (rotation < 0) {
        rotation += 2*M_PI;
    }
}

// Sets the rotation angle based on the position vector's angle.
void Transform::SetAngule() const {
    rotation = position.GetAngle();
    CheckAngle();
}



// Computes the shortest rotational difference between the current rotation and the target rotation.
// Returns the difference in radians, accounting for wrap-around at 2π.
float Transform::GetRotationalDifference(float objective) const{
    float dif = objective - rotation;
    if(abs(dif) < M_PI){
        return dif; // If the difference is less than π, return it directly.
    }else{
        return (signbit(dif) ? 1 : -1) * 2*M_PI + dif;    // Adjust for wrap-around at 2π.
    }
}


// Sets the Transform's position and rotation.
// Ensures the rotation is within the valid range [0, 2π].
void Transform::SetTransform(float x, float y, float r) {
    position.x = x;
    position.y = y;
    rotation = r;
    CheckAngle();
}

ostream& operator<<(ostream& os, const Transform& t) {
    os << "Position: " << t.position << " Angle: " << t.rotation;
    return os;
}