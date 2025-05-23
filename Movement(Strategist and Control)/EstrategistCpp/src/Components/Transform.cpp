#include "Transform.h"
#include <math.h>
constexpr double pi = 3.141592653589793;


using namespace std;
Transform::Transform() {
    position = Vector2();
    velocities =Vector2();
    rotation = 0.0f;
    angularVelocity = 0.0f;
}
/*
 * Constructor for the Transform class with position (x, y) and rotation r.
 * It takes the following parameters:
 *   - x: the x-coordinate of the position.
 *   - y: the y-coordinate of the position.
 *   - r: the rotation angle in radians.
 * Initializes velocities to zero and ensures the rotation is within [0, 2π].
 */
Transform::Transform(float x, float y, float r) {
    position = Vector2(x, y);
    velocities = Vector2();
    rotation = r;
    CheckAngle();  // Ensure the rotation is within the valid range [0, 2π].
}

Transform::Transform(Vector2 p, float r) {
    position = p;
    rotation = r;
}
/*
 * Operator overload to compute the difference between two Transform objects.
 * It takes the following parameter:
 *   - b: the Transform to subtract from the current Transform.
 * Returns a new Transform respresenting the difference in coordinates and the difference between the angle that this vector represents and the transform angle
 */
Transform Transform::operator-(const Transform& b) const {

    //Verify both angles are within the valid range
    this->CheckAngle();
    b.CheckAngle();
    //Get Vector between the two transforms
    Transform t;
    t.position =  this->position - b.position ;
    t.SetAngule();     
    t.CheckAngle();   
    t.rotation = b.GetRotationalDifference(t.rotation); // compute the rotational difference between the actual rotation
                                                        // and the rotation of the objective vector
    return t;
}

/*
 * Ensures the rotation angle is within the range [0, 2π].
 * Modifies the rotation value if it is outside this range.
 */
void Transform::CheckAngle() const {
    if (rotation > 2*pi) {
        rotation -= 2*pi;
    }
    if (rotation < 0) {
        rotation += 2*pi;
    }
}

/*
 * Sets the rotation angle based on the position vector's angle.
 * Calls CheckAngle() to ensure the rotation is valid.
 */
void Transform::SetAngule() const {
    rotation = position.GetAngle();
    CheckAngle();
}


/*
 * Computes the shortest rotational difference between the current rotation and the target rotation.
 * It takes the following parameter:
 *   - objective: the target rotation angle in radians.
 * Returns the difference in radians, accounting for wrap-around at 2π.
 */

float Transform::GetRotationalDifference(float objective) const{
    CheckAngle();
    
    float dif = objective - rotation;
    if(abs(dif) < pi){
        return dif; // If the difference is less than π, return it directly.
    }else{
        return (signbit(dif) ? 1 : -1) * 2*pi + dif;    // Adjust for wrap-around at 2π.
    }
}

/*
 * Sets the Transform's position and rotation using x, y, and r values.
 * It takes the following parameters:
 *   - x: the new x-coordinate (scaled by 1/10).
 *   - y: the new y-coordinate (scaled by -1/10).
 *   - r: the new rotation angle.
 * Also updates velocities and angularVelocity, and ensures the rotation is valid.
 */
void Transform::SetTransform(float x, float y, float r) {
    x = x/10; y /=-10; r = +- r;
    velocities =  Vector2(x,y) - position;
    velocities = velocities.Magnitude() < 0.2? Vector2(0,0) : velocities;
    angularVelocity = r - rotation > 0.01 ? r - rotation : 0;
    position.x = x ;
    position.y = y;
    rotation =  r;
    CheckAngle();
}
void Transform::SetTransform(Vector2 p, float r) {
    position = p;
    rotation = r;
    CheckAngle();
}

ostream& operator<<(ostream& os, const Transform& t) {
    os << "Position: " << t.position << " Angle: " << t.rotation;
    return os;
}