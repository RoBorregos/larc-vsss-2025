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
}

Transform::Transform(Vector2 p, float r) {
    position = p;
    rotation = r;
}
// This function is to get the data Diference Between two Transforms
Transform Transform::operator-(const Transform& b) const {

    //Verify Format of both angles
    this->CheckAngle();
    b.CheckAngle();
    //Get Vector between the two origins
    Transform t;
    t.position = this->position - b.position;
    t.SetAngule();
    //Verify Integrity again
    t.CheckAngle();
    //Set the angle to be the difference between the two angles
    t.rotation = this->rotation - t.rotation;
    return t;
}
// This function is to check the angle so that is always 0<theta<2pi
void Transform::CheckAngle() const {
    if (rotation > 2*M_PI) {
        rotation -= 2*M_PI;
    }
    if (rotation < 0) {
        rotation += 2*M_PI;
    }
}

void Transform::SetAngule() const {
    rotation = atan2(position.y, position.x);
    CheckAngle();
}