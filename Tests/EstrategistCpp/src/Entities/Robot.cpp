#include "Robot.h"


Robot::Robot(Transform& t, int i, float f) : Entity(t, i, f),
    communication(Entity::ID) {
    ID = i;
}
