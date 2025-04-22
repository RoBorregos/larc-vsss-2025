#include "Robot.h"


Robot::Robot(Transform& t, int i, float f, int port) : Entity(t, i, f, port), kinematic(t){
    ID = i;
}
