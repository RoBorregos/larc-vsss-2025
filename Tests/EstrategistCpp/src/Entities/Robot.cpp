#include "Robot.h"


Robot::Robot(Transform& t, int i, float f, int portR, int portS) : Entity(t, i, f, portR, portS), kinematic(t){
    ID = i;
}
