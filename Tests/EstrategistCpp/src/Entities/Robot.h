#ifndef ROBOT_H
#define ROBOT_H
#include "Entity.h"
#include "Communication.h"
#include <Kinematic.h>

// Represents a robot entity in the system.
// Inherits from the Entity class and adds kinematic properties for movement control.
class Robot :public Entity{
    public :
        Kinematic kinematic; //Handles the robots kinematics. Generates the necesary rpm to achieve different objectives
        Robot(Transform& t, int i, float f, int portR, int portS); // transform, ID, impact, port
};



#endif