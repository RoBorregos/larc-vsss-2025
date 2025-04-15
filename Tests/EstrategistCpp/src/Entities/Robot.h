#ifndef ROBOT_H
#define ROBOT_H
#include "Entity.h"
#include "Communication.h"

class Robot :public Entity{
    public :
        Communication communication;
        Robot(Transform& t, int i, float f, int port); // transform, ID, impact, port
};



#endif