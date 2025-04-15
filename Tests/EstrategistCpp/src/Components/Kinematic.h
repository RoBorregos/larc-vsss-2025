#ifndef KINEMATIC_H
#define KINEMATIC_H
#include "Transform.h"
#include "Output.h"
//clase para crear los outputs necesarios para el robot. 
// La info que genera esta hecha para mandar directamente a el robot
class Kinematic
{
    public:
        Transform &transform;
        //Constantes 
        float ANGULAR_CONSTANT = 0.66f;
        float LINEAR_CONSTANT = 0.33f;
        float RADIUS = 0.03f;
        float WHEEL_DISTANCE = 0.076f;
        float CIRCUMFERENCE;
        Kinematic(Transform &t);
        //Funiones para obtener la velocidad de las llantas dependie si es objetivo o velocidad
        Output GetVelocities(Transform target);
        Output GetVelocities(Vector2 target);

        Kinematic(const Kinematic& other) ;
        Kinematic& operator=(const Kinematic& other) ;
        //Optionaly I can add odometry function to not just send an static rpm


        
};

#endif