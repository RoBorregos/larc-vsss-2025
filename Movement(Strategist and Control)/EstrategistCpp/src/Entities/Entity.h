#ifndef ENTITY_H
#define ENTITY_H
#include "Communication.h"
#include "ForceGenerator.h"


// Represents a general entity in the system, such as a robot or a ball.
// This class encapsulates the entity's transform, communication, force generation, and other properties.
class Entity
{
    public:
    Transform& transform; // Reference to the entity's Transform object, representing its position and rotation.
    Communication communication; // Handles communication with the entity (e.g., sending/receiving data).
    ForceGenerator forceGenerator; // Generates forces acting on the entity (e.g., attraction, repulsion).
    float impact; // Impact factor used in the ForceGenerator to scale forces.
    int ID; // entities [negatives] are enemies and [positives] are allies [zero] is for the ball
    Entity(Transform& t, int i, float f, int portR, int portS);

};


#endif