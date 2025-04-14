#ifndef ENTITY_H
#define ENTITY_H
#include "Kinematic.h"
#include "ForceGenerator.h"

class Entity
{
    public:
    Transform& transform;
    Kinematic kinematic;
    ForceGenerator forceGenerator;
    float impact;
    int ID; // entities [negatives] are enemies and [positives] are allies
    Entity(Transform& t, int i, float f);

};


#endif