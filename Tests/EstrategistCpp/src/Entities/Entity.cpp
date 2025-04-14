#include "Entity.h"

Entity::Entity(Transform &t, int i, float f) : transform(t), 
                 kinematic(transform), 
                 forceGenerator(transform, impact) {
    ID = i;
    impact = f;
}