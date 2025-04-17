#include "Entity.h"

Entity::Entity(Transform &t, int i, float f, int port) : transform(t), 
                 communication(transform, i, port), 
                 forceGenerator(transform, impact) {
    ID = i;
    impact = f;
}