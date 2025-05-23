#include "Entity.h"

Entity::Entity(Transform &t, int i, float f, int portR, int portS) : transform(t), 
                 communication(transform, i, portR, portS), 
                 forceGenerator(transform, impact) {
    ID = i;
    impact = f;
}