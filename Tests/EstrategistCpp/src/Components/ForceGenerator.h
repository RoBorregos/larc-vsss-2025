#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H
#include "Transform.h"

enum class ForceType
{
    ATRACT,
    REPELENT,
    VORTEX,
    MAGNETIC,
};

class ForceGenerator
{
    public:
    Transform& transform;
    float& impact;
    ForceGenerator(Transform& t, float& i) ;
    ForceGenerator();
    Vector2 GetForce(Transform target, ForceType type);   
    Vector2 GetForce(Transform target, Transform goal, ForceType type, float dist);



    //DataTranfer Override operators
    ForceGenerator& operator=(const ForceGenerator& other);
    ForceGenerator(const ForceGenerator& other) ;
};



#endif