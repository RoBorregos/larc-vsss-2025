#include "ForceGenerator.h"
#include <cmath>
#include <iostream>
using namespace std;
//functions depending on the case

Vector2 Repelent(Transform& origin, Transform& target, float impact){
    Vector2 force, dif;
    dif = origin.position - target.position;
    float difMagnitude = dif.Magnitude();
    dif.Normallize();
    force.x = dif.x/ difMagnitude;
    force.y = dif.y / difMagnitude;
    force *= impact;
    return force;
}

Vector2 Atract(Transform& origin, Transform& target, float impact){
    Vector2 force, dif;
    dif = target.position - origin.position;
    float difMagnitude = dif.Magnitude();
    dif.Normallize();
    force.x = dif.x / difMagnitude;
    force.y = dif.y/ difMagnitude;
    force *= (impact);
    return force;
}

Vector2 Vortex(Transform& origin, Transform& target, float impact){
    Vector2 force, dif;
    float difMagnitude = (origin.position - target.position).Magnitude();
    force.x = (origin.position.y - target.position.y) / difMagnitude/difMagnitude;
    force.y = (target.position.x - origin.position.x) / difMagnitude/difMagnitude;
    force *= (impact);
    return force;
}

Vector2 Magnetic(Transform& origin, Transform& target, Transform& goal, float impact, float dist){
    float m = (goal.position.y - target.position.y)/(goal.position.x-target.position.x);
    float d = sqrt(pow(dist,2) / (1+pow(m,2)));
    Vector2 tempPos;
    tempPos.x = target.position.x + d;
    tempPos.y = m*tempPos.x- m*target.position.x + target.position.y;
    Transform tempTransform(tempPos, 0);
    Vector2 force;
    force = Atract(origin, target, impact) + Repelent(origin, tempTransform, impact*0.5f);
    return force;
}

ForceGenerator::ForceGenerator(Transform& t, float &i) : transform(t), impact(i) {
}

ForceGenerator::ForceGenerator() : transform(*new Transform()), impact(*new float(0)) {
}

Vector2 ForceGenerator::GetForce(Transform target, ForceType type){
    Vector2 force;
    enum ForceType myVar = type;
    switch (static_cast<int>(myVar))
    {
        case static_cast<int>(ForceType::ATRACT):
            force = Atract(transform, target, impact);
            break;
        case static_cast<int>(ForceType::REPELENT):
            force = Repelent(transform, target, impact);
            break;
        case static_cast<int>(ForceType::VORTEX):
            force = Vortex(transform, target, impact);
            break;
        default:
            break;
    }
    return force;
}

Vector2 ForceGenerator::GetForce(Transform target, Transform goal, ForceType type, float dist){
    Vector2 force;
    force = Magnetic(transform, target, goal, impact, dist);
    return force;
}



ForceGenerator& ForceGenerator::operator=(const ForceGenerator& other) {
    if (this != &other) {
        transform = other.transform;
        impact = other.impact;
    }
    return *this;
}
ForceGenerator::ForceGenerator(const ForceGenerator& other) : transform(other.transform), impact(other.impact) {
}