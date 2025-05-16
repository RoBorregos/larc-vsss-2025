#include "ForceGenerator.h"
#include <cmath>
#include <iostream>
using namespace std;
    //functions to generate forces depending on the Force.Type
    // the origine variable defines the object that recives the force
    // the target variable defines the object that creates the force

    //------------------FUN Fact: 
    //            - El modo proporcional es para aquellos objetos que no queremos que afecten en gran medida. 
    //              Solo es dividir el vector normalizado por la magnitud de distancia
    //            - Caso contrario, el constante es solo normalizado y es justo si queremos que afecte en gran medida

        Vector2 Repelent(Transform& target, Transform& origin, float impact, ForceMode mode){
            Vector2 force, dif;
            dif = origin.position - target.position;
            float difMagnitude = dif.Magnitude();
            dif.Normallize();
            force.x = dif.x;
            force.y = dif.y;
            if(mode == ForceMode::PROPORTIONAL){
                force.x /= difMagnitude;
                force.y /= difMagnitude;
            }
            force *= impact;
            return force;
        }

        Vector2 Atract(Transform& target, Transform& origin, float impact, ForceMode mode){
            Vector2 force, dif;
            dif = target.position - origin.position;
            float difMagnitude = dif.Magnitude();
            dif.Normallize();
            force.x = dif.x ;
            force.y = dif.y ;
            if(mode == ForceMode::PROPORTIONAL){
                force.x /= difMagnitude;
                force.y /= difMagnitude;
            }
            force *= (impact);
            return force;
        }

        Vector2 Vortex(Transform& target, Transform& origin, float impact, ForceMode mode){
            Vector2 force, dif;
            float difMagnitude = (origin.position - target.position).Magnitude();
            force.x = (origin.position.y - target.position.y) / difMagnitude;
            force.y = (target.position.x - origin.position.x) / difMagnitude;
            if(mode == ForceMode::PROPORTIONAL){
                force.x /= difMagnitude;
                force.y /= difMagnitude;
            }
            force *= (impact) *( signbit(origin.angularVelocity) ? 1 : -1);
            return force;
        }


ForceGenerator::ForceGenerator(Transform& t, float &i) : transform(t), impact(i) {
}

ForceGenerator::ForceGenerator() : transform(*new Transform()), impact(*new float(0)) {
}

Transform ForceGenerator::CreatePositionInRect( Transform goal, float dist){
    float m = (transform.position.y - goal.position.y)/(transform.position.x-goal.position.x);
    float d = sqrt(pow(dist,2) / (1+pow(m,2)));
    Vector2 tempPos;
    d = goal.position.x > transform.position.x ? d : -d;
    d = dist > 0 ? d: -d;
    tempPos.x = transform.position.x + d;
    tempPos.y = m*tempPos.x- m*transform.position.x + transform.position.y;
    Transform tempTransform(tempPos, 0);
    return tempTransform;
}


Vector2 ForceGenerator::GetForce(Transform target, ForceType type, ForceMode mode){
    Vector2 force;
    enum ForceType myVar = type;
    switch (static_cast<int>(myVar))
    {
        case static_cast<int>(ForceType::ATRACT):
            force = Atract(transform, target, impact, mode);
            break;
        case static_cast<int>(ForceType::REPELENT):
            force = Repelent(transform, target, impact, mode);
            break;
        case static_cast<int>(ForceType::VORTEX):
            force = Vortex(transform, target, impact, mode );
            break;
        default:
            break;
    }
    return force;
}
// same function as before, just used to select the magnetic force, because it needs more data
Vector2 ForceGenerator::GetForce(Transform target, Transform goal, ForceType type, float dist, ForceMode mode){
    Vector2 force;
    Transform tempPos = CreatePositionInRect( goal, dist);
    force = Atract(transform, target, impact, mode) + Repelent(tempPos, target, impact*0.7, ForceMode::PROPORTIONAL);
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