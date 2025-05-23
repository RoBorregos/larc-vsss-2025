#include "ForceGenerator.h"
#include <cmath>
#include <iostream>
using namespace std;
    //functions to generate forces depending on the Force.Type
    // the origine variable defines the object that recives the force
    // the target variable defines the object that creates the force

//------------------FUN Fact: 
//            - The proportional mode is for those objects that we don't want to have a big impact.
//              You just divide the normalized vector by the distance magnitude.
//            - On the other hand, the constant mode is just normalized and is suitable if we want it to have a big impact.


/*
 * This function generates a repellent force vector from the origin to the target.
 * It takes the following parameters:
 *   - target: reference to the Transform receiving the force.
 *   - origin: reference to the Transform generating the force.
 *   - impact: the magnitude of the force.
 *   - mode: the ForceMode (PROPORTIONAL or CONSTANT).
 * Returns a Vector2 representing the repellent force.
 */

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
/*
 * This function generates an attractive force vector from the origin to the target.
 * It takes the following parameters:
 *   - target: reference to the Transform receiving the force.
 *   - origin: reference to the Transform generating the force.
 *   - impact: the magnitude of the force.
 *   - mode: the ForceMode (PROPORTIONAL or CONSTANT).
 * Returns a Vector2 representing the attractive force.
 */
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

        
/*
 * This function generates a vortex (rotational) force vector between the origin and the target.
 * It takes the following parameters:
 *   - target: reference to the Transform receiving the force.
 *   - origin: reference to the Transform generating the force.
 *   - impact: the magnitude of the force.
 *   - mode: the ForceMode (PROPORTIONAL or CONSTANT).
 * Returns a Vector2 representing the vortex force.
 */
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


/*
 * This function creates a new Transform at a specified distance from the current transform towards a goal.
 * It takes the following parameters:
 *   - goal: the target Transform.
 *   - dist: the distance from the current transform to the new position.
 * Returns a Transform at the calculated position.
 */

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

/*
 * This function computes the force vector to apply to a target Transform based on the specified force type and mode.
 * It takes the following parameters:
 *   - target: the Transform to which the force will be applied.
 *   - type: the ForceType (ATRACT, REPELENT, VORTEX).
 *   - mode: the ForceMode (PROPORTIONAL or CONSTANT).
 * Returns a Vector2 representing the computed force.
 */
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

/*
 * This function computes a combined force for magnetic behavior, using both attraction and repellent forces.
 * It takes the following parameters:
 *   - target: the Transform to which the force will be applied.
 *   - goal: the goal Transform used for repellent calculation.
 *   - type: the ForceType (not used directly in this implementation).
 *   - dist: the distance for the repellent calculation.
 *   - mode: the ForceMode for the attractive force.
 * Returns a Vector2 representing the combined force.
 */

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