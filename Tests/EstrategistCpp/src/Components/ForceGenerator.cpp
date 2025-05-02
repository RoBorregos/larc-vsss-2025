#include "ForceGenerator.h"
#include <cmath>
#include <iostream>
using namespace std;
    //functions to generate forces depending on the Force.Type
    // the origine variable defines the object that recives the force
    // the target variable defines the object that creates the force

        Vector2 Repelent(Transform& target, Transform& origin, float impact){
            Vector2 force, dif;
            dif = origin.position - target.position;
            float difMagnitude = dif.Magnitude();
            dif.Normallize();
            force.x = dif.x/ difMagnitude;
            force.y = dif.y / difMagnitude;
            force *= impact;
            return force;
        }

        Vector2 Atract(Transform& target, Transform& origin, float impact){
            Vector2 force, dif;
            dif = target.position - origin.position;
            dif.Normallize();
            force.x = dif.x;
            force.y = dif.y;
            force *= (impact);
            return force;
        }

        Vector2 Vortex(Transform& target, Transform& origin, float impact){
            Vector2 force, dif;
            float difMagnitude = (origin.position - target.position).Magnitude();
            force.x = (origin.position.y - target.position.y) / difMagnitude;
            force.y = (target.position.x - origin.position.x) / difMagnitude;
            force *= (impact) *( signbit(origin.angularVelocity) ? 1 : -1);
            return force;
        }
        // In this case, to create a magnetic field, I have to set a second transform that would act as the position of the goal
        // so at the end the the magnetic field will always target towards the goal
        /* 
            B######     ball
            #\#####         
            ##T####     tempPos
            ###\###
            ####\##
            #####\#
            ######G     Goal
        */
        Vector2 Magnetic(Transform& target, Transform& origin, Transform& goal, float impact, float dist){
            float m = (goal.position.y - target.position.y)/(goal.position.x-target.position.x);
            float d = sqrt(pow(dist,2) / (1+pow(m,2)));
            Vector2 tempPos;
            d = goal.position.x > target.position.x ? d : -d;
            tempPos.x = target.position.x + d;
            tempPos.y = m*tempPos.x- m*target.position.x + target.position.y;
            Transform tempTransform(tempPos, 0);
//            cout<<"PosTemp"<<tempTransform<<endl;
            Vector2 force;
            force = Atract(origin, target, impact) + Repelent(origin, tempTransform, impact*0.6f); // combine the attractive and repelent forces to simulate a magnetic field
            return force;
        }
    //

ForceGenerator::ForceGenerator(Transform& t, float &i) : transform(t), impact(i) {
}

ForceGenerator::ForceGenerator() : transform(*new Transform()), impact(*new float(0)) {
}


// Function: GetForce
// Calculates the force vector to be applied to the object based on the target Transform and force type.
// Parameters:
// - target: The Transform object creating the force.
// - type: The type of force to be applied (e.g., attractive, repellent).
// Returns:
// - A Vector2 object representing the calculated force.
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
            force = Vortex(transform, target, impact );
            break;
        default:
            break;
    }
    return force;
}
// same function as before, just used to select the magnetic force
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