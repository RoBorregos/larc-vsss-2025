#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H
#include "Transform.h"


// Represents the type of force to be applied to another robot.
// This enum is used to easily switch between different force generation methods.
enum class ForceType
{
    ATRACT,
    REPELENT,
    VORTEX,
    MAGNETIC,
};
enum class ForceMode{
    CONSTANT,
    PROPORTIONAL,
};

// A component that generates force vectors for a robot based on its transform and other parameters.
// This class is responsible for calculating forces such as attraction, repulsion, vortex, and magnetic forces.
class ForceGenerator
{
    public:

    //Transform origin of the vectores
    Transform& transform; // Reference to the Transform object which is the origin of the force vector
    float& impact;          // Reference to the impact factor, which scales the generated force.
    ForceGenerator(Transform& t, float& i) ;
    ForceGenerator();
                    // Objeto que genera la fuerza 
                                        // El tipo de fuerza
    Vector2 GetForce(Transform target, ForceType type, ForceMode mode = ForceMode::PROPORTIONAL); // Attraction force   
    Vector2 GetForce(Transform target, Transform goal, ForceType type, float dist, ForceMode mode = ForceMode::PROPORTIONAL); // Magnetic force

    Transform CreatePositionInRect( Transform goal, float dist);

    //DataTranfer Override operators
    ForceGenerator& operator=(const ForceGenerator& other);
    ForceGenerator(const ForceGenerator& other) ;
};



#endif