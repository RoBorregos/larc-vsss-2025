#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H
#include "Transform.h"


//Para decidir que tipo de fuerza aplicar
// estoy usando un enum para que sea mas facil intercambiar funciones para un solo objeto
enum class ForceType
{
    ATRACT,
    REPELENT,
    VORTEX,
    MAGNETIC,
};
//componente que genera vectores para el robot
class ForceGenerator
{
    public:

    //Objeto al que se aplica la fuerza 
    Transform& transform;
    float& impact;
    ForceGenerator(Transform& t, float& i) ;
    ForceGenerator();
                    // Objeto que genera la fuerza 
                                        // El tipo de fuerza
    Vector2 GetForce(Transform target, ForceType type);  
    Vector2 GetForce(Transform target, Transform goal, ForceType type, float dist);



    //DataTranfer Override operators
    ForceGenerator& operator=(const ForceGenerator& other);
    ForceGenerator(const ForceGenerator& other) ;
};



#endif