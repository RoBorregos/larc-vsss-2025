#include "vsss_simulation/MathU.hpp"



float wrapToPI(float a){
     if (a > M_PI){
        return a - 2 * M_PI;
     }
    if (a < -M_PI){
        return 2 * M_PI + a;
    }
    return a;
}

Vector3 Theta2Vector(float a){
    return Vector3(cos(a), sin(a), 0);
}


float gaussian(float r, float constant){
    return exp(-(r*r/(2*constant*constant)));
}