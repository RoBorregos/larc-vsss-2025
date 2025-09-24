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

vector<Vector3> Rectangle(Vector3 origin, float width, float hight){
    float x = width/2 ;
    float y = hight/2 ;
    vector<Vector3> ans(4);
    ans[0] = Vector3(-x, -y,0) + origin;
    ans[1] = Vector3(x, -y, 0) + origin;
    ans[2] = Vector3(x, y, 0) + origin;
    ans[3] = Vector3(-x, y, 0) + origin;
    return ans;
}