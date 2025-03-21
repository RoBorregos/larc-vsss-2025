#ifndef ROBOT_
#define ROBOT_
//header file content


#include <iostream>
using namespace std;


class force{
    public :
    force(float, float);
    force();
    float x;
    float y;
    force& operator += (const force&);
};

class position{
public:
    float x;
    float y;
    float z;
    float impact;
    position();
    position(float,float, float, float);
    virtual force GetForce(position);
    bool operator==( const position &);
    
};
class allie : public position{
    public :
    int rol;
    force GetForce (position) override;
    allie(float, float, float, float, int);
    allie();

};

#endif

