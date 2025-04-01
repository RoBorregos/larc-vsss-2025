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
    force operator * (const float&);
    force operator +(const force&);
};

class robot{
public:
    float x;
    float y;
    float z;
    float impact;
    robot();
    robot(float,float, float, float);
    virtual force GetForce(robot);
    bool operator==( const robot &);
    
};
class allie : public robot{
    public :
    int rol;
    force GetForce (robot) override;
    allie(float, float, float, float, int);
    allie();

};

#endif

