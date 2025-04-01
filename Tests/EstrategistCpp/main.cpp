#include "Ball.h"
#include <vector>
#include <iomanip> 


int main(){
    vector<robot*> robots (2,new robot()) ;
    robot * attacker = new allie(0,0,3,     0.5f,0);
    robot * support  = new allie(-3,-7,3,     3,1);
    robot goal(12,0,0,0);
    //robot * ball = new Ball(1,1,0,           3.6f,goal, 2.3f);
    robots[1] = (attacker);
    robots[0] = (support);
    //robots[0] = ball;

    force finalF;

    for(auto robot : robots){
        if(robot == attacker){
            continue;
        }
        finalF += robot->GetForce(*attacker);
    }
    cout<<setprecision(15)<<fixed;
    cout<<finalF.x<< " "<< finalF.y<<endl;

}
