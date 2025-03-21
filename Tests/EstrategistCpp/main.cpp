#include "Robots.h"
#include <vector>


int main(){
    vector<position*> robots ;
    position * attacker = new allie(1,2,3,0.5,0);
    position * support  = new allie(3,3,3,0.2,1);
    robots.push_back(attacker);
    robots.push_back(support);
    force finalF;

    for(auto robot : robots){
        if(robot == attacker){
            continue;
        }
        finalF += robot->GetForce(*attacker);
    }
    cout<<finalF.x<< " "<< finalF.y<<endl;

}
