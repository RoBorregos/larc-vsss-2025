#include "Robot.h"


Robot::Robot(Transform& t, int i, float f, int portR, int portS) : Entity(t, i, f, portR, portS), kinematic(t){
    ID = i;
}


void Robot::GoTo(Transform objective){
    Output F;
    if((transform.position - objective.position).Magnitude() > 0.6){
        cout<<" -- Moving To Position"<<endl;
        F = kinematic.GetVelocities(objective);
    }else if(abs(transform.GetRotationalDifference(objective.rotation)) > 0.1) {
        cout<<" -- Rotating To Rotation"<<endl;
        F = kinematic.GetVelocitiesForRotation(objective);
    }else{
        cout<<" -- Staing in Position"<<endl;
        F = Output(0,0);
    }
    F.Scale(120);
    communication.SendData(F);
    cout<<" -- Sending Data: "<< F<<" To: "<<ID<<endl;
}