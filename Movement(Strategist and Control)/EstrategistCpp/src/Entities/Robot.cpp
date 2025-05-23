#include "Robot.h"


Robot::Robot(Transform& t, int i, float f, int portR, int portS) : Entity(t, i, f, portR, portS), kinematic(t){
    ID = i;
}

/*
 * This function moves the robot towards a specified objective position and orientation.
 * It takes the following parameter:
 *   - objective: a Transform object representing the target position and rotation for the robot.
 * The function checks the distance and orientation difference between the robot and the objective:
 *   - If the robot is far from the target position (> 0.6 units), it moves towards the position.
 *   - If the robot is close to the position but not correctly oriented (> 0.1 radians), it rotates to align.
 *   - If both position and orientation are correct, the robot stops.
 * The calculated velocity output is scaled and sent to the robot via communication.
 */
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