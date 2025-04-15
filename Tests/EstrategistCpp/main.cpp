#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include <vector>
#include <iomanip>
#include <iostream>
using namespace std;

float vortexConstant = 0.02f;
float repelentConstant = 0.02f;
float magneticConstant = 1.0f;
//Dentro del codigo, cada una de las posiciones de las entidades estan definidas por transform. Tienen este valor como referencia
//Por lo que puedes cambiar dentro de el vector de transform o en el mapa de entidades las posiciones de los obejtos.
//El id esta hecho para pelota = 0; alidados = 1,2,3; enemigos = -1,-2,-3;


int main()
{
    unordered_map<int, Entity*> entities;
    unordered_map<int, Robot*> robots;
    // create transforms for each of the entities;
    vector<Transform> transforms(7,Transform());
    transforms[0] = Transform (4, 1, 6);// robot 1
    transforms[1] = Transform (2, 4, 9);
    transforms[2] = Transform (3, 6, 4);
    transforms[3] = Transform (4, 2, 1);// robot -1
    transforms[4] = Transform (5, 3, 2);
    transforms[5] = Transform (6, 5, 3);
    transforms[6] = Transform (2, 0, 0);//ball
    transforms[7] = Transform (5, 0, 0);//goal Porteria
    //Create each of the robots and add it in the robot map
    robots[1] = new Robot(transforms[0], 1, vortexConstant, 1000);
    robots[2] = new Robot(transforms[1], 2, vortexConstant, 1001);
    robots[3] = new Robot(transforms[2], 3, vortexConstant, 1002);
    robots[-1] = new Robot(transforms[3], -1, repelentConstant,0);
    robots[-2] = new Robot(transforms[4], -2, repelentConstant,0);
    robots[-3] = new Robot(transforms[5], -3, repelentConstant,0);
    //Add the robots to the entities map
    for(auto robot: robots){
        entities[robot.first] = robot.second;
    }
    //Create the ball and add it to the entities map
    Ball b (transforms[6], transforms[7], 0, magneticConstant);
    entities[0] = &b;
//ALL Entities Transform
    for (auto entiti : entities)
    {
        cout << "ID: " << entiti.second->ID << " Transform: " << entiti.second->transform << endl;
    }
//MINIDISTANCE Robot find
    float minDIST = 15.0f, dist ;
    int minID = 0;
    
    for (auto entiti : robots)
    {
        if (entiti.first > 0)
        {
            dist = (entiti.second->transform.position - entities[0]->transform.position).Magnitude();
            cout << "Robot ID: " << entiti.first<< " distance: " << dist << endl;
            if (dist < minDIST)
            {
                minDIST = dist;
                minID = entiti.first;
            }
        }
    }
    cout << "The closest robot is: " << minID << endl;
//VECTOR Determination
    Vector2 tforce, result;
    for (auto entitie: entities)
    {
        if (entitie.first == minID)
        {
            continue;
        }
        if (entitie.first > 0)
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, ForceType::VORTEX);
            cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
        }
        else if (entitie.first < 0)
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, ForceType::REPELENT);
            cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
        }
        else
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, b.goal, ForceType::MAGNETIC, 0.4f);
            cout << "Ball ID: " << entitie.first << " Force: " << tforce << endl;
        }
        result += tforce;
    }
    cout << "Resultant Force: " << result << endl;
    //Kinematics output
    Output output = robots[minID]->kinematic.GetVelocities(result);
    output.Scale(200.0f);
    //Communication
    robots[minID]->communication.SendData(output);
}