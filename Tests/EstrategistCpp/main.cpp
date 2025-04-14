#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include <vector>
#include <iomanip> 
#include <iostream>
using namespace std;

float vortexConstant = 0.5f;
float repelentConstant = 0.2f;
float magneticConstant = 1.0f;

int main(){
    vector<Entity*> entities;
    //create transforms for each of the entities, this can later be stored in a vector;
    Transform t1(4,1,6), t2(2,4,9), t3(3,6,4);
    Transform t4(4,2,1), t5(5,3,2), t6(6,5,3);
    Transform tb(2,0,0), tg(5,0,0);
    //
    Robot r1(t1, 1, vortexConstant), r2(t2, 2, vortexConstant), r3(t3, 3, vortexConstant);
    Robot r4(t4, -1, repelentConstant), r5(t5, -2, repelentConstant), r6(t6, -3, repelentConstant);
    Ball b(tb, tg, 0, magneticConstant);
    entities.push_back(&r1);
    entities.push_back(&r2);
    entities.push_back(&r3);
    entities.push_back(&r4);
    entities.push_back(&r5);
    entities.push_back(&r6);
    entities.push_back(&b);
    float minDIST = 15.0f;
    int minID = 0; float dist = 30.0f;
    for(auto entiti: entities){
        cout << "ID: " << entiti->ID << " position: " << entiti->transform.position << endl;
    }
    for(int i = 0; i < entities.size(); i++){
        if(entities[i]->ID > 0){
            dist = (entities[i]->transform.position - b.transform.position).Magnitude();
            cout << "Robot ID: " << entities[i]->ID << " distance: " << dist << endl;
            if(dist < minDIST){
                minDIST = dist;
                minID = i;
            }
        }else{
            break;
        }
    }
    cout << "The closest robot is: " << minID << endl;
    Vector2 force ;
    Vector2 tforce, result ;
    for(int i = 0; i < entities.size(); i++){
        Entity* entitie = entities[i];

        if(i == minID){continue;}
        if(entitie->ID > 0){
             tforce = entities[minID]->forceGenerator.GetForce(entitie->transform, ForceType::VORTEX);
            cout << "Robot ID: " << entitie->ID << " Force: " << tforce << endl;
        }else if (entitie->ID < 0){
            tforce = entities[minID]->forceGenerator.GetForce(entitie->transform, ForceType::REPELENT);
            cout << "Robot ID: " << entitie->ID << " Force: " << tforce << endl;
        }else{
            tforce = entities[minID]->forceGenerator.GetForce(entitie->transform, b.goal, ForceType::MAGNETIC, 0.4f);
            cout << "Ball ID: " << entitie->ID << " Force: " << tforce << endl;
        }
        result += tforce;
    }
    cout<< "Resultant Force: " << result << endl;

}