#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include "src/Entities/Line.h"
#include <vector>
#include <thread>
#include <mutex>
#include <iomanip>
#include <iostream>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <chrono>
#include <thread>

using namespace std;

mutex entityMutex;

//This function is to define the structure of the thread that will be assigned for each of the entities that recieve data from vision
//It recieves the entitie, and the bools that determine the state of the game
//Thus the information will be 
void ReceiveDataAsync(Entity* entity, atomic<bool>& running, atomic<bool>& penalty) {
    while (running || penalty) {
        int result = entity->communication.ReceiveData(); //Function set to wait
        if (result == 0) {
            lock_guard<mutex> lock(entityMutex);
            cout << "Entity ID: " << entity->ID << " updated transform: " << entity->transform << endl;
        } else {
            cerr << "Error receiving data for Entity ID: " << entity->ID << ", Error Code: " << result << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(10)); // Avoid busy-waiting
    }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Values for the vector field
float vortexConstant = 0.08f;   //Vortex for allies 
float repelentConstant = 0.02f; //Repelent for enemies
float magneticConstant = 0.1f;  //Magnetic ball ->  <Atractive> and <Repelent>  [Defined in the class ForceGenerator]
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//##Values Calibrated with the ball
const Vector2 PorteriaEnemiga(12.53,-7.67); //x /= -10 
const Vector2 PorteriaAliada(0.87,-6.63); // y /= 10

int main()
{
    //Thread division
    atomic<bool> Playing(true);
    atomic<bool> Penalty(false);
    string cmd;
    //Setting the environment
    unordered_map<int, Entity*> entities;
    unordered_map<int, Robot*> robots;
    // create transforms(position classes) for each of the entities;    
    vector<Transform> transforms(9 ,Transform());    
    
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Goal Position
    float maxDist = 4.5; //45 cm
    Vector2 pAliadaStart, pAliadaEnd;
    pAliadaStart = PorteriaAliada + Vector2(0,-maxDist); //Porteria Aliada
    pAliadaEnd = PorteriaAliada + Vector2(0,maxDist); //Porteria Aliada
        Line porteriaAliada (pAliadaStart, pAliadaEnd);
    cout<<"Limits Line: "<<endl;
    cout<< "Start: "<<pAliadaStart<<" End: "<<pAliadaEnd<<endl;
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    
    

    //Create the ball and add it to the entities5 map
    //      BallPos         GoalPoss    ID   ForceImpactVectorF PortR       PortS
    Ball ball (transforms[0], transforms[1], 0, magneticConstant , 1200 ); 
                                                                            entities[0] = &ball;

    //Creation of each robot in the game

    robots[1] = new Robot(transforms[4], 1,     vortexConstant, 1201,  1001 );
    robots[2] = new Robot(transforms[3], 2,     vortexConstant, 1202,  1001 );
    robots[3] = new Robot(transforms[5], 3,    repelentConstant, 1203,  1001 );
    robots[-1] = new Robot(transforms[6], -1,    repelentConstant, 1204,  1001 );
    robots[-2] = new Robot(transforms[7], -2,    repelentConstant, 1205,  1001 );
    robots[-3] = new Robot(transforms[8], -3,    repelentConstant, 1206,  1001 );
    ball.goal.SetTransform(PorteriaEnemiga,0);
            
    //      Set Robots in Entities           //
            for(auto r: robots){
                entities[r.first] = r.second;
            }
    //     Set Communication tasks in threads     //
            vector<thread> threads;
            for (auto& entityPair : entities) {
                threads.emplace_back(ReceiveDataAsync, entityPair.second, ref(Playing), ref(Penalty));
            }
    
    Transform DefenderObjective; 
    Transform OtherObjective;
    Output attackerOut;
    
    //Positions for Reset
    vector<Transform> PenaltyPos{
        Transform(7.5,5, 0), 
        Transform(7.5,5, 0), 
        Transform(7.5,5, 0)
    };

//Central Point when robots get stuck
    Line LineBetweenGoals (PorteriaEnemiga, PorteriaAliada);
    Vector2 Center = LineBetweenGoals.MidPoint();

//Threads to recieve inputs in the terminal
    thread commandThread([&]() {
        while (Playing || Penalty) {
            cout << "Enter command (stop/change): ";
            cin >> cmd;
            if (cmd == "s") {
                Playing = false;
                Penalty = false;
            } else if (cmd == "c") {
                Penalty = !Penalty;
                Playing = !Playing;
            } else {
                cout << "Unknown cmd: " << cmd << endl;
            } 
            this_thread::sleep_for(chrono::milliseconds(30));
        }
    });

    int attackerID = 1; 
    int defenderID = 2; 
    int otherID = 3;


    while (Penalty || Playing) {     

        //Main Loop
    if(Playing){
        //If the game is going



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Attacker Play
        Vector2 tforce, result;
        cout<<"Adding Forces:                          "<<endl;
        bool Attacking = false;
        for (auto entitie: entities)
        {
            // if the entitie is the same as the attacker we continue
            if (entitie.first == attackerID)
            {
                continue;
            }
            cout<<"                ";
            // if the entity is an enemy we add a repulsive form originated from the enemy.
            if (entitie.first > 0)
            {
                tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform, ForceType::VORTEX);
                cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
            }
            // if the entity is an ally, it adds a clockwise vortex force form the allie
            else if (entitie.first < 0)
            {
                tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform, ForceType::REPELENT);
                cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
            }
            //and if the entity is the ball, it generates a magnetic force
            // setting the ball as attractive and a tempPos pointing to the goal as repulsive
            //This way the robot will always push the ball pointing to the goal
            else
            {
                ///////// -- First Go to the Optimal Pos and later use Magnetic for easy response 
                Transform temporal = entitie.second->forceGenerator.CreatePositionInRect(ball.goal, -1.2);


                //distance from player to the temporal position
                float distance = (temporal.position - entities[attackerID]->transform.position).Magnitude();
                
                
                if(distance > 2){
                    tforce = entities[attackerID]->forceGenerator.GetForce(temporal, ForceType::REPELENT, ForceMode::CONSTANT);
                    Attacking = false;
                }else{
                    tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform, ball.goal , ForceType::MAGNETIC, 0.4,  ForceMode::CONSTANT);
                    tforce *= 3;
                    Attacking = true;
                }

                cout<<"                ";
                cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
                
            }
            result += tforce;
        }
        
        //Walls repulsiveForce (Fuerza del robot hacia el centro )
        // Esta fuerza se puede cambiar dependiendo del estado de los demas objetos
        result += (Center - robots[attackerID]->transform.position) * 0.3f;


        attackerOut = robots[attackerID]->kinematic.GetVelocities(result);
        //Generating the rpm and sending it to the robot
        //here the kinematic component will transform this velocitie vectore into rpm the robot should follow
        attackerOut.Scale(160.0f);
        robots[attackerID]->communication.SendData(attackerOut);
        cout<<"Attacker Move: "<<attackerOut<<endl;
        //Al igual que esta linea para el defensor


////////////////////////////////////////////////////////////////////////////////////////////////// - Defender Play
        //Primero determina si con la velocidad actual de la pelota podria haber alguna colision
        Vector2 TrayectoryIntersection = porteriaAliada.Intersect(ball.transform);
        cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n  ---------- Defender: "<<endl;
        //Si es que existe el caso, Determina ese punto de colision como objetivo
        if(TrayectoryIntersection.x > -1000 ){
            DefenderObjective = Transform (TrayectoryIntersection, 0);
            cout<<"New Objective: "<< DefenderObjective<<endl; 
        }
        //Mover el robot a la posicion
        robots[defenderID]->GoTo(DefenderObjective);


///////////////////////////////////////////////////////////////////////////////////////////////// - Support Play
        //Search of the nearest enemy to the attacker
        Line SoportLine;
        int NearestEnemyID = -1;
        double dist;
        for(auto r: robots){
            if(r.first < 0 ){
               if( (r.second->transform.position  - robots[attackerID]->transform.position).Magnitude() 
                        <  (robots[attackerID]->transform.position - robots[NearestEnemyID]->transform.position).Magnitude()){
                    NearestEnemyID = r.first;
                    dist = (robots[attackerID]->transform.position - robots[NearestEnemyID]->transform.position).Magnitude();
                }
            }
        }
        //create a line between the nereast attacker to the attacker and obstruct their path
        SoportLine.SetLine(robots[attackerID]->transform.position, robots[NearestEnemyID]->transform.position);
        Vector2 MidPointAt_EN = SoportLine.MidPoint();
        if(dist > 1.5){
            OtherObjective = Transform(MidPointAt_EN, 0);
        }
        robots[otherID]->GoTo(OtherObjective);
        
   

        
    }else if (Penalty){
        //Move every robot to an especific location
        cout<<"Moving Robots to Place..."<<endl;
        int i = 0;
        for(auto r: robots){
            r.second->GoTo(PenaltyPos[i]);
            i++;
        }
    }
    
    this_thread::sleep_for(chrono::milliseconds(25));

   

    }

    //Turn Off all the robots
    for(int i = 0; i < 3; i++){
        for(auto r: robots){
            if(r.first > 0){
                r.second->communication.SendData(Output(0,0));
            }
        }
    }
    // Elimin all the threads for comm
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
        
    // Elim the last thread
    commandThread.join(); 
    cout << "Program terminated." << endl;
    return 0;
}





/*


------------------------------------Sequence of actions: -------------------------------------------------------
- Inclusion of headers and definition of constants for robot behavior (vortex, repulsive, magnetic).
- Definition of key positions on the field (goals, center line).
- Initialization of game control variables (Playing, Penalty) and structures for entities and robots.
- Creation and initialization of the ball and robots, assigning their positions and parameters.
- Insertion of robots into the entity map.
- Launching threads to receive data from each entity asynchronously.
- Definition of positions for penalty situations.
- Calculation of the center point of the field.
- Launching a thread to receive user commands (stop/change).
- Main loop:
    - If in game mode:
        - Calculation of forces on the attacker (repulsive, vortex, magnetic) and sending movement commands.
        - Calculation of the defender's target position based on the ball's trajectory and movement toward it.
        - Determines the closest enemy to the attacker and positions the support robot.
    - If in penalty mode:
        - Moves the robots to predefined positions.
        - Brief pause to avoid excessive CPU consumption.
    - At the end of the loop:
        - Shuts down the robots (sends stop commands).
        - Waits for all communication and command threads to terminate.
        - Program termination message.
-------------------------------------------------------------------------------------------------------------------------------------------------------------
*/