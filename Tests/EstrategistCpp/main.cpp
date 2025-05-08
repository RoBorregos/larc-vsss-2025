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
void ReceiveDataAsync(Entity* entity, atomic<bool>& running, atomic<bool>& penalty) {
    while (running || penalty) {
        int result = entity->communication.ReceiveData();
        if (result == 0) {
            lock_guard<mutex> lock(entityMutex);
            // Data received successfully, transform is updated inside ReceiveData
            cout << "Entity ID: " << entity->ID << " updated transform: " << entity->transform << endl;
        } else {
            cerr << "Error receiving data for Entity ID: " << entity->ID << ", Error Code: " << result << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(10)); // Avoid busy-waiting
    }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Valores importantes para el comportamiento de los robots
//Definen impacto sobre el atacante
float vortexConstant = 0.08f;  //Vortice para los aliados
float repelentConstant = 0.02f; //Repelente para los enemigos
float magneticConstant = 0.1f; //Magnetico para la pelota -> <Atrayente> y <Repelente> [Definido en la clase ForceGenerator]
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Dentro del codigo, cada una de las posiciones de las entidades estan definidas por transform. Tienen este valor como referencia
//Por lo que puedes cambiar dentro de el vector de transform o en el mapa de entidades las posiciones de los obejtos.
//El id esta hecho para pelota = 0; alidados = 1,2,3; enemigos = -1,-2,-3;
//##Valores definidos por la pelota
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
    
                //---------------------------------------Gaols Position-----------------------
                float maxDist = 4.5; //45 cm
                Vector2 pAliadaStart, pAliadaEnd;
                pAliadaStart = PorteriaAliada + Vector2(0,-maxDist); //Porteria Aliada
                pAliadaEnd = PorteriaAliada + Vector2(0,maxDist); //Porteria Aliada
                    Line porteriaAliada (pAliadaStart, pAliadaEnd);
                cout<<"Limits Line: "<<endl;
                cout<< "Start: "<<pAliadaStart<<" End: "<<pAliadaEnd<<endl;

    
    

    //Create the ball and add it to the entities5 map
    //      BallPos         GoalPoss    ID   ForceImpactVectorF PortR       PortS
    Ball ball (transforms[0], transforms[1], 0, magneticConstant , 1200 ); 
                                                                            entities[0] = &ball;
    robots[2] = new Robot(transforms[4], 2,     vortexConstant, 1202,  1001 );
    robots[1] = new Robot(transforms[3], 1,     vortexConstant, 1201,  1001 );
    robots[3] = new Robot(transforms[5], 3,    repelentConstant, 1203,  1001 );
    robots[-1] = new Robot(transforms[6], -1,    repelentConstant, 1204,  1001 );
    robots[-2] = new Robot(transforms[7], -2,    repelentConstant, 1205,  1001 );
    robots[-3] = new Robot(transforms[8], -3,    repelentConstant, 1206,  1001 );
    ball.goal.SetTransform(PorteriaEnemiga,0);
                                                for(auto r: robots){
                                                    entities[r.first] = r.second;
                                                }

                                                vector<thread> threads;
                                                for (auto& entityPair : entities) {
                                                    threads.emplace_back(ReceiveDataAsync, entityPair.second, ref(Playing), ref(Penalty));
                                                }
    
    Transform DefenderObjective; 
    Output attackerOut;
    //Print all entities transform (position and rotation)
    
    vector<Transform> PenaltyPos{
        Transform(7.5,5, 0), 
        Transform(7.5,5, 0), 
        Transform(7.5,5, 0)
    };


    Line LineBetweenGoals (PorteriaEnemiga, PorteriaAliada);
    Vector2 Center = LineBetweenGoals.MidPoint();


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
    


    while (Penalty || Playing) {     

        // Loop through all entities and receive position updates
    if(Playing){
        //Determine the velocity vector the robot should follow by checking each of the entities on the map
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
                cout<<"                              ";

                Transform temporal = entitie.second->forceGenerator.CreatePositionInRect(ball.goal, -1.2);

                float distance = (temporal.position - entities[attackerID]->transform.position).Magnitude();
                cout<<"Distance: "<<distance<<endl;
                
                if(distance > 2){
                    cout<<"Moving To Pos: "<< temporal<<endl;
                    tforce = entities[attackerID]->forceGenerator.GetForce(temporal, ForceType::REPELENT, ForceMode::CONSTANT);
                    Attacking = false;
                }else{
                    cout<<"Attacking to: "<< entitie.second->transform<<endl;
                    tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform, ball.goal , ForceType::MAGNETIC, 0.4,  ForceMode::CONSTANT);
                    tforce *= 3;
                    Attacking = true;
                }

                cout<<"                ";
                cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
                
            }
            result += tforce;
        }
        
        //Walls repulsiveForce
        result += (Center - robots[attackerID]->transform.position) * 0.5f;

        cout<<"Result Force: "<< result<<endl;
        //Generating the rpm and sending it to the robot
        //here the kinematic component will transform this velocitie vectore into rpm the robot should follow
        if(!Attacking){
            attackerOut = robots[attackerID]->kinematic.GetVelocities(result);
        }else{
            attackerOut = robots[attackerID]->kinematic.GetVelocitiesForMagn(result);
        }
        attackerOut.Scale(160.0f);
        robots[attackerID]->communication.SendData(attackerOut);
        cout<<"Attacker Move: "<<attackerOut<<endl;
        //Al igual que esta linea para el defensor


        // Defender Movement
        //Descomenta toda esta area para ver como se comporta el defensor
        Vector2 TrayectoryIntersection = porteriaAliada.Intersect(ball.transform);
        cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n  ---------- Defender: "<<endl;
        if(TrayectoryIntersection.x > -1000 ){
            DefenderObjective = Transform (TrayectoryIntersection, 0);
            cout<<"New Objective: "<< DefenderObjective<<endl; 
        }
        robots[defenderID]->GoTo(DefenderObjective);

        /*
        //OtherMovement 
        Line SoportLine;
        int NearestEnemyID = -1;
        for(auto r: robots){
            if(r.first < 0 ){
               if( (r.second->transform.position  - robots[attackerID]->transform.position).Magnitude() 
                        <  (robots[attackerID]->transform.position - robots[NearestEnemyID]->transform.position).Magnitude()){
                    NearestEnemyID = r.first;
                }
            }
        }*/
        
   

        
    }else if (Penalty){
        cout<<"Moving Robots to Place..."<<endl;
        int i = 0;
        for(auto r: robots){
            r.second->GoTo(PenaltyPos[i]);
            i++;
        }
    }
    
    this_thread::sleep_for(chrono::milliseconds(25));

   

    }


    for(int i = 0; i < 3; i++){
        for(auto r: robots){
            if(r.first > 0){
                r.second->communication.SendData(Output(0,0));
            }
        }
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
        
    
    commandThread.join(); // Wait for the command thread to finish
    cout << "Program terminated." << endl;
    return 0;
}





/*
En resumen: Ahora lo que tienes que hacer es modificar los valores de las constantes para que el movimiento sea más fluido
-----Primero esta que el robot vaya hacia la pelota

------------------------------------Orden de acciones a cometer-------------------------------------------------------
<variable, archivo>
(par1, par2)

- Primero definir variables para que funcione el campo vectorial base : Que el robot vaya a la pelota 
++ Variable que define -> <magneticConstant, main.cpp> <ANGULAR_CONSTANT, Kinematic.h> <LINEAR_CONSTANT, Kinematic.h> 

- Ahora que ya sabemos que funciona toca configurar las constantes de movimiento para que funcione de manera magnetica
**Acciones a realizar <- Modificar el tipo de accion (cambiar de atrayente a magnetica) <main.cpp>
++ Variable que define -> < proporcion entre atrayente y repelente, ForceGenerator.cpp>   <distancia de la pelota al punto excluyente, main.cpp>
?? Dato: La funcion atrayente es constante en cualquier punto del mapa mientras que la funcion repelente es proporcional a la distancia del Robot

- Finalmente poner a probar el portero solo es cambiar las variables de movimiento bidireccional y definir los limites de las porterias
**Acciones a realizar <- Quitar de comentario a la accion de portero <main.cpp>
++ Variable que define -> <Constante angular de robot bidireccional, Kinematic.cpp>  <(start, end), main.cpp>
??Dato: puede que la constante del robot bidireccional puede que sea mejor ponerla a 1 despues de probar lo anterior 

-------------------------------------------------------------Reperir------------------------------------------------------------------------------------------------
*/