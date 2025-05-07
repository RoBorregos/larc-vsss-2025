#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include "src/Entities/Line.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <thread>
#include <atomic>
#include <unordered_map>
using namespace std;

#include <chrono>
#include <thread>

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
const Vector2 PorteriaEnemiga(-7.6,12); //x /= -10 
const Vector2 PorteriaAliada(-7.5,1.2); // y /= 10

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
    float maxDist = 4.5; //45 cm
    Vector2 pAliadaStart, pAliadaEnd;
    pAliadaStart = PorteriaAliada + Vector2(-maxDist,0); //Porteria Aliada
    pAliadaEnd = PorteriaAliada + Vector2(maxDist,0); //Porteria Aliada
        Line porteriaAliada (pAliadaStart, pAliadaEnd);
    cout<<"Limits Line: "<<endl;
    cout<< "Start: "<<pAliadaStart<<" End: "<<pAliadaEnd<<endl;
    

    //Create the ball and add it to the entities5 map
    //      BallPos         GoalPoss    ID   ForceImpactVectorF PortR       PortS
    Ball ball (transforms[0], transforms[1], 0, magneticConstant , 1200 ); 
                                                                            entities[0] = &ball;
        robots[3] = new Robot(transforms[2],  3,     vortexConstant, 1203,  1001); //robot with the correct udpPOR
                                                                                        entities[3] = robots[3];
        robots[2] = new Robot(transforms[3], 2,     vortexConstant, 1202,  1001 );
                                                                                        entities[2] = robots[2];
        robots[3] = new Robot(transforms[4], 1,     vortexConstant, 1201,  1001 );
                                                                                        entities[1] = robots[3];
        robots[-1] = new Robot(transforms[5], -1,  repelentConstant, 1204, 1001); //robot with the correct udpPOR
                                                                                        entities[-1] = robots[-1];
        robots[-2] = new Robot(transforms[6], -2,  repelentConstant, 1205, 1001);
                                                                                        entities[-2] = robots[-2];
        robots[-3] = new Robot(transforms[7], -3,  repelentConstant, 1206, 1001);
                                                                                        entities[-3] = robots[-3];
            robots[3]->transform.SetTransform(50,10,3.14);
            robots[2]->transform.SetTransform(80,75,3.14);
            ball.transform.SetTransform(75,70,0);
            ball.goal.SetTransform(PorteriaEnemiga,0);
    
    Transform DefenderObjective; 
    Output attackerOut;
    //Print all entities transform (position and rotation)
    
    vector<Transform> PenaltyPos{
        Transform(-7.5,5, 0), 
        Transform(-7.5,5, 0), 
        Transform(-7.5,5, 0)
    };
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
        }
    });
    int attackerID = 2; 
    int defenderID = 3; 
    int otherID = 1;
    
    while (Penalty || Playing) {     


        // Loop through all entities and receive position updates
       
        for (auto entity : entities) {
            //cout << "------------------Receiving data for ID: " << entity.second->ID << endl;
            
            // Call ReceiveData to update the entity's transform
            cout<<"Reciving from: "<< entity.second->communication.portR<<endl;
            int a = entity.second->communication.ReceiveData();
            if(a != 0){
                cout<<"                 Error: "<<a<<endl;
                continue;
            }
            cout << "ID: " << entity.second->ID << " Transform: " << entity.second->transform << endl;
            // Print the updated positionBall
        } 
        cout<<"Recived Data: "<<endl;
        if((robots[attackerID]->transform.position - ball.transform.position).Magnitude() > (robots[otherID]->transform.position - ball.transform.position).Magnitude()){
            int temp = attackerID;
            attackerID = otherID;
            otherID = temp;
        }
        cout<<"Attacker ID: "<<attackerID<<endl;
 
          
    if(Playing){
        for(auto r: entities){
            cout<<"             ";
            cout<<r.first<< " "<<r.second->transform<<endl;
        }
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
        
        cout<<"Result Force: "<< result<<endl;
        //Generating the rpm and sending it to the robot
        //here the kinematic component will transform this velocitie vectore into rpm the robot should follow
        if(!Attacking){
            attackerOut = robots[attackerID]->kinematic.GetVelocities(result);
        }else{
            attackerOut = robots[attackerID]->kinematic.GetVelocitiesForMagn(result);
        }

        // Defender Movement
        attackerOut.Scale(160.0f);
        cout<<"Attacker Move: "<<attackerOut<<endl;
        //Descomenta toda esta area para ver como se comporta el defensor
        Vector2 TrayectoryIntersection = porteriaAliada.Intersect(ball.transform);
        cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n  ---------- Defender: "<<endl;
        if(TrayectoryIntersection.x > -1000 ){
            DefenderObjective = Transform (TrayectoryIntersection, 0);
            cout<<"New Objective: "<< DefenderObjective<<endl; 
        }
        robots[defenderID]->GoTo(DefenderObjective);
        
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
        }
        
        SoportLine.SetLine(robots[attackerID]->transform.position, robots[NearestEnemyID]->transform.position);
        Vector2 SupportPos = SoportLine.MidPoint();
        if((SupportPos - robots[attackerID]->transform.position).Magnitude() > 5){
            robots[otherID]->GoTo(Transform(SupportPos, 0));
        }else{
            robots[otherID]->communication.SendData(Output(0,0));
        }

        //>>>>>>>>>>>>Sending data to the robots<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        vector<int> errors(3,0);
        errors[0] = robots[attackerID]->communication.SendData(attackerOut);
        //Al igual que esta linea para el defensor
        cout<<"//////////////////////////////Sending data to robots "<<endl;
        for(int i = 0; i < errors.size(); i++){
            if(errors[i] != 0){
                cout<<"Error sending : "<<i<<"Error N: "<<errors[i]<<endl;
            }
        }
        cout<<"//////////////////////////////Sended data to robots \n"<<endl;



        
    }else if (Penalty){
        cout<<"Moving Robots to Place..."<<endl;
        int i = 0;
        for(auto r: robots){
            r.second->GoTo(PenaltyPos[i]);
            i++;
        }
    }
    
    this_thread::sleep_for(chrono::milliseconds(15));

   

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