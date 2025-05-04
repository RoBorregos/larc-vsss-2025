#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include "src/Entities/Line.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <unordered_map>
using namespace std;

#include <chrono>
#include <thread>

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Valores importantes para el comportamiento de los robots
//Definen impacto sobre el atacante
float vortexConstant = 0.02f;  //Vortice para los aliados
float repelentConstant = 0.02f; //Repelente para los enemigos
float magneticConstant = 15.0f; //Magnetico para la pelota -> <Atrayente> y <Repelente> [Definido en la clase ForceGenerator]
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Dentro del codigo, cada una de las posiciones de las entidades estan definidas por transform. Tienen este valor como referencia
//Por lo que puedes cambiar dentro de el vector de transform o en el mapa de entidades las posiciones de los obejtos.
//El id esta hecho para pelota = 0; alidados = 1,2,3; enemigos = -1,-2,-3;
//##Valores definidos por la pelota
const Vector2 PorteriaEnemiga(-10.3,0.2); //x /= -10 
const Vector2 PorteriaAliada(-7.5,8); // y /= 10

int main()
{
    //Setting the environment
    unordered_map<int, Entity*> entities;
    unordered_map<int, Robot*> robots;
    // create transforms(position classes) for each of the entities;    
    vector<Transform> transforms(4,Transform());                         //Goal
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //Posicion de limites de la pelota conversion a negativo y *10
    //---------------------------------------------------- <50 cm
    //        ^                             ^
    //       -60                             -20
    float maxDist = 5; //20 cm
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    Vector2 pAliadaStart, pAliadaEnd;
    pAliadaStart = PorteriaAliada + Vector2(-maxDist,0); //Porteria Aliada
    pAliadaEnd = PorteriaAliada + Vector2(maxDist,0); //Porteria Aliada
        Line porteriaAliada (pAliadaStart, pAliadaEnd);

    //Create the ball and add it to the entities5 map
    //      BallPos         GoalPoss    ID   ForceImpactVectorF PortR       PortS
    Ball ball (transforms[1], transforms[2], 0, magneticConstant , 1200 ); 
                                                                            entities[0] = &ball;
        robots[3] = new Robot(transforms[0],  3,     vortexConstant, 1201       ,1001); //robot with the correct udpPOR
                                                                                        entities[3] = robots[3];
        robots[2] = new Robot(transforms[3], 2,     vortexConstant, 1202,       1001 );
                                                                                   //    entities[2] = robots[2];
            robots[3]->transform.SetTransform(0,0,0);
            robots[2]->transform.SetTransform(0,0,0);
            ball.transform.SetTransform(0,0,0);
            ball.goal.SetTransform(PorteriaEnemiga,0);
            
    //Print all entities transform (position and rotation)
    
    
    
    while (true) {     
        //atacker;
        int attackerID = 3; 
        int defenderID = 2;   
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
            // Print the updated position
        }
       
       //robots[attackerID]->transform.SetTransform(robots[attackerID]->transform.position.x, robots[attackerID]->transform.position.y,j);

    
        //Determine the velocity vector the robot should follow by checking each of the entities on the map
        Vector2 tforce, result;
        cout<<"Adding Forces:                          "<<endl;
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
                
                //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                //Por ahora para debugear es hacer que el robot pueda ir a un punto bien, desde cualquier posicion
                //tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform,  ForceType::ATRACT);
                //Despues es modificar para que responda a que evite el punto
                                                                                                            //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                                                                                               
                tforce = entitie.second->forceGenerator.GetForce(entities[attackerID]->transform,ball.goal, ForceType::MAGNETIC, 0.12);
                                                                                                            //Distancia entre pelota ^ y el punto repelente
                                                                                                             
                                                                                                             //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
                
                //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            }
            result += tforce;
        }
        
        cout<<"Result Force: "<< result<<endl;
        //Generating the rpm and sending it to the robot
        //here the kinematic component will transform this velocitie vectore into rpm the robot should follow
        Output attackerOut = robots[attackerID]->kinematic.GetVelocities(result);
        attackerOut.Scale(160.0f);
        cout<<"Attacker Move: "<<attackerOut<<endl;
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        //Descomenta toda esta area para ver como se comporta el defensor

        Output defnederOut;
        Vector2 TrayectoryIntersection = porteriaAliada.Intersect(ball.transform);
        if(TrayectoryIntersection.x > -1000){
            Transform transformTrayectoryIntersection (TrayectoryIntersection, 0);
            defnederOut = robots[defenderID]->kinematic.GetVelocities(transformTrayectoryIntersection);
            defnederOut.Scale(80.0f);
        }else{
            defnederOut = Output(120,-120);
        }
        cout<<"Defender Move: "<< defnederOut<<endl;

        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


        vector<int> errors(3,0);
        errors[0] = robots[attackerID]->communication.SendData(attackerOut);
        //Al igual que esta linea para el defensor
        errors[1] = robots[defenderID]->communication.SendData(defnederOut);
        cout<<"//////////////////////////////Sending data to robots "<<endl;
        for(int i = 0; i < errors.size(); i++){
            if(errors[i] != 0){
                cout<<"Error sending : "<<i<<"Error N: "<<errors[i]<<endl;
            }
        }
        cout<<"//////////////////////////////Sended data to robots \n"<<endl;
        if(errors[0] != 0){
            cout<<"Error sending data to attacker"<<endl;
            break;
        }
        
        
        // Add a small delay between updates

    }

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