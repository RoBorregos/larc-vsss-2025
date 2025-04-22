#include "src/Entities/Robot.h"
#include "src/Entities/Ball.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <unordered_map>
using namespace std;

#include <chrono>
#include <thread>

float vortexConstant = 0.02f;
float repelentConstant = 0.02f;
float magneticConstant = 1.0f;
//Dentro del codigo, cada una de las posiciones de las entidades estan definidas por transform. Tienen este valor como referencia
//Por lo que puedes cambiar dentro de el vector de transform o en el mapa de entidades las posiciones de los obejtos.
//El id esta hecho para pelota = 0; alidados = 1,2,3; enemigos = -1,-2,-3;

int main()
{
    //Setting the environment
    unordered_map<int, Entity*> entities;
    unordered_map<int, Robot*> robots;
    // create transforms(position classes) for each of the entities;    
    vector<Transform> transforms(7,Transform());
    
    transforms[0] = Transform (4, 1, 6);// robot 1

    //transforms[1] = Transform (2, 4, 9);
    //transforms[2] = Transform (3, 6, 4);
    //transforms[3] = Transform (4, 2, 1);// robot -1
    //transforms[4] = Transform (5, 3, 2);
    //transforms[5] = Transform (6, 5, 3);
    //transforms[6] = Transform (3, 0, 0); //ball positions received from python -> we can automate it in the communication component 
    //transforms[7] = Transform (5, 0, 0);//goal -> Porteria
    //Create each of the robots and add it in the robot map for easy and direct access
    
    //(position,         ID,    ForceImpactInVectorField,port)
    
    robots[1] = new Robot(transforms[0],     1,     vortexConstant,         1001); //robot with the correct udpPOR
    robots[2] = new Robot(transforms[1],     2,     vortexConstant,         1001);
    robots[3] = new Robot(transforms[2],     3,     vortexConstant,         1002);
    robots[-1] = new Robot(transforms[3],    -1,    repelentConstant,       0);
    robots[-2] = new Robot(transforms[4],    -2,    repelentConstant,       0);
    robots[-3] = new Robot(transforms[5],    -3,    repelentConstant,       0);
    

    while (true) {        
        // Loop through all entities and receive position updates
        for (auto& entity : entities) {
            cout << "Receiving data for ID: " << entity.second->ID << endl;
            
            // Call ReceiveData to update the entity's transform
            entity.second->communication.ReceiveData();
            
            // Print the updated position
            cout << "Updated position - ID: " << entity.second->ID 
                 << " Position: (" << entity.second->transform.position.x << ", " 
                 << entity.second->transform.position.y << ") "
                 << "Rotation: " << entity.second->transform.rotation << endl;
        }
        // Add a small delay between updates
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    //Add the robots to the entities map To use later in the Force Generation
    for(auto robot: robots){
        entities[robot.first] = robot.second;
    }
    //Create the ball and add it to the entities map
    //      BallPos         GoalPoss    ID   ForceImpactVectorF Port
    Ball b (transforms[6], transforms[7], 0, magneticConstant , 1004);
    entities[0] = &b;
    //Print all entities transform (position and rotation)
    for (auto entiti : entities)
    {
        cout << "ID: " << entiti.second->ID << " Transform: " << entiti.second->transform << endl;
    }



//Find the robot with the least distance to the ball
    float minDIST = 15.0f, dist ;
    int minID = 0; //nearest robot to the ball
    //Print the distance of each of the robots towards the ball
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
    //Just as for now, we define as the attacker the robot nearest to the ball
    
//Determine the velocity vector the robot should follow by checking each of the entities on the map
    Vector2 tforce, result;
    for (auto entitie: entities)
    {
        // if the entitie is the same as the attacker we continue
        if (entitie.first == minID)
        {
            continue;
        }
        // if the entity is an enemie we add a repulsive form originated from the enemie.
        if (entitie.first > 0)
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, ForceType::VORTEX);
            cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
        }
        // if the entity is an ally, it adds a clockwise vortex force form the allie
        else if (entitie.first < 0)
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, ForceType::REPELENT);
            cout << "Robot ID: " << entitie.first << " Force: " << tforce << endl;
        }
        //and if the entity is the ball, it generates a magnetic force
        // setting the ball as attractive and a tempPos pointing to the goal as repulsive
        //This way the robot will always push the ball pointing to the goal
        else
        {
            tforce = entities[minID]->forceGenerator.GetForce(entitie.second->transform, b.goal, ForceType::MAGNETIC, 0.4f);
            cout << "Ball ID: " << entitie.first << " Force: " << tforce << endl;
        }
        result += tforce;
    }
    cout << "Resultant Force: " << result << endl;
//Generating the rpm and sending it to the robot
    //here the kinematic component will transform this velocitie vectore into rpm the robot should follow
    Output output = robots[minID]->kinematic.GetVelocities(result);
    output.Scale(200.0f);
    //Later the communication component will transmit this output information to the attacker robot
    robots[minID]->communication.SendData(output);
    cout << "Sending to Robot " << minID << " - Left: " << output.a << ", Right: " << output.b << endl;





}


//Nota: Disculpa si de vez en cuando no uso el mismo termino para vector de velocidad (fuerza, vector) más que 
//      nada es que yo me refiero a lo mismo pero uso diferentes palabras
//Entonces para resumir
/*
Primero generamos los transform (positions) de cada una de las entidades, ya sean los robots o las pelotas

------------------------------------A partir de aqui podriamos poner todo lo siguiente en un loop-------------------------------------------------------

[Communication] Recibimos la informacion de cada uno de las entidades y las actualizamos 
-> {No implementado aun}

[Transform] Encontramos el robot aliado más cercano a la pelota y lo seleccionamos como atacante
-> {Este metodo de toma de accion es arbitrario y se puede cambiar}

[ForceGenerator] Iteramos por cada entidad generando una fuerza resultante en nuestro atacante a la cual el debe seguir 
-> {Esta iteracion se puede hacer una funcion para no ocupar mucho espacio en el main(loop)}

[Kinematics] Generamos el rpm necesario para cumplir con el vector de velocidad generado por la iteracion anterior

[Communication] Mandas la informacion de rpm a el robot 
-> {Creo que ya esta implementada}

-------------------------------------------------------------Reperir------------------------------------------------------------------------------------------------
*/
/*
Entonces, como comentarios sobre la informacion que recivo de vision.
Lo que requiero es que se actualicen la informacion de posicion y rotacion de cada uno de las entidades, de 
manera que se pueda generar luego vectores y finalmente rpm para los robots 👍
*/