#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "Output.h"
#include "Transform.h"
// Class created to send messages directly to the robot.
#include <iostream>
#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <map>
#include <unordered_map>
using namespace std;

// Link with Winsock library
#pragma comment(lib, "ws2_32.lib")

// Handles communication with a robot via network sockets.
// This class is responsible for sending data and reciving data 
// Usefull examples would be sending data to the robot or recive position data from python
class Communication {
public:
    int robotID; // Reference to the robot's ID. Used to identify the robot when reciving data
    Transform & transform; // references to the entitie position
    int portS;     // The port number used for communication with the robot.9
    int portR;
    
    unordered_map<int, string> ips;

    Communication(Transform& t, int id, int portA, int portB);

    // Function: SendData
    // Sends wheel velocity data to the robot.
    int SendData(Output data);


    int ReceiveData();

    //Later you can set the recieve data from python here
    //and as the Transform variable is referenced, we can edit it here, and all the reset of the Components will notice 
        // void Recieve data(float, float, float)
    //However, as I dont know how the communication works, I leave you this class all for you to set your necessary functions.
    //Remember That this component all the entities will have it, to is up to you how you want this component work
    // I just need for now that this recive the new positions and sends the new velocities


};

#endif