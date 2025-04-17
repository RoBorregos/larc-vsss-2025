#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "Output.h"
#include "Transform.h"
// Class created to send messages directly to the robot.
#include <iostream>
#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>

// Link with Winsock library
#pragma comment(lib, "ws2_32.lib")

// Handles communication with a robot via network sockets.
// This class is responsible for sending data and reciving data 
// Usefull examples would be sending data to the robot or recive position data from python
class Communication {
public:
    int& robotID; // Reference to the robot's ID. Used to identify the robot when reciving data
    Transform & transform; // references to the entitie position
    int port;     // The port number used for communication with the robot.

    Communication(Transformn& t, int& id, int port);

    // Function: SendData
    // Sends wheel velocity data to the robot.
    void SendData(Output data);

    // Function: SendData
    // Sends position data to the robot.
    void SendData(Transform data);

    //Later you can set the recieve data from python here
    //and as the Transform variable is referenced, we can edit it here, and all the reset of the Components will notice 
        // void Recieve data(float, float, float)
    //However, as I dont know how the communication works, I leave you this class all for you to set your necessary functions.
    //Remember That this component all the entities will have it, to is up to you how you want this component work
    // I just need for now that this recive the new positions and sends the new velocities


};

#endif