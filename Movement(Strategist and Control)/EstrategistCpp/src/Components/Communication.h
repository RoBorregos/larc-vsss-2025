#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "Output.h"
#include "Transform.h"
// Class created to send messages directly to the robot.
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <map>
#include <unordered_map>
using namespace std;


// Handles communication with a robot via network sockets.
// This class is responsible for sending data and reciving data 
// Usefull examples would be sending data to the robot or recive position data from python
class Communication {
public:
    int robotID; // Reference to the robot's ID. Used to identify the robot when reciving data
    Transform & transform; // references to the entitie position
    int portS;     // The port number used for communication with the robot
    int portR;      // The port number used for communication with the vision code
    
    const unordered_map<int, string> ips{
    {1 , "192.168.0.188"}, // Attacker Dflt
    {2 , "192.168.0.113"}, //Defender Dflt
    {3 , "192.168.0.199"}, // extra Dflt 
    {4 , "192.168.0.100"}, // replacement Dflt;
    };

    Communication(Transform& t, int id, int portA, int portB);

    // Function: SendData
    // Sends wheel velocity data to the robot.
    int SendData(Output data);


    int ReceiveData();



};

#endif