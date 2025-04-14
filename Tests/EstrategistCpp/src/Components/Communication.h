#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "Output.h"
#include "Transform.h"
class Communication
{
    public:
        int& robotID;

        Communication();
        Communication(int& id);
        // posibly send more than just the rpm, later there would be two functions
        // alberto will have to make the function to send the data
        void SendData(Output data);
        void SendData(Transform data);
};

#endif
