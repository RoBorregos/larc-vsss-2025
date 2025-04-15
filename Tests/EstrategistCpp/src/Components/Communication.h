#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "Output.h"
#include "Transform.h"
//clase hecha para poder mandar mesnajes directamente a el robot
class Communication
{
    public:
    //aqui tenia definido cual seria el id del robot con el que se mandaria los mensajes, pero como todo sera por puertos
    //le agrege la clase puerto para que sea solo sea usarlo en la funcion de send data
    
        int& robotID;
        int port;

        Communication();
        Communication(int& id, int port);
        //Mandar info
    //En este quiero que mandes la info de la velocidad de las llantas solamente, si quieres puedes crear un 
    //identificador para saber que funcion es 0,1
        void SendData(Output data);
    //en esta es mandar la info de la posicion, esta es para que el robot internmente sea indepenbdiente
        void SendData(Transform data);
};

#endif
