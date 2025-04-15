#include "Communication.h"

#include <iostream>



Communication::Communication(int& id, int port) : robotID(id), port(port) {
    // Constructor implementation
}

Communication::Communication() : robotID(*new int(0)) {
    // Default constructor implementation
}

void Communication::SendData(Output data) {
    // Send the data to the robot
    std::cout << "Sending data: " << data.a << ", " << data.b << std::endl;
}

void Communication::SendData(Transform data) {
    // Send the transform data to the robot
    std::cout << "Sending transform data: " << data.position.x << ", " << data.position.y << ", " << data.rotation << std::endl;

}