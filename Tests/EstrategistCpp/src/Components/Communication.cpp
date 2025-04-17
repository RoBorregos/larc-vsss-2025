#include "Communication.h"

#include <iostream>



Communication::Communication(Transform t, & id, int port) : robotID(id), port(port), transfrom(t) {
    // Constructor implementation
}


void Communication::SendData(Output data) {    
    {
        // Init Winsock
        WSADATA wsaData;
        int startupResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (startupResult != 0) {
            std::cerr << "WSAStartup failed: " << startupResult << std::endl;
            return;
        }
    
        SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udpSocket == INVALID_SOCKET) {
            std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
            WSACleanup();
            return;
        }
    
        // Define ESP32 address
        sockaddr_in esp32Addr;
        memset(&esp32Addr, 0, sizeof(esp32Addr));
        esp32Addr.sin_family = AF_INET;
        esp32Addr.sin_port = htons(this->port);  // Use the port passed to the object
    
        // Set the ESP32 IP (replace with the real IP)
        const char* esp32_ip = "10.22.234.26";  // TODO: Update if necessary
        if (inet_pton(AF_INET, esp32_ip, &esp32Addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address format for ESP32." << std::endl;
            closesocket(udpSocket);
            WSACleanup();
            return;
        }
    
        // Prepare the float array to send
        float dataToSend[2] = { data.a, data.b };
    
        int sendResult = sendto(
            udpSocket,
            reinterpret_cast<const char*>(dataToSend),
            sizeof(dataToSend),
            0,
            reinterpret_cast<sockaddr*>(&esp32Addr),
            sizeof(esp32Addr)
        );
    
        if (sendResult == SOCKET_ERROR) {
            std::cerr << "Failed to send data to ESP32: " << WSAGetLastError() << std::endl;
        } else {
            std::cout << "Sent " << sendResult << " bytes to ESP32: a=" << data.a << ", b=" << data.b << std::endl;
        }
        closesocket(udpSocket);
        WSACleanup();
    }     
}

void Communication::SendData(Transform data) {
    // Send the transform data to the robot
    std::cout << "Sending transform data: " << data.position.x << ", " << data.position.y << ", " << data.rotation << std::endl;

}