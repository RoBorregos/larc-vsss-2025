#include "Communication.h"
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

Communication::Communication(Transform& t, int id, int port) : robotID(id), port(port), transform(t) {
    ips[1] = "192.168.0.216"; // change for each xiaoC6
    ips[2] = "192.168.137.212";
    ips[3] = "192.168.137.213";
}


void Communication::SendData(Output data) {   
    {
        // Init Winsock
        std::string ip = ips[robotID]; // Get the IP address for the robot ID
        cout<< "Sending to Ip = "<<ip<<" For ID = "<<robotID<<endl;
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
        esp32Addr.sin_family = AF_INET; // IPV04
        esp32Addr.sin_port = htons(this->port);  // Use the port passed to the object
    
        // Set the ESP32 IP (replace with the real IP)
        
        
        const char* esp32_ip = ip.c_str() ; // pointer to const char 
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
        /*
        } else {
            std::cout << "Sent " << sendResult << " bytes to ESP32: a=" << data.a << ", b=" << data.b << std::endl;
        }
        */
        closesocket(udpSocket);
        WSACleanup();
    }     
}
}

void Communication::ReceiveData() {
    #define BUFFER_SIZE 255 // Define the buffer size for receiving data


    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock with version 2.2 
    if (result != 0) { // Check if WSAStartup was successful
        std::cerr << "WSAStartup failed: " << result << std::endl;
        return;
    }

    SOCKET receive_py = socket(AF_INET, SOCK_DGRAM,0); // Create a UDP socket, using IPv4 (AF_INET) and datagram (SOCK_DGRAM) protocol for UDP
    if (receive_py == INVALID_SOCKET) { // Check if socket creation was successful
        std::cerr << "Fail receive socket: " << WSAGetLastError() << std::endl;
        WSACleanup(); // Clean up Winsock
        return;
    }


    /* 
    struct addrinfo hints = {0}, *addr_result = nullptr;
    hints.ai_family = AF_INET; // IPv4
    hints.ai_socktype = SOCK_DGRAM; // UDP
    hints.ai_flags = AI_PASSIVE; // For binding to any address
    */

    struct sockaddr_in position_addr; // Define a structure to hold the server address information for the vision ball
    memset(&position_addr, 0, sizeof(position_addr)); // Initialize the server address structure to zero   
    position_addr.sin_family = AF_INET; // Set the address family to IPv4
    position_addr.sin_port = htons(this->port); // Set the port number for the server (PYPORT) and convert it to network byte order


    char hostname[NI_MAXHOST]; // Buffer to store the hostname
    if (gethostname(hostname, sizeof(hostname)) == SOCKET_ERROR) {
        std::cerr << "Failed to get hostname: " << WSAGetLastError() << std::endl;
        closesocket(receive_py);
        WSACleanup();
        return;
    }
    //struct sockaddr_in* local_addr = (struct sockaddr_in*)addr_result->ai_addr;
    //position_addr.sin_addr = local_addr->sin_addr; // Use the local IP address
    position_addr.sin_addr.s_addr = INADDR_ANY; // Using an external python IP address
    //freeaddrinfo(addr_result); // Free the addrinfo structure

    if (bind(receive_py, (struct sockaddr*)&position_addr, sizeof(position_addr)) == SOCKET_ERROR) { // Bind the socket to the address and port
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl; // Check if bind was successful
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(receive_py, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

    char buffer[BUFFER_SIZE];
    struct sockaddr_in python_addr;
    int python_addL = sizeof(python_addr);

//////// 

    int received_bytes = recvfrom(receive_py, buffer, BUFFER_SIZE, 0, 
        (struct sockaddr*)&python_addr, &python_addL);
    
    float x, y, theta;
    
    if (received_bytes == 12) { // change it tp 16 bit which is 2 bytes * 2 = 4bytes
            // Extract float values for logging
            memcpy(&x, buffer, 4);
            memcpy(&y, buffer + 4, 4);
            memcpy(&theta, buffer + 8, 4);
            
            char python_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &python_addr.sin_addr, python_ip, INET_ADDRSTRLEN);
            
            std::cout << "Received coordinates from " << python_ip << ": x=" 
                      << x << ", y=" << y <<  "thehta=" << theta << std::endl;
            transform.SetTransform(x,y,theta);
        } 
        else if(received_bytes == 8){ 
            memcpy(&x, buffer, 4);
            memcpy(&y, buffer + 4, 4);
            
            char python_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &python_addr.sin_addr, python_ip, INET_ADDRSTRLEN);
            
            std::cout << "Received coordinates from " << python_ip << ": x=" 
                      << x << ", y=" << y << std::endl;
            transform.SetTransform(x,y,0.0f); // theta is not used in this case

        }
        else {
            std::cerr << "Received unexpected data size: " << received_bytes << " bytes" << std::endl;
        }
    
    //´procedimiento deone obtengas la info
        // Ensure the socket is closed
        closesocket(receive_py);
        WSACleanup();
    }