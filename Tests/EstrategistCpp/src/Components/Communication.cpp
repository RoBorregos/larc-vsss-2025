#include "Communication.h"
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

Communication::Communication(Transform& t, int id, int portA, int portB) : robotID(id), portR(portA), portS(portB), transform(t) {
    ips[1] = "192.168.0.216"; // Attacker Dflt
    ips[2] = "192.168.0.218"; //Defender Dflt
    ips[3] = "192.168.0.219"; // extra 
}


int Communication::SendData(Output data) 
    {
        // Init Winsock
        std::string ip = ips[robotID]; // Get the IP address for the robot ID
        //cout<< "                            Sending to Ip = "<<ip<<" For ID = "<<robotID<<endl;
        WSADATA wsaData;
        int startupResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (startupResult != 0) {
            return 2;
        }
    
        SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udpSocket == INVALID_SOCKET) {
            WSACleanup();
            return 3;
        }
    
        // Define ESP32 address
        sockaddr_in esp32Addr;
        memset(&esp32Addr, 0, sizeof(esp32Addr));
        esp32Addr.sin_family = AF_INET; // IPV04
        esp32Addr.sin_port = htons(this->portS);  // Use the port passed to the object
    
        // Set the ESP32 IP (replace with the real IP)
        
        
        const char* esp32_ip = ip.c_str() ; // pointer to const char 
        if (inet_pton(AF_INET, esp32_ip, &esp32Addr.sin_addr) <= 0) {
            closesocket(udpSocket);
            WSACleanup();
            return 4;
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
            closesocket(udpSocket);
            WSACleanup();
            return 5;
        }     
        return 0;
}

int Communication::ReceiveData() {
    #define BUFFER_SIZE 255 // Define the buffer size for receiving data


    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock with version 2.2 
    if (result != 0) { // Check if WSAStartup was successful
        return 2;
    }

    SOCKET receive_py = socket(AF_INET, SOCK_DGRAM,0); // Create a UDP socket, using IPv4 (AF_INET) and datagram (SOCK_DGRAM) protocol for UDP
    if (receive_py == INVALID_SOCKET) { // Check if socket creation was successful
        WSACleanup(); // Clean up Winsock
        return 3;
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
    position_addr.sin_port = htons(this->portR); // Set the port number for the server (PYPORT) and convert it to network byte order


    char hostname[NI_MAXHOST]; // Buffer to store the hostname
    if (gethostname(hostname, sizeof(hostname)) == SOCKET_ERROR) {
        closesocket(receive_py);
        WSACleanup();
        return 4;
    }
    //struct sockaddr_in* local_addr = (struct sockaddr_in*)addr_result->ai_addr;
    //position_addr.sin_addr = local_addr->sin_addr; // Use the local IP address
    position_addr.sin_addr.s_addr = INADDR_ANY; // Using an external python IP address
    //freeaddrinfo(addr_result); // Free the addrinfo structure



    int opt = 1;
    if (setsockopt(receive_py, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) == SOCKET_ERROR) {
       // std::cerr << "Failed to set SO_REUSEADDR: " << WSAGetLastError() << std::endl;
        closesocket(receive_py);
        WSACleanup();
        return 6; // Return a new error code for this failure
    }


    if (bind(receive_py, (struct sockaddr*)&position_addr, sizeof(position_addr)) == SOCKET_ERROR) { // Bind the socket to the address and port
      cout<<"                   Port: "<<this->portR<<endl;
      int error_code = WSAGetLastError();
      std::cerr << "Bind failed with error: " << error_code << std::endl;
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return 5;
    }

    char buffer[BUFFER_SIZE] = {0};
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
            
          //  std::cout << "Received coordinates from " << python_ip << ": x=" 
               //       << x << ", y=" << y <<  "thehta=" << theta << std::endl;
               if(x== 0 && y == 0){
                   std::cout<<"Recived 0"<<endl;
                   
                   closesocket(receive_py);
                   WSACleanup();
                   return 0;
               }
                transform.SetTransform(x,y,theta);
        } 
        else if(received_bytes == 8) { // change it tp 16 bit which is 2 bytes * 2 = 4bytes
            memcpy(&x, buffer, 4);
            memcpy(&y, buffer + 4, 4);
            
            char python_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &python_addr.sin_addr, python_ip, INET_ADDRSTRLEN);
            
         //   std::cout << "Received coordinates from " << python_ip << ": x=" 
          //            << x << ", y=" << y << std::endl;
            transform.SetTransform(x,y,0.0f); // theta is not used in this case
        closesocket(receive_py);
        WSACleanup();

        }
        else {
            std::cerr << "Received unexpected data size: " << received_bytes << " bytes" << std::endl;
            if (received_bytes == SOCKET_ERROR) {
                int error_code = WSAGetLastError();
                std::cerr << "recvfrom failed with error: " << error_code << std::endl;
                
            }
            closesocket(receive_py);
            WSACleanup();
            return 1;
        }
        closesocket(receive_py);
        WSACleanup();
        return 0;
    
    //´procedimiento deone obtengas la info
        // Ensure the socket is closed

    }