#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
// Link with Winsock library
#pragma comment(lib, "ws2_32.lib")

#define BUFFER_SIZE 255 // Define the buffer size for receiving data

#define PYPORT 1234
#define ROBOT1PORT 1235
#define ROBOT2PORT 1236
#define ROBOT3PORT 1237

int main(){
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock with version 2.2 
    if (result != 0) { // Check if WSAStartup was successful
        std::cerr << "WSAStartup failed: " << result << std::endl;
        return 1;
    }
    SOCKET receive_py = socket(AF_INET, SOCK_DGRAM,0); // Create a UDP socket, using IPv4 (AF_INET) and datagram (SOCK_DGRAM) protocol for UDP
    if (receive_py == INVALID_SOCKET) { // Check if socket creation was successful
        std::cerr << "Fail receive socket: " << WSAGetLastError() << std::endl;
        WSACleanup(); // Clean up Winsock
        return 1;
    }
    SOCKET send_robot1 = socket(AF_INET, SOCK_DGRAM,0); // Create a UDP socket for sending to robot 1
    if (send_robot1 == INVALID_SOCKET) { // Check if socket creation was successful
        std::cerr << "Fail send robot [1] socket: " << WSAGetLastError() << std::endl;
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return 1;
    }
    // repeat for robot2 and robot3
    

    struct sockaddr_in vision_ball_addr; // Define a structure to hold the server address information for the vision ball
    memset(&vision_ball_addr, 0, sizeof(vision_ball_addr)); // Initialize the server address structure to zero   
    vision_ball_addr.sin_family = AF_INET; // Set the address family to IPv4
    vision_ball_addr.sin_port = htons(PYPORT); // Set the port number for the server (PYPORT) and convert it to network byte order
    vision_ball_addr.sin_addr.s_addr = INADDR_ANY; // Set the IP address to accept connections from any address
    
    if (bind(receive_py, (struct sockaddr*)&vision_ball_addr, sizeof(vision_ball_addr)) == SOCKET_ERROR) { // Bind the socket to the address and port
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl; // Check if bind was successful
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return 1;
    }



    // Define the address structure for sending data to robot 1
    struct sockaddr_in robot1_addr; // Define a structure to hold the server address information for robot 1
    memset(&robot1_addr, 0, sizeof(robot1_addr)); // Initialize the server address structure to zero
    robot1_addr.sin_family = AF_INET; // Set the address family to IPv4
    robot1_addr.sin_port = htons(ROBOT1PORT); // Set the port number for the server (ROBOT1PORT) and convert it to network byte order

    const char * robot1_ip = "192.168.1.8"; // Define the IP address for robot 1
    if (inet_pton(AF_INET, robot1_ip, &robot1_addr.sin_addr) <= 0) { // Convert the IP address from string to binary form
        std::cerr << "Invalid address for robot 1: " << robot1_ip << std::endl; // Check if the IP address conversion was successful
        closesocket(receive_py); // Close the receive socket
        closesocket(send_robot1); // Close the send socket for robot 1
        WSACleanup(); // Clean up Winsock
        return 1;
    }


    while (true) {
        char buffer[BUFFER_SIZE];
        struct sockaddr_in python_addr;
        int python_addr_len = sizeof(python_addr);
        
        // Receive data from Python
        int received_bytes = recvfrom(receive_py, buffer, BUFFER_SIZE, 0, 
                                     (struct sockaddr*)&python_addr, &python_addr_len);
        
        if (received_bytes == SOCKET_ERROR) {
            std::cerr << "Error receiving data: " << WSAGetLastError() << std::endl;
            continue;
        }
        
        // Check if we received the expected 8 bytes (two floats) 
            // here we are receiving 8 bytes (4 bytes for x and 4 bytes for y) (need to optimize this)!!!!!
        if (received_bytes == 8) { // change it tp 16 bit which is 2 bytes * 2 = 4bytes
            // Extract float values for logging
            float x_coord, y_coord;
            memcpy(&x_coord, buffer, 4);
            memcpy(&y_coord, buffer + 4, 4);
            
            char python_ip[INET_ADDRSTRLEN]; // Convert the IP address to string format
            inet_ntop(AF_INET, &python_addr.sin_addr, python_ip, INET_ADDRSTRLEN);
            
            std::cout << "Received coordinates from " << python_ip << ": x=" 
                      << x_coord << ", y=" << y_coord << std::endl;
            
            // Forward the data to ESP32
            int sent_bytes = sendto(send_robot1, buffer, received_bytes, 0,
                                   (struct sockaddr*)&robot1_addr, sizeof(robot1_addr));
            
            if (sent_bytes == SOCKET_ERROR) {
                std::cerr << "Failed to forward data to ESP32: " << WSAGetLastError() << std::endl;
            } else {
                std::cout << "Forwarded coordinates to ESP32" << std::endl;
            }
        } else {
            std::cerr << "Received unexpected data size: " << received_bytes << " bytes" << std::endl;
        }
    }
    
    // Close sockets and cleanup Winsock
    // (This code is unreachable in this example but good practice)
    closesocket(receive_py);
    closesocket(send_robot1);
    WSACleanup();
}