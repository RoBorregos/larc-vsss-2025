#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#define BUFFER_SIZE 255 /
#define PYPORT 1234
#define ROBOT1PORT 1233
#define ROBOT2PORT 1233
#define ROBOT3PORT 1233

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
    if (send_robot1 == INVALID_SOCKET) {
        std::cerr << "Fail send robot [1] socket: " << WSAGetLastError() << std::endl;
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return 1;
    }
    

    /*
    sockaddr_in -> Define the address structure for receiving data from Python
    memset -> Initialize the server address structure to zero
    .sin_family -> Set the address family to IPv4
    .sin_port -> Set the port number for the server (PYPORT) and convert it to network byte order
    .sin_addr.s_addr -> Set the IP address to accept connections from any address
    */

    struct sockaddr_in vision_ball_addr; 
    memset(&vision_ball_addr, 0, sizeof(vision_ball_addr)); 
    vision_ball_addr.sin_family = AF_INET; 
    vision_ball_addr.sin_port = htons(PYPORT); 
    vision_ball_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(receive_py, (struct sockaddr*)&vision_ball_addr, sizeof(vision_ball_addr)) == SOCKET_ERROR) { // Bind the socket to the address and port
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl; // Check if bind was successful
        closesocket(receive_py); // Close the receive socket
        WSACleanup(); // Clean up Winsock
        return 1;
    }



    struct sockaddr_in robot1_addr; 
    memset(&robot1_addr, 0, sizeof(robot1_addr)); 
    robot1_addr.sin_family = AF_INET; 
    robot1_addr.sin_port = htons(ROBOT1PORT);

    const char * robot1_ip = "192.168.0.6"; 
    //Convert the IP address from string to binary form
    if (inet_pton(AF_INET, robot1_ip, &robot1_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address for robot 1: " << robot1_ip << std::endl; 
        closesocket(receive_py); 
        closesocket(send_robot1); 
        WSACleanup();
        return 1;
    }


    while (true) {
        char buffer[BUFFER_SIZE]; 
        struct sockaddr_in python_addr;
        int python_addr_len = sizeof(python_addr);
                int received_bytes = recvfrom(receive_py, buffer, BUFFER_SIZE, 0, 
                                     (struct sockaddr*)&python_addr, &python_addr_len);
        
        if (received_bytes == SOCKET_ERROR) {
            std::cerr << "Error receiving data: " << WSAGetLastError() << std::endl;
            continue;
        }
        
        if (received_bytes == 8) { 
            float x_coord, y_coord;
            memcpy(&x_coord, buffer, 4);
            memcpy(&y_coord, buffer + 4, 4);
            
            char python_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &python_addr.sin_addr, python_ip, INET_ADDRSTRLEN);
            
            std::cout << "Received coordinates from " << python_ip << ": x=" 
                      << x_coord << ", y=" << y_coord << std::endl;
            
            
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
    closesocket(receive_py);
    closesocket(send_robot1);
    WSACleanup();
}