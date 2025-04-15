#include <iostream>
#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>

// Link with Winsock library
#pragma comment(lib, "ws2_32.lib")

void Ballpos(float& x, float& y, int port = 1234) { // Default port is 1234 for the archive in python
    // Variable initialization
    WSADATA wsaData;
    SOCKET receive_socket = INVALID_SOCKET;
    struct sockaddr_in receive_addr, sender_addr;
    int sender_addr_len = sizeof(sender_addr);
    char buffer[255];
    int received_bytes;
    
    try {
        // Initialize Winsock
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed" << std::endl;
            return; // Early return on error
        }
        
        // Create socket for receiving data
        receive_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (receive_socket == INVALID_SOCKET) {
            throw std::runtime_error("Failed to create socket: " + std::to_string(WSAGetLastError()));
        }
        
        // Setup address structure for receiving data
        memset(&receive_addr, 0, sizeof(receive_addr));
        receive_addr.sin_family = AF_INET;
        receive_addr.sin_port = htons(port);
        receive_addr.sin_addr.s_addr = INADDR_ANY;
        
        // Bind the socket
        if (bind(receive_socket, (struct sockaddr*)&receive_addr, sizeof(receive_addr)) == SOCKET_ERROR) {
            throw std::runtime_error("Failed to bind socket: " + std::to_string(WSAGetLastError()));
        }
        
        // Set socket to non-blocking mode with a timeout
        DWORD timeout = 5000; // 5 seconds timeout
        if (setsockopt(receive_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) {
            throw std::runtime_error("Failed to set socket timeout: " + std::to_string(WSAGetLastError()));
        }
        
        // Receive data
        received_bytes = recvfrom(receive_socket, buffer, sizeof(buffer), 0,
                                 (struct sockaddr*)&sender_addr, &sender_addr_len);
        
        if (received_bytes == SOCKET_ERROR) {
            if (WSAGetLastError() == WSAETIMEDOUT) {
                throw std::runtime_error("Receive timeout");
            } else {
                throw std::runtime_error("Error receiving data: " + std::to_string(WSAGetLastError()));
            }
        }
        
        // Check if we received the expected 8 bytes (two floats)
        // Later is going to be change to 12 bytes (three floats) for the ball position and angle
        if (received_bytes != 8) {
            throw std::runtime_error("Received unexpected data size: " + std::to_string(received_bytes) + " bytes");
        }
        
        // Extract the X and Y coordinates from the buffer
        memcpy(&x, buffer, 4);
        memcpy(&y, buffer + 4, 4);
        
        // Log the received values
        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);
        std::cout << "Received coordinates from " << sender_ip << ": x=" << x << ", y=" << y << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        // Set default values in case of error
        x = 0.0f;
        y = 0.0f;
    }
    
    // Clean up
    if (receive_socket != INVALID_SOCKET) {
        closesocket(receive_socket);
    }
    WSACleanup();
}