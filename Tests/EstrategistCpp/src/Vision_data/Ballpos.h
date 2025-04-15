#ifndef POSITION_RECEIVER_H
#define POSITION_RECEIVER_H

/**
 * Receives UDP data and extracts X and Y position coordinates
 * Sets up socket, receives data, processes it, and returns the position values
 * 
 * @param x Reference to store the extracted X coordinate value
 * @param y Reference to store the extracted Y coordinate value
 * @param port The UDP port to listen on (default: 1234)
 * @return true if coordinates were successfully received and extracted, false otherwise
 */
void Ballpos(float& x, float& y, int port = 1234);

#endif // POSITION_RECEIVER_H