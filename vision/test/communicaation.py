import socket
import struct
import time

def send_coordinates(x_coord, y_coord, relay_ip, relay_port=1234):
    """
    Send two float coordinates to C++ relay via UDP
    Args:
        x_coord (float): X coordinate value
        y_coord (float): Y coordinate value
        relay_ip (str): IP address of the C++ relay
        relay_port (int): UDP port of the C++ relay
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = struct.pack('ff', x_coord, y_coord) 
    sock.sendto(data, (relay_ip, relay_port))
    sock.close()

if __name__ == "__main__":
    # C++ relay IP address - update this to match your relay's IP
    RELAY_IP = "10.22.234.26"  # Replace with your C++ program's device IP
    RELAY_PORT = 1234
    
    try:
        while True:
            try:
                # Get coordinates from user input
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                
                # Send the coordinates
                send_coordinates(x, y, RELAY_IP, RELAY_PORT)
                
            except ValueError:
                print("Please enter valid float numbers")
            
            # Wait a bit before next input
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")