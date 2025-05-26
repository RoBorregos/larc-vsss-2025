
import socket
import struct
import time

def send_coordinates(x_coord, y_coord, relay_ip, relay_port):
    """
    Send two float coordinates to C++ relay via UDP
    Args:
        x_coord (float): X coordinate value
        y_coord (float): Y coordinate value
        relay_ip (str): IP address of the C++ relay
        relay_port (int): UDP port of the C++ relay
    """

    # Create UDP socket and pack the coordinates
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = struct.pack('ff', x_coord, y_coord)  
    sock.sendto(data, (relay_ip, relay_port))
    sock.close()

if __name__ == "__main__":
    
    self_ip = socket.gethostbyname(socket.gethostname()) 
    listen_port = 1234 
        
    try:
        while True:
            try:
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                send_coordinates(x, y, self_ip, listen_port)
                
            except ValueError:
                print("Please enter valid float numbers")           
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")