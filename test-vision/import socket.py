import socket
import struct
import time


def send_coordinates(x_coord, y_coord, esp32_ip, esp32_port=1111):
    """
    Send two float coordinates to ESP32 via UDP
    Args:
        x_coord (float): X coordinate value
        y_coord (float): Y coordinate value
        esp32_ip (str): IP address of the ESP32
        esp32_port (int): UDP port of the ESP32
    """
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Pack the two float values into bytes
    # 'ff' format means two 32-bit float values
    data = struct.pack('ff', x_coord, y_coord)
    
    # Send the data
    sock.sendto(data, (esp32_ip, esp32_port))
    print(f"Sent coordinates: x={x_coord}, y={y_coord} to {esp32_ip}:{esp32_port}")
    
    # Close the socket
    sock.close()

if __name__ == "__main__":
    # ESP32 IP address - update this to match your ESP32's IP
    ESP32_IP = "192.168.137.207"  # Replace with your ESP32's actual IP
    ESP32_PORT = 1111
    
    try:
        while True:
            try:
                # Get coordinates from user input
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                
                # Send the coordinates
                send_coordinates(x, y, ESP32_IP, ESP32_PORT)
                
            except ValueError:
                print("Please enter valid float numbers")
            
            # Wait a bit before next input
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")