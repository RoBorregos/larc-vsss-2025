import socket  # For Windows/Linux (pybluez)
import time

# Replace with your ESP32's Bluetooth MAC address (see below on how to find it)
esp32_mac = "E4:65:B8:1B:22:2A"  # Example format
port = 1  # Default RFCOMM port for Bluetooth Serial

# Create a Bluetooth socket
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

# Connect to the ESP32
print(f"Connecting to {esp32_mac}...")
sock.connect((esp32_mac, port))
print("Connected!")

# Send messages
try:
    while True:
        message = input("Enter message to send to ESP32 (or 'exit' to quit): ")
        if message.lower() == "exit":
            break
        message = (message + "\n").encode()
        sock.send(message)  # Add newline for ESP32 to read properly
        print(f"Sent: {message}")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    sock.close()
    print("Connection closed.")