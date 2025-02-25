import socket

esp32_mac = "E4:65:B8:1B:22:2A"  # Replace with your ESP32 MAC
port = 1

sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
print(f"Connecting to {esp32_mac}...")
sock.connect((esp32_mac, port))
print("Connected!")

try:
    while True:
        message = input("Enter message (or 'exit'): ")
        if message.lower() == "exit":
            break
        message_to_send = (message + "\r\n").encode('utf-8') # Tipe of signal to convert to bytes and send to esp32
        print(f"Sending: {message_to_send}")
        sent = sock.send(message_to_send)
        print(f"Sent {sent} bytes")
except Exception as e:
    print(f"Error: {e}")
finally:
    sock.close()
    print("Closed.")