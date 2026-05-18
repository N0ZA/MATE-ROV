import socket
import ast

# Configuration matching your Teensy's surfaceIP and surfacePort
UDP_IP = "192.168.2.1"  # This must be your PC's static IP
UDP_PORT = 5000           # Port specified in your Teensy code
BUFFER_SIZE = 1024

# Create and bind the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for Teensy telemetry on {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        # Receive the raw bytes packet
        raw_data, addr = sock.recvfrom(BUFFER_SIZE)
        
        # 1. Decode bytes to string and strip the trailing newline (\n)
        clean_string = raw_data.decode('utf-8').strip()
        
        # 2. Convert the text "[roll, depth]" safely into a Python list
        telemetry_array = ast.literal_eval(clean_string)
        
        # Print the actual array
        print(telemetry_array)

except KeyboardInterrupt:
    print("\nStopping receiver.")
finally:
    sock.close()