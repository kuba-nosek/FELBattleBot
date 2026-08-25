import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 4444

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Nasloucham na portu {UDP_PORT}...")
print("Pripoj se na Macu k Wi-Fi 'MeltyBot_Debug' a zapni robota.\n")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"[{addr[0]}]: {data.decode('utf-8')}")
except KeyboardInterrupt:
    print("\nUkonceno uzivatelem.")
    sock.close()