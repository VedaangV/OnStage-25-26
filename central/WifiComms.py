import socket
import time

PICO_IP = "192.168.32.172"   # StormingKids
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PICO_IP, PORT))
print("Connected to Pico W")
i = 0
try:
    while True:
        message = str(i)+"\n"
        i += 1
        sock.send(message.encode())
        print("Sent:", message.strip())

        # Optional: read response
        reply = sock.recv(1024).decode().strip()
        print("Pico replied:", reply)

        time.sleep(1)

except KeyboardInterrupt:
    print("Closing connection.")

sock.close()
