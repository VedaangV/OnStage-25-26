import socket
import time

PICO_IP = "192.168.32.209"  # StormingKids IP
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PICO_IP, PORT))
print("Connected to Pico W")

def wifi_write(message):
    sock.send(message.encode())
    print("Sent:", message.strip())
    return
        
def wifi_read():
    reply = sock.recv(1024).decode().strip()
    print("Pico replied:", reply)
    return reply

if __name__ == "__main__":
    try:
        while True:
            message = "Hello world\n"
            wifi_write(message)
            time.sleep(1)
            reply = wifi_read()
            print(reply)
            
            if 0xFF == ord('q'): 
                break

    except KeyboardInterrupt:
        print("Closing connection.")

    sock.close()          
