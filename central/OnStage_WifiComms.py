import socket
import time

PICO_IP = "192.168.32.209"  # StormingKids IP
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PICO_IP, PORT))
print("Connected to Pico W")

def wifi_write(message):
    errorval = sock.send(message.encode())
    if (errorval == 0):
        print("Failed to send message")
        return -1
    print("Sent:", message.strip())
    return
        
def wifi_read():
    reply = sock.recv(1024).decode().strip()
    if reply:
        print("Pico replied:", reply)
        return reply
    return -1

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
