import socket
import time

def wifi_connect(IP, port): 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    err = s.connect_ex((IP, port))
    if (err != 0):
        print(f"Failed to connect to {IP} {port}")
        return -1
    print(f"Connected to {IP} {port}")
    return s

def wifi_write(s, message):
    message = f"{message}"
    err = s.send(message.encode())
    if (err == 0):
        print("Failed to send message")
        return -1
    print("Sent:", message.strip())
    return
        
def wifi_read(s):
    reply = s.recv(1024).decode().strip()
    if reply:
        print("Replied:", reply)
        return reply
    return -1

def wifi_disconnect(s):
    s.close()

if __name__ == "__main__":
    s = wifi_connect("192.168.32.209", 80)
    if (s == -1):
        exit(1)
    
    while (wifi_write(s, "G") == -1):
        time.sleep(10)
        continue
        
    wifi_disconnect(s)
