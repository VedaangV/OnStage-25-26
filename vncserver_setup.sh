sudo apt update && sudo apt install x11vnc -y
x11vnc -storepasswd
x11vnc -display :0 -forever -bg -noxdamage -repeat -rfbport 5900 -rfbauth ~/.vnc/passwd
echo "Connect to 10.42.0.1:5900 in RealVNC"
