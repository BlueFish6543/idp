import socket

UDP_IP = "192.168.43.225"
UDP_PORT = 2390
message = "Hello world!"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(message.encode(), (UDP_IP, UDP_PORT))