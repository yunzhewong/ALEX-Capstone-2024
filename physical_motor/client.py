import socket
from serverConstants import ROS_HOST, ROS_PORT

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ROS_HOST, ROS_PORT))

message = "10.10.10.17|0|5"
client_socket.sendall(message.encode())
data = client_socket.recv(1024)
print(data)

client_socket.close()
