import socket
from serverConstants import HOST, PORT, RECEIVED

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

message = "Hello, server!"
client_socket.sendall(message.encode())
data = client_socket.recv(1024)
if data.decode() != RECEIVED:
    raise Exception("Failed")

# Close the connection with the server
client_socket.close()
