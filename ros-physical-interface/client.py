import socket

# Define the server's address and port
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 12345

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the server's address and port
client_socket.connect((SERVER_HOST, SERVER_PORT))

# Send data to the server
message = "Hello, server!"
client_socket.sendall(message.encode())

# Receive data from the server
data = client_socket.recv(1024)
print(f"Received: {data.decode()}")

# Close the connection with the server
client_socket.close()
