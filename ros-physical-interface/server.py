import socket

# Define the host and port on which the server will listen
HOST = '127.0.0.1'
PORT = 12345

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen(5)
print(f"Server listening on {HOST}:{PORT}")

while True:
    # Accept a new connection
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address}")

    # Receive data from the client
    data = client_socket.recv(1024)
    if not data:
        break

    # Process received data (in this example, just echo it back)
    print(f"Received: {data.decode()}")
    client_socket.sendall(data)

    # Close the connection with the client
    client_socket.close()

# Close the server socket
server_socket.close()