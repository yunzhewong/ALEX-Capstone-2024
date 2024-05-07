import socket
from serverConstants import HOST, PORT, RECEIVED

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))

server_socket.listen(1)
print(f"Server listening on {HOST}:{PORT}")

try:
    while True:
        client_socket, client_address = server_socket.accept()

        data = client_socket.recv(1024)
        if not data:
            break

        print(f"Received: {data.decode()}")

        returnBytes = RECEIVED.encode()
        client_socket.sendall(returnBytes)

        client_socket.close()
except KeyboardInterrupt:
    server_socket.close()