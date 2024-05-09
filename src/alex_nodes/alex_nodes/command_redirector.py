import os
import sys
from alex_interfaces.srv import Command
import rclpy
from rclpy.node import Node
import socket

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from alex_nodes.classes.CommandObject import CommandObject
from serverConstants import ROS_HOST, ROS_PORT

class CommandRedirector(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((ROS_HOST, ROS_PORT ))
        server_socket.listen(1)
        self.get_logger().info(f"Server listening on {ROS_HOST}:{ROS_PORT}")
        while True: 
            client_socket, _ = server_socket.accept()

            data = client_socket.recv(1024)
            if not data:
                break

            self.get_logger().info(f"Received: {data.decode()}")

            returnBytes = "Received".encode()
            client_socket.sendall(returnBytes)

            client_socket.close()

    def send_request(self, commandObject: CommandObject):
        self.req.command = commandObject.getCommandValue()
        self.req.value = float(commandObject.value)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = CommandRedirector()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()