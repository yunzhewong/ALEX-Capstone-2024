import os
import sys
from alex_interfaces.srv import Command
import rclpy
from rclpy.node import Node
import socket
import json
import re
import math

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from alex_nodes.commandTypes import CommandType
from alex_nodes.classes.Commands import CommandObject, DirectedCommand
from alex_nodes.serverConstants import ROS_HOST, ROS_PORT

CONVERSION_RATIO = 100000 / math.pi

class ParsedJSON():
    def __init__(self, jsonData: dict):
        self.ip = jsonData.get("IP", None)
        self.commandJSON = jsonData.get("commandJSON", None)

        if self.ip is None or self.commandJSON is None:
            raise Exception("JSON not parsable")
        
    def convertToCommand(self):
        reqTarget = self.throwIfMissingGet("reqTarget")

        if (reqTarget == "/m1/setPosition"):
            position = self.throwIfMissingGet("position")            
            return DirectedCommand(self.ip, CommandType.Position, position / CONVERSION_RATIO)
        elif (reqTarget == "/m1/setVelocity"):
            velocity = self.throwIfMissingGet("velocity")            
            return DirectedCommand(self.ip, CommandType.Velocity, velocity / CONVERSION_RATIO)
        elif (reqTarget == "/m1/setCurrent"):
            current = self.throwIfMissingGet("current")
            return DirectedCommand(self.ip, CommandType.Current, current)
        
    def throwIfMissingGet(self, key):
        value = self.commandJSON.get(key, None)
        if value is None:
            raise Exception("Not convertable")
        return value

class StreamParser():
    def parseStream(self, stream) -> tuple[list[DirectedCommand], str]:
        separatedStrings = re.split(r'(?<=})\B(?={)', stream)
        commands = []
        newStream = ""
        for readString in separatedStrings:
            try:
                command = ParsedJSON(json.loads(readString)).convertToCommand() # check if loadable
                commands.append(command)
            except:
                newStream = readString
                break

        return commands, newStream

class CommandRedirector(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()
        self.streamParser = StreamParser()

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((ROS_HOST, ROS_PORT ))
        server_socket.listen(1)
        self.get_logger().info(f"Server listening on {ROS_HOST}:{ROS_PORT}")

        while True:
            client_socket, _ = server_socket.accept()

            stream = ""
            while True:
                data = client_socket.recv(1024)
                if not data:
                    client_socket.close()
                    break

                stream += data.decode("utf-8")

                commands, nextStream = self.streamParser.parseStream(stream)
                stream = nextStream

                for command in commands:
                    self.send_request(command)

    def send_request(self, directedCommand: DirectedCommand):
        self.req.ip = directedCommand.ip
        self.req.command = directedCommand.commandObject.getCommandValue()
        self.req.value = float(directedCommand.commandObject.value)
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