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

class MessageParser():
    def parseStream(self, stream: str) -> tuple[list[DirectedCommand], str]:
        readJSONs = re.split(r'(?<=})\B(?={)', stream)
        if len(readJSONs) == 1:
            return [], stream
    
        commands = []
        for i in range(len(readJSONs) - 1):
            json = readJSONs[i]
            command = self.parseMessage(json)
            if command is not None:
                commands.append(command)
        return commands, readJSONs[-1]

    def parseMessage(self, jsonString: str) -> DirectedCommand:
        try:
            fullJSON = json.loads(jsonString)
            ip = fullJSON.get("IP")
            commandJSON = fullJSON.get("commandJSON")
            commandResult = self.parseCommand(commandJSON)
            if commandResult is None:
                return None
            commandNumber, value = commandResult
            finalCommand = DirectedCommand(ip, commandNumber, value)
            return finalCommand
        except Exception as e:
            print(e)
            return None

    def parseCommand(self, commandJSON: dict) -> tuple[CommandType, float] | None:
        reqTarget = commandJSON.get("reqTarget")
        if (reqTarget == "/m1/setPosition"):
            position = commandJSON.get("position")
            return CommandType.Position.value, position / CONVERSION_RATIO
        elif (reqTarget == "/m1/setVelocity"):
            velocity = commandJSON.get("velocity")
            return CommandType.Velocity.value, velocity / CONVERSION_RATIO
        elif (reqTarget == "/m1/setCurrent"):
            current = commandJSON.get("current")
            return CommandType.Current.value, current
        return None

class CommandRedirector(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()
        self.messageParser = MessageParser()

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

                commands, newStream = self.messageParser.parseStream(stream)
                stream = newStream

                for command in commands:
                    self.send_request(command)
                    self.get_logger().info(f"Received: IP {command.ip} | {command.commandObject.toString()}")


            

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