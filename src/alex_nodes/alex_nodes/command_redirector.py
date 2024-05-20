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

class StreamParser():
    def __init__(self):
        self.messageParser = MessageParser()
    
    def parseStream(self, stream) -> tuple[list[DirectedCommand], str]:
        separatedStrings = re.split(r'(?<=})\B(?={)', stream)
        parsedJSONs, newStream = self.extractJSONs(separatedStrings)
        commands = self.createCommands(parsedJSONs)
        return commands, newStream
    
    def extractJSONs(self, separatedStrings):
        parsedJSONs = []
        newStream = ""
        for readString in separatedStrings:
            try:
                readJSON = json.loads(readString) # check if loadable
                parsedJSONs.append(readJSON)
            except:
                newStream = readString

        return parsedJSONs, newStream
    
    def createCommands(self, parsedJSONs):
        commands = []
        for parsedJSON in parsedJSONs:
            command = self.messageParser.parseMessage(parsedJSON)
            if command is not None:
                commands.append(command)
        return commands


class MessageParser():
    def parseMessage(self, parsedJSON: dict) -> DirectedCommand:
        try:
            ip = parsedJSON.get("IP")
            commandJSON = parsedJSON.get("commandJSON")
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

                commands, newStream = self.streamParser.parseStream(stream)
                stream = newStream

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