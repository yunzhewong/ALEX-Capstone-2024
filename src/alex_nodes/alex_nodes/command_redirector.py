import os
import sys
from alex_interfaces.srv import Command
import rclpy
from rclpy.node import Node
import socket
import json
import re
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


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
            return reqTarget, DirectedCommand(self.ip, CommandType.Position, position / CONVERSION_RATIO)
        elif (reqTarget == "/m1/setVelocity"):
            velocity = self.throwIfMissingGet("velocity")            
            return reqTarget, DirectedCommand(self.ip, CommandType.Velocity, velocity / CONVERSION_RATIO)
        elif (reqTarget == "/m1/setCurrent"):
            current = self.throwIfMissingGet("current")
            return reqTarget, DirectedCommand(self.ip, CommandType.Current, current)
        return None
        
    def throwIfMissingGet(self, key):
        value = self.commandJSON.get(key, None)
        if value is None:
            raise Exception("Not convertable")
        return value

class StreamParser():
    def parseStream(self, stream) -> tuple[list[DirectedCommand], str]:
        separatedJSONStrings = re.split(r'(?<=})\B(?={)', stream)
        target_commands = []
        newStream = ""
        for (i, jsonString) in enumerate(separatedJSONStrings):
            try:
                command = ParsedJSON(json.loads(jsonString)).convertToCommand() # check if loadable
                if command is not None:
                    target_commands.append(command)
            except:
                newStream = "".join(separatedJSONStrings[i:])
                break

        return target_commands, newStream

class CommandRedirector(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()
        self.streamParser = StreamParser()

        self.current_subscriber = self.create_subscription(Float64MultiArray, "/currents", self.read_current, 10)
        self.currents = []

        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_states", self.read_joints, 10)
        self.joints = []

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((ROS_HOST, ROS_PORT))
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

                target_commands, nextStream = self.streamParser.parseStream(stream)
                stream = nextStream

                cvps = []
                for i in range(len(self.joints)):
                    cvps.append((self.joints[i][0], self.currents[i], self.joints[i][1], self.joints[i][2]))

                for target_command in target_commands:
                    reqTarget, command = target_command
                    self.send_request(command)

                    if len(cvps) != 0:
                        formatted_str = json.dumps({
                            "status": "OK",
                            "reqTarget": reqTarget,
                            "current": cvps[0][1],
                            "velocity": cvps[0][2],
                            "position": cvps[0][3],
                        })
                        client_socket.send(formatted_str.encode("utf-8"))

    def send_request(self, directedCommand: DirectedCommand):
        self.req.ip = directedCommand.ip
        self.req.command = directedCommand.commandObject.getCommandValue()
        self.req.value = float(directedCommand.commandObject.value)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def read_current(self, msg: Float64MultiArray):
        self.currents = msg.data[:]

    def read_joints(self, msg: JointState):
        self.joints = []

        for i in range(len(msg.position)):
            self.joints.append((msg.name[i], msg.velocity[i], msg.position[i]))


def main(args=None):
    rclpy.init(args=args)

    minimal_client = CommandRedirector()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()