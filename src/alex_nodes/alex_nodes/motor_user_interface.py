import math
import os
import sys
import time
from alex_interfaces.srv import Command
import rclpy
import tkinter as tk
import threading
from rclpy.node import Node

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from classes.CommandObject import CommandObject

class ThreadStates():
    def __init__(self):
        self.lock = threading.Lock()
        self.commands = []
        self.closed = False

    def read_command(self) -> None | CommandObject:
        with self.lock:
            if len(self.commands) == 0:
                return None
            value = self.commands.pop(0)
            return value
    
    def add_command(self, command: CommandObject):
        with self.lock:
            self.commands.append(command)

    def close(self):
        self.closed = True



class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()

    def send_request(self, commandObject: CommandObject):
        self.req.command = commandObject.getCommandValue()
        self.req.value = float(commandObject.value)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def run_ros(threadStates: ThreadStates):
    minimal_client = MinimalClientAsync()

    while not threadStates.closed:
        command = threadStates.read_command()
        if command is not None:
            minimal_client.send_request(command)
        time.sleep(1)

    minimal_client.destroy_node()

def run_gui(threadStates: ThreadStates):
    root = tk.Tk()
    frm = tk.Frame(root)
    frm.grid(padx=10, pady=10)
    tk.Label(frm, text="Hello World!").grid(column=0, row=0, padx=10, pady=10)

    def close():
        threadStates.close()
        root.destroy()

    tk.Button(frm, text="Quit", command=close).grid(column=1, row=0, padx=10, pady=10)

    commandOptions = ["Position", "Velocity", "Current"]
    commandVariable = tk.StringVar(frm)
    commandVariable.set(commandOptions[0])

    commandSelect = tk.OptionMenu(frm, commandVariable, *commandOptions)
    commandSelect.grid(column=0, row=1, padx=10, pady=10)

    valueEntry = tk.Entry(frm)
    valueEntry.grid(column=1, row=1, padx=10, pady=10)

    def send_command():
        command = commandOptions.index(commandVariable.get())
        value = float(valueEntry.get())
        threadStates.add_command(CommandObject(command, value))

    tk.Button(frm, text="Send Command", command=send_command).grid(column=0, row=2, padx=10, pady=10)

    root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    threadStates = ThreadStates()

    gui_thread = threading.Thread(target=run_gui, args=(threadStates,))
    gui_thread.start()

    ros_thread = threading.Thread(target=run_ros, args=(threadStates,))
    ros_thread.start()

    ros_thread.join()
    gui_thread.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()