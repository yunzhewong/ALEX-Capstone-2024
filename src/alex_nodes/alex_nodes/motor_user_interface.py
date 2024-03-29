import math
import os
import sys
from alex_interfaces.srv import Command
import rclpy
import tkinter as tk
import threading
from rclpy.node import Node

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

import commandTypes as CommandType

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Command, 'command_receiver')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()

    def send_request(self):
        self.req.command = CommandType.POSITION
        self.req.value = float(0)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def send_command():
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f"Received: {response.received}")
    minimal_client.destroy_node()

def run_gui():
    root = tk.Tk()
    frm = tk.Frame(root)
    frm.grid(padx=10, pady=10)
    tk.Label(frm, text="Hello World!").grid(column=0, row=0, padx=10, pady=10)
    tk.Button(frm, text="Quit", command=root.destroy).grid(column=1, row=0, padx=10, pady=10)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    gui_thread = threading.Thread(target=run_gui)
    gui_thread.start()

    gui_thread.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()