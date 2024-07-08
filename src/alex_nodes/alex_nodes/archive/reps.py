from enum import Enum
import math
import os
import rclpy
from rclpy.node import Node
from alex_interfaces.msg import Command
from sensor_msgs.msg import JointState

import sys
package_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(package_dir)

from motor_pubsub_utils.constants import SEND_PERIOD
from twinmotor import IPS
from commandTypes import CommandType

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3
    
def commands(self, t):
      MAX_ANGLE = math.pi / 2
      MAX_TIME = 10
      STARTING_ANGLE = -math.pi / 2
      RESET_TIME = 5
      PAUSE_TIME = 1
      START_CURRENT = 0.1
      INCREMENT = 0.1
      
      if len(self.positions) < 2:
          return

      if self.state == State.Collecting:
          command = START_CURRENT + INCREMENT * self.collect_index
          if not self.initialised:
              self.collecting_start = t
              self.initialised = True
              print(command)

          exceeded_max_angle = self.positions[1] >= MAX_ANGLE
          exceeded_max_time = (t - self.collecting_start) >= MAX_TIME

          should_stop = exceeded_max_angle or exceeded_max_time

          if should_stop:
              self.state = State.Paused
              self.post_pause_state = State.Resetting
              self.collect_index += 1
              self.initialised = False

          self.types = [CommandType.Current.value, CommandType.Current.value] 
          self.values = [0.0, command]
      elif self.state == State.Paused:
          if not self.initialised:
              self.pause_end_time = t + PAUSE_TIME
              self.initialised = True

          if t > self.pause_end_time:
              self.state = self.post_pause_state
              self.initialised = False

          self.types = [CommandType.Current.value, CommandType.Current.value] 
          self.values = [0.0, 0.0]
      elif self.state == State.Resetting:
          if not self.initialised:
              self.reset_start = t
              self.reset_end = t + RESET_TIME
              self.reset_angle = self.positions[1]
              self.initialised = True

          expected_pos = self.reset_angle - ((self.reset_angle - STARTING_ANGLE) / RESET_TIME) * (t - self.reset_start)

          if t > self.reset_end:
              self.post_pause_state = State.Collecting
              self.state = State.Paused
              self.initialised = False
          
          self.types = [CommandType.Current.value, CommandType.Position.value] 
          self.values = [0.0, float(expected_pos)]