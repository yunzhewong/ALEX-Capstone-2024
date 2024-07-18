from io import TextIOWrapper
import sys
from typing import List

def get_value(file: TextIOWrapper):
  line = file.readline()
  return line.split("=")[-1]

def read_config():
  cmd_args = sys.argv[1:]

  if len(cmd_args) == 0:
    raise Exception("Too few args")
  
  config_source = cmd_args[0]
  
  file = open(config_source, "r")

  counts = int(get_value(file))

  configs: List[MotorConfiguration] = []
  for _ in range(counts):
    file.readline() # blank space
    configs.append(MotorConfiguration(file))

  file.close()
  return counts, configs

class MotorConfiguration():
  def __init__(self, file: TextIOWrapper):
    self.name = get_value(file)
    self.ip = get_value(file)
    self.motor_constant = float(get_value(file))
    self.position_p = float(get_value(file))
    self.vel_p = float(get_value(file))
    self.vel_i = float(get_value(file))
    self.friction_adjustment = float(get_value(file))
