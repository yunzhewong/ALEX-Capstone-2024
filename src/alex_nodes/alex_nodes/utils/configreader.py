import json
import sys
from typing import List

def get_value(obj: dict, key: str):
  value = obj.get(key, None)
  if value is None:
    raise Exception(f"{key} not found")
  return value

def read_config():
  cmd_args = sys.argv[1:]

  if len(cmd_args) == 0:
    raise Exception("Too few args")
  
  config_source = cmd_args[0]
  
  file = open(config_source, "r")

  data = json.loads(file.read())


  counts = get_value(data, "count")
  motor_dicts = get_value(data, "motors") 

  configs: List[MotorConfiguration] = []
  for motor_dict in motor_dicts:
    configs.append(MotorConfiguration(motor_dict))

  file.close()


  return counts, configs

class MotorConfiguration():
  def __init__(self, dict):
    self.name = get_value(dict, "name")
    self.ip = get_value(dict, "ip")
    self.motor_constant = get_value(dict, "motor_constant")
    self.position_p = get_value(dict, "position_p")
    self.vel_p = get_value(dict, "vel_p")
    self.vel_i = get_value(dict, "vel_i")
    self.friction_adjustment = get_value(dict, "friction_adjustment")
