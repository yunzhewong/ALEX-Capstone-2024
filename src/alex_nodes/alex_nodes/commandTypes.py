from enum import Enum

class CommandType(Enum):
  Position = 0
  Velocity = 1
  Current = 2

  def convertToString(self):
    if self == CommandType.Position:
      return "Position"
    if self == CommandType.Velocity:
      return "Velocity"
    if self == CommandType.Current:
      return "Current"
    raise Exception("Not Defined")