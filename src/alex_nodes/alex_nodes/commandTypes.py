from enum import Enum

class CommandType(Enum):
  Current = 1
  Velocity = 2
  Position = 3

  def convertToString(self):
    if self == CommandType.Position:
      return "Position"
    if self == CommandType.Velocity:
      return "Velocity"
    if self == CommandType.Current:
      return "Current"
    raise Exception("Not Defined")