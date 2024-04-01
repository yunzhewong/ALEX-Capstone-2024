POSITION = 0
VELOCITY = 1
CURRENT = 2

def convertToString(value):
  if value == POSITION:
    return "Position"
  if value == VELOCITY:
    return "Velocity"
  if value == CURRENT:
    return "Current"
  raise Exception("Not defined")