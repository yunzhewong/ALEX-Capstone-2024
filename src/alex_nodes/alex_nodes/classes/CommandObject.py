from commandTypes import CommandType

class CommandObject():
    def __init__(self, commandNumber: int, value: float):
        if (commandNumber == CommandType.Position.value):
            self.command = CommandType.Position
        elif commandNumber == CommandType.Velocity.value:
            self.command = CommandType.Velocity
        elif commandNumber == CommandType.Current.value:
            self.command = CommandType.Current
        else:
            raise Exception("Command not expected")
        
        self.value = value

    def toString(self):
        commandString = self.command.convertToString()
        return f"{commandString} Control: {self.value}"
    
    def getCommandValue(self):
        return self.command.value