import commandTypes as CommandType

class CommandObject():
    def __init__(self, command: int, value: float):
        self.command = command
        self.value = value

    def toString(self):
        commandString = CommandType.convertToString(self.command)

        return f"{commandString} Control: {self.value}"