import aiosv2

from dataGathering import gather_data

MAGNITUDE = 1
DURATION = 5

if __name__ == "__main__":
    def command_func(connection: aiosv2.SafeMotor, runningTime: float):
        connection.setCurrent(MAGNITUDE)    

    gather_data(command_func, DURATION, f"step{MAGNITUDE}A.csv")

    
