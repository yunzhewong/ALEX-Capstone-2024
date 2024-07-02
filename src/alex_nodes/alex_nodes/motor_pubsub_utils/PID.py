from alex_nodes.motor_pubsub_utils.constants import FREQUENCY

class PIDController():
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.target = None

        self.previousTime = 0
        self.previousValue = 0

        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def clear(self):
        self.target = None
        self.e = 0
        self.e_int = 0
        self.e_dot = 0

         
    def setTarget(self, target):
        self.target = target
        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def updateLatest(self, time, value):
        if self.target is None:
            return
        self.e = value - self.target 
        self.e_dot = (value - self.previousValue) / (FREQUENCY * (time - self.previousTime))
        self.e_int += self.e * (1 / FREQUENCY)

        self.previousTime = time
        self.previousValue = value

    def getControlValue(self):
        return self.K_p * self.e + self.K_d * self.e_dot + self.K_i * self.e_int
        
