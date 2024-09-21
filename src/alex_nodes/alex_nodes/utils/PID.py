class PIDController():
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.reference = None

        self.previousTime = 0

        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def setReference(self, reference):
        self.reference = reference
        self.e = 0
        self.e_int = 0
        self.e_dot = 0

    def update(self, time, value):
        if self.reference is None:
            return

        dt = time - self.previousTime
        if dt == 0:
            return
        # E = R - Y
        newError = self.reference - value 

        self.e_dot = (newError - self.e) / dt
        self.e_int += newError * dt

        self.e = newError

        self.previousTime = time

    def getControlValue(self):
        return self.K_p * self.e + self.K_d * self.e_dot + self.K_i * self.e_int
        
