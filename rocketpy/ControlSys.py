class ControlSys:
    # Define Class Attributes
    
    def __init__(self, t0, u0):
        self.t = t0
        self.u = u0
    
    def getForceMoment(self, t, u):
        # Returns the vector [Rx Ry Rz Mx My Mz]
        # Do test where a constant torque is applied about the x axis after t = 4
        if t > 10:
            return [0, 0, -50, 0, 0, 0]
        else:
            return [0, 0, 0, 0, 0, 0]
    