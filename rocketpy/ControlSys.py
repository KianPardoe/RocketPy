import math

class ControlSys:
    # Define Class Attributes
    
    def __init__(self, t0, u0):
        self.t = t0
        self.u = u0
        setpoint = 3134
        cuma_error = 0
    
    def getForceMoment(self, t, u):
        # Returns the vector [Rx Ry Rz Mx My Mz]
        # Do test where a constant torque is applied about the x axis after t = 4
        if t > 10:
            return [0, 0, -50, 0, 0, 0]
        else:
            return [0, 0, 0, 0, 0, 0]
    
    def getAnglesSISOfromPID(self, position_x , velocity_z):
        # Calculates fin angles using PID controller for SISO
            # Input: self, position_x: altitude as scalar (m), velocity_z: vertical velocity (m/s)
            # Fin angles (radians)

        # Controller parameters
        proportial_gain = 1
        integral_gain = 0
        derivative_gain = 0

        # Term calculation
        error = self.setpoint - position_x
        error_integral = self.cuma_error + error
        rate_error = velocity_z
        
        # Cumalative error calculation
        self.cuma_error = error_integral

        # Output calculation using PID
        out = proportial_gain*error
        out += integral_gain*error_integral
        out += derivative_gain*rate_error

        # Limit Control
        if(out>math.pi/2):
            out = math.pi/2
        elif(out<0):
            out = 0

        return [out,out,out,out]

    def getAnglesLQR(self, position , velocity, orientation, angular_velocity):
        # Calculates fin angles using LQR
        out = [0,0,0,0]

        return out