import math
import numpy as np

class ControlSys:
    # Define Class Attributes
    
    def __init__(self, surfs):
        # surfs is a list containing each control surface object
        self.surfs = surfs
        self.setpoint = 3000
        self.cuma_error = 0
    
    def getForceMoment(self, t, u, finAngles):
        # Returns the vector [Fx Fy Fz Mx My Mz] in frame of Rocket

        # Linear drag estimation
        '''min_drag = 0.0
        max_drag = 1.0
        rho = 1.225
        finArea = 0.005

        drag_cof = 2*(max_drag-min_drag)*finAngles[0]/math.pi
        drag_force = 0.5*rho*u[5]*u[5]*drag_cof*finArea
        return [0, 0, -drag_force, 0, 0, 0]'''

        # Sum forces and moments for each control surface
        ForceMoments = np.zeros(6)
        for surf in self.surfs:
            ForceMoments = np.add(ForceMoments, surf.getForceMoment())

        # Do test where a constant torque is applied about the x axis after t = 4
        if t > 10:
            return [0, 0, -50, 0, 0, 0] 
        else:
            return [0, 0, 0, 0, 0, 0]
    
    def getAnglesSISOfromPID(self, t,u):
        # Calculates fin angles using PID controller for SISO
            # Input: self, position_z: altitude as scalar (m), velocity_z: vertical velocity (m/s)
            # Fin angles (radians)

        # Input parsing
        position_z = u[2]
        velocity_z = u[5]

        # Controller parameters
        proportial_gain = 0.0005
        integral_gain = 0
        derivative_gain = 0

        # Term calculation
        error = self.setpoint - position_z
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