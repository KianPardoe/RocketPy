import math
import numpy as np
from scipy import interpolate

class ControlSys:

    """
    setpoint: float
    This is the target altitude for the rocket in metres
    
    finAngles: list of floats
    List of the 4 fin angles as measured from the vertical position, specified in Radians.
    Fin ordering in the list is at 0, pi/2, pi and 3/2 pi around the rocket

    A: float
    Reference area for calculation of drag and lift forces specified in m^2

    r: 1x3 np array 
    Representing distance from centre of mass to the centre of pressure of the fin
    aligned in the direction of the positive x axis of the rocket frame. Expressed in rocket frame
    """
    def __init__(self):
        self.setpoint = 100 # above ground level
        self.hold_error = 0
        self.cuma_error = 0
        self.finAngles = [0, 0, 0, 0]
        self.A =  0.00325 # Fin reference area for Cd Cl
        self.r = np.array([25/1000, 0, 3/1000])
        self.cut = 0
        self.apog = 0

        # Read in Cd and Cl data
        CdFile = open("data/proxima/proximaFinCD.csv", "r")
        ClFile = open("data/proxima/proximaFinCL.csv", "r")

        CdData = np.loadtxt(CdFile, delimiter = ",")
        ClData = np.loadtxt(ClFile, delimiter = ",")

        # Methods for interpolating Cd, Cl. Call self.getCd(0.2) to get Cd at 0.2 rad
        self.getCd = interpolate.interp1d(CdData[:,0]* math.pi/180, CdData[:,1])
        self.getCl = interpolate.interp1d(ClData[:,0]* math.pi/180, ClData[:,1])
        
    
    def getForceMoment(self, t, u, finAngles, rho, compStreamVxB, compStreamVyB, compStreamVzB):
        # Returns the vector [Fx Fy Fz Mx My Mz] in frame of Rocket

        # Set new Angles for the control surfaces
        self.finAngles = finAngles

        # Forces
        F = np.array([0, 0, 0])

        # Moments
        M = np.array([0, 0, 0])

        # Sum forces and moments for each control surface
        for i in range(len(self.finAngles)):
            Cd = self.getCd(abs(finAngles[i]))
            Cl = self.getCl(abs(finAngles[i]))
            
            # Drag and Lift Forces
            Fz = (compStreamVzB/abs(compStreamVzB)) * 0.5 * Cd * self.A * rho * (compStreamVzB ** 2)
            Fx = 0
            Fy = 0
            # Assuming Lift force is only generated for flow perpendicular to the axis of rotation of fin.
            # This means only finAngle[0], finAngle[2] (those aligned with x axis) generate lift for stream velocity in y direction
            if i % 2:
                Fy = (compStreamVyB/abs(compStreamVyB)) * 0.5 * Cl * self.A * rho * (compStreamVyB ** 2)
            else:
                Fx = (compStreamVxB/abs(compStreamVxB)) * 0.5 * Cl * self.A * rho * (compStreamVxB ** 2)

            F = F + np.array([Fx, Fy, Fz])

            theta = i * math.pi/2
            R = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])
            M = M + np.cross((R.dot(self.r)), np.array([Fx, Fy, Fz]))

        return np.concatenate((F, M), axis=None).tolist()
    

    def predictApogee(self,t,u,z_accel):

        # Input parsing
        p = u[2]
        v = u[5]
        g = -9.81
        a = z_accel + g
        # Calculate apogee
        apogee = (v*v*math.log(abs(a/g)))/(2*abs(a+g)) + p - 722
        # print('-------------------')
        # print('Velocity: ' + str(v))
        # print('Acceleration: ' +  str(a))
        # print('Apogee: ' +  str(apogee))
        # print('-------------------')
        return apogee
        

    def getAnglesSISOfromPID(self, t,u,z_accel):
        # Calculates fin angles using PID controller for SISO
            # Input: self, position_z: altitude as scalar (m), velocity_z: vertical velocity (m/s)
            # Fin angles (radians)

        # Input parsing
        position_z = u[2]
        velocity_z = u[5]

        # Cut Logic
        if(position_z - 722> self.apog):
            self.apog = position_z - 722

        if(position_z - 722 < self.apog - 100):
            self.cut = self.cut + 1

        if(self.cut>3):
            return [0,0,0,0]    
        
        # Controller parameters
        proportial_gain = 0.5
        integral_gain = 0#0.0000001
        derivative_gain = 0.1
        # Term calculation
        
        error = self.predictApogee(t,u,z_accel) - self.setpoint
        error_integral = self.cuma_error + error
        rate_error = error - self.hold_error
        
        # Update Error Terms for store
        self.cuma_error = error_integral
        self.hold_error = error

        # Output calculation using PID
        out = proportial_gain*error
        out -= integral_gain*error_integral
        out -= derivative_gain*rate_error

        # Limit Control
        if(out>math.pi/2.0):
            out = math.pi/2.0
        elif(out<0):
            out = 0
        return [-out,out,out,-out]

    def getAnglesLQR(self, position , velocity, orientation, angular_velocity):
        # Calculates fin angles using LQR
        out = [0,0,0,0]

        return out