import math
import vtkplotlib as vpl
from stl.mesh import Mesh
import copy
import numpy as np
import os
import glob
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import vtkplotlib.image_io
import matplotlib.pylab as plt
import vtk

class RocketVisual:

    """
    frameRate: int, float
        Specifies the number of frames per secod in the visualisation
    visTime: float
        Specifies the simulation time that the last frame was taken
    rocketMesh: numpy-stl Mesh object
        Object containing the rocket model stl mesh
    finMesh: numpy-stl Mesh object
        Object containing the fin/control surface model stl mesh
    controlSys: ControlSys object
        object containing each control surface with the relevant angles
    """
    def __init__(
        self,
        controlSys,
        frameRate = 20,
        imageDim = 1000
    ):

        self.controlSys = controlSys
        self.frameRate = frameRate
        self.imageDim = imageDim
        self.visTime = 0
        self.frameNum = 0
        self.ims = []
        
        # Read in the rocket stl and control surface stl
        self.rocketMesh = Mesh.from_file("data/visualisation/3D_models/Rocket.stl")
        self.surfMesh = Mesh.from_file("data/visualisation/3D_models/Fin.stl")

        """
        # Delete any jpeg images in the working images directory
        files = glob.glob('data/visualisation/working_images/.jpeg*')
        for f in files:
            os.remove(f) """

    
    # Return the np.array for the rotation from frame B to frame A by angle phi
    def R_BA(self, phi):
        return np.array([[1, 0, 0], [0, math.cos(phi), -1*math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])

    # Return the np.array for the rotation from frame A to frame R by angle theta
    def R_AR(self, theta):
        return np.array([[math.cos(theta), -1*math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

    def rotateMesh(self, myMesh, R):
        for i in range(len(myMesh.points)):
            myMesh.v0[i] = np.array(R).dot(myMesh.v0[i])
            myMesh.v1[i] = np.array(R).dot(myMesh.v1[i])
            myMesh.v2[i] = np.array(R).dot(myMesh.v2[i])
        return myMesh

    def translateMesh(self, myMesh, r):
        for i in range(len(myMesh.points)):
            myMesh.v0[i] = myMesh.v0[i] + r
            myMesh.v1[i] = myMesh.v1[i] + r
            myMesh.v2[i] = myMesh.v2[i] + r
        return myMesh

    def makeRocketVisFrame(self, t, R_RG):
        """
        Saves an image of the rocket in current state if the time t calls for it given 
        the frame rate of the visualisation
        t: float
            Current model time
        R_RG: 3x3 numpy array
            Rotation matrix used to rotate vector from Rocket frame to Global frame (v_G = R_RG * v_R)
            """
        if t > self.visTime + (1.0/self.frameRate):
            self.visTime = t # Update visTime to the current time
            self.frameNum += 1 # Increase frame count

            # Create a copy of the mesh for each component of the rocket
            rocket = copy.deepcopy(self.rocketMesh)
            surfs = []
            for i in range(len(self.controlSys.surfs)):
                surfs.append(copy.deepcopy(self.surfMesh))
            
            # Rotate rocket body from Rocket frame to global frame.
            self.rotateMesh(rocket, R_RG)

            # Setup vtkplot figure
            fig = vpl.figure()
            
            # Plot rocket
            vpl.mesh_plot(rocket)

            # Change camera view of rocket
            vpl.view(focal_point=(0,0,0), camera_position=(0, 0, 0), camera_direction=(0,1500,-1500))
            
            # Rotate and translate control surfaces from control surface frame to Global frame.
            for i in range(len(surfs)):
                R_AR = self.R_AR(self.controlSys.surfs[i].Theta)
                R_BA = self.R_BA(self.controlSys.surfs[i].Phi)

                # Rotate from control surface frame to rocket frame
                self.rotateMesh(surfs[i], R_AR.dot(R_BA))

                # Translate control surface to correct position on rocket
                # multiply by 1000 to get r from m to mm
                r = 1000*np.array([0.5*self.controlSys.surfs[i].D, 0, self.controlSys.surfs[i].h])
                self.translateMesh(surfs[i], R_AR.dot(r))

                # Rotate control surface from Rocket frame to Global frame
                self.rotateMesh(surfs[i], R_RG)

                # Plot the control surface
                vpl.mesh_plot(surfs[i], color="dark blue")
            
            
            im = vpl.screenshot_fig(magnification=1, pixels=self.imageDim, trim_pad_width=None, off_screen=True)
            self.ims.append(im)
            vpl.close()
            

    def createVideo(self):
        
        height, width, layers = self.ims[0].shape
        size = (width,height)
        fps = 15
        out = cv2.VideoWriter('Flight.mp4',cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        for i in range(len(self.ims)):
            out.write(self.ims[i])
        out.release()


            


        