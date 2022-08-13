import numpy as np
from scipy import interpolate
import math

CdFile = open("data/proxima/proximaFinCD.csv", "r")
CdData = np.loadtxt(CdFile, delimiter = ",")

getCd = interpolate.interp1d(CdData[:,0]* math.pi/180, CdData[:,1])
theta = math.pi
r = np.array([1, 2, 3])
d = np.array([4, 5, 6])

print(np.concatenate((r, d), axis=None))
