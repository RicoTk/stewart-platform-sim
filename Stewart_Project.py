#This project is for a stewart platform only containing rotational motion of the platform with respect to the base


#install libraries in the VSCode Terminal

import numpy as np
import matplotlib as plt

#Angles related to each rotational motion
theta = np.deg2rad(-90)  # pitch
phi   = np.deg2rad(0)  # roll
psi   = np.deg2rad(0)  # yaw

# Base center pivot point
O = np.array([0, 0, 0]) 


#Radial Distance from Anchor points to origin of platform (in cm)
#RDP = Radial Distance Platform
RDP = 4

#Height at home position for anchor points
#HAP = Height of Anchor Points
HAP = 5

#Initial Translation Vector (at home position)
TransV = np.array([0, 0, HAP]) 

#Full Rotation Matrix for Platform relative to Base
def rotation_matrix(roll, pitch, yaw):
    # Rotation about Z (yaw), Y (pitch), X (roll)
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [0,            0,           1]])
    
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [0,              1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rx = np.array([[1, 0,             0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll),  np.cos(roll)]])
    
    return Rz @ Ry @ Rx

Rot_Matrix = rotation_matrix(phi, theta, psi)


# Initial Platform Anchor points (matrices of [x, y, z])
P_1 = np.array([RDP * np.cos(np.deg2rad(0)),   RDP * np.sin(np.deg2rad(0)),   HAP])
P_2 = np.array([RDP * np.cos(np.deg2rad(120)), RDP * np.sin(np.deg2rad(120)), HAP])
P_3 = np.array([RDP * np.cos(np.deg2rad(240)), RDP * np.sin(np.deg2rad(240)), HAP])

# Initial Base Anchor points (matrices of [x, y, z])

B_1 = np.array([RDP * np.cos(np.deg2rad(0)),   RDP * np.sin(np.deg2rad(0)),   0])
B_2 = np.array([RDP * np.cos(np.deg2rad(120)), RDP * np.sin(np.deg2rad(120)), 0])
B_3 = np.array([RDP * np.cos(np.deg2rad(240)), RDP * np.sin(np.deg2rad(240)), 0])

#Coordinates of Anchor Points with Respect to Base reference framework

#Point 1
Q_1 = O + Rot_Matrix @ P_1

#Point 2
Q_2 = O + Rot_Matrix @ P_2

#Point 3
Q_3 = O + Rot_Matrix @ P_3

Q_points = [Q_1, Q_2, Q_3]
B_points = [B_1, B_2, B_3]

#Computation of Actuator Lengths (For this project it will most likely represent links)
for i in range(3):
    L_vec = Q_points[i] - B_points[i]
    L_len = np.linalg.norm(L_vec)
    print(f"Actuator {i+1} length: {L_len:.3f} cm")

#Plot of Platform with Vectors