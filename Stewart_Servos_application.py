import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.animation as Animation

#constants of platform

# Base center pivot point
O = np.array([0, 0, 0]) 

#Radial Distance from Anchor points to origin of platform and base 
#RDP = Radial Distance Platform (in cm)
RDP = 4

#RDB = Radial Distance Base Points (in cm)
RDB = 7


#Height at home position for anchor points
#HAP = Height of Anchor Points (in cm)
HAP = 5

#Initial Translation Vector (at home position)
TransV = np.array([0, 0, HAP]) 

#legth of short servo arm (in cm)
S_Arm = 1.5 

#length of long servo arm (in cm)
L_Arm = np.sqrt((5**2) + (S_Arm**2))

# Home Platform Anchor points (matrices of [x, y, z])
P_1 = np.array([RDP * np.cos(np.deg2rad(330)),   RDP * np.sin(np.deg2rad(330)),   HAP])
P_2 = np.array([RDP * np.cos(np.deg2rad(30)),    RDP * np.sin(np.deg2rad(30)),    HAP])
P_3 = np.array([RDP * np.cos(np.deg2rad(90)),    RDP * np.sin(np.deg2rad(90)),    HAP])
P_4 = np.array([RDP * np.cos(np.deg2rad(150)),   RDP * np.sin(np.deg2rad(150)),   HAP])
P_5 = np.array([RDP * np.cos(np.deg2rad(210)),   RDP * np.sin(np.deg2rad(210)),   HAP])
P_6 = np.array([RDP * np.cos(np.deg2rad(270)),   RDP * np.sin(np.deg2rad(240)),   HAP])

# Base Anchor points (matrices of [x, y, z])
B_1 = np.array([RDB* np.cos(np.deg2rad(330)), RDB * np.sin(np.deg2rad(330)), 0])
B_2 = np.array([RDB* np.cos(np.deg2rad(30)),  RDB * np.sin(np.deg2rad(30)),  0])
B_3 = np.array([RDB* np.cos(np.deg2rad(90)),  RDB * np.sin(np.deg2rad(90)),  0])
B_4 = np.array([RDB* np.cos(np.deg2rad(150)), RDB * np.sin(np.deg2rad(150)), 0])
B_5 = np.array([RDB* np.cos(np.deg2rad(210)), RDB * np.sin(np.deg2rad(210)), 0])
B_6 = np.array([RDB* np.cos(np.deg2rad(270)), RDB * np.sin(np.deg2rad(240)), 0])


#-------------------------------------------------------------------------------------------------
# Home Servo Joint points (matrices of [x, y, z])
A_1 = np.array([RDB* np.cos(np.deg2rad(330)), RDB * np.sin(np.deg2rad(330)), 0])
A_2 = np.array([RDB* np.cos(np.deg2rad(30)),  RDB * np.sin(np.deg2rad(30)),  0])
A_3 = np.array([RDB* np.cos(np.deg2rad(90)),  RDB * np.sin(np.deg2rad(90)),  0])
A_4 = np.array([RDB* np.cos(np.deg2rad(150)), RDB * np.sin(np.deg2rad(150)), 0])
A_5 = np.array([RDB* np.cos(np.deg2rad(210)), RDB * np.sin(np.deg2rad(210)), 0])
A_6 = np.array([RDB* np.cos(np.deg2rad(270)), RDB * np.sin(np.deg2rad(240)), 0])

A_points = [A_1, A_2, A_3, A_4, A_5, A_6]

#Calculating M and N constants to get phase shift 
N_1 = 2 * np.linalg.norm(A_1) * ( (np.cos(330) *(P_1[0] - B_1[0]))  +  (np.sin(330) * (P_1[1] - B_1[1])))   
print(N_1)

M_1 = 2 * np.linalg.norm(A_1) * HAP
print(M_1)

#Phase Shift
pS_1 = np.atan(N_1/M_1)
print(pS_1)

Sin_funct = np.sin(np.linalg.norm(A_1) + pS_1)
print(Sin_funct)

Sqrt_funct

'''
#Angles related to each rotational motion
theta = np.deg2rad(30)  # pitch
phi   = np.deg2rad(30)  # roll
psi   = np.deg2rad(30)  # yaw


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


#lists of Anchor points (base B and platform P) and transformed platform points (Q_points)
P_points = [P_1, P_2, P_3, P_4, P_5, P_6]
B_points = [B_1, B_2, B_3]
Q_points = [] 

#Coordinates of Anchor Points with Respect to Base reference framework
for i in range(6):
    Q_points[i] = TransV + Rot_Matrix @ P_points[i]

#Computation of Actuator Lengths (For this project it will most likely represent links)
for i in range(3):
    L_vec = Q_points[i] - B_points[i]
    L_len = np.linalg.norm(L_vec)
    print(f"Actuator {i+1} length: {L_len:.3f} cm")

#Plot of Platform with Vectors

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot base anchor points
B_x, B_y, B_z = zip(*B_points)
ax.scatter(B_x, B_y, B_z, color='blue', label='Base Anchors')

# Plot original platform points (before rotation)
P_x, P_y, P_z = zip(*P_points)
ax.scatter(P_x, P_y, P_z, color='green', label='Original Platform (Unrotated)', alpha=0.5)

# Plot rotated platform points
Q_x, Q_y, Q_z = zip(*Q_points)
ax.scatter(Q_x, Q_y, Q_z, color='red', label='Rotated Platform')

# Plot actuator lines
for i in range(3):
    ax.plot([B_points[i][0], Q_points[i][0]],
            [B_points[i][1], Q_points[i][1]],
            [B_points[i][2], Q_points[i][2]],
            'r-')
    ax.plot([B_points[i][0], P_points[i][0]],
            [B_points[i][1], P_points[i][1]],
            [B_points[i][2], P_points[i][2]],
            'g--o')

# Set labels and view
ax.set_xlabel('X [cm]')
ax.set_ylabel('Y [cm]')
ax.set_zlabel('Z [cm]')
ax.set_title('Stewart Platform')
ax.legend()
ax.grid(True)
ax.view_init(elev=25, azim=45)
plt.tight_layout()
plt.show()'''