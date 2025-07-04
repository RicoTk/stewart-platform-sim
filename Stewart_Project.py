#This project is for a stewart platform only containing rotational motion of the platform with respect to the base


#install libraries in the VSCode Terminal

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.animation as Animation


#----------------------------------------------------------------------------------------------------------------------------

#Initial Static Analysis of Poses and Actuator Dynamics

#Angles related to each rotational motion
theta = np.deg2rad(0)  # pitch
phi   = np.deg2rad(0)  # roll
psi   = np.deg2rad(0)  # yaw

# Base center pivot point
O = np.array([0, 0, 0]) 


#Radial Distance from Anchor points to origin of platform and base (in cm)
#RDP = Radial Distance Platform
RDP = 5

#RDB = Radial Distance Base
RDB = 10

#Height at home position for anchor points
#HAP = Height of Anchor Points
HAP = 10

#Initial Translation Vector (at home position)
TransV = np.array([0, 0, 0]) 

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

B_1 = np.array([RDB * np.cos(np.deg2rad(0)),   RDB * np.sin(np.deg2rad(0)),   0])
B_2 = np.array([RDB * np.cos(np.deg2rad(120)), RDB * np.sin(np.deg2rad(120)), 0])
B_3 = np.array([RDB * np.cos(np.deg2rad(240)), RDB * np.sin(np.deg2rad(240)), 0])

#Coordinates of Anchor Points with Respect to Base reference framework

#Point 1
Q_1 = TransV + O + Rot_Matrix @ P_1

#Point 2
Q_2 = TransV + O + Rot_Matrix @ P_2

#Point 3
Q_3 = TransV + O + Rot_Matrix @ P_3

P_points = [P_1, P_2, P_3]
Q_points = [Q_1, Q_2, Q_3]
B_points = [B_1, B_2, B_3]

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
plt.show()




#-------------------------------------------------------------------------------------------------------------------------

#Animation of Stewart Platform Poses and Actuator Dynamics

# Base center pivot point
O = np.array([0, 0, 0]) 


#Radial Distance from Anchor points to origin of platform and base (in cm)
#RDP = Radial Distance Platform
RDP = 5

#RDB = Radial Distance Base
RDB = 10

#Height at home position for anchor points
#HAP = Height of Anchor Points
HAP = 10

# Initial Platform Anchor points (matrices of [x, y, z])
P_1 = np.array([RDP * np.cos(np.deg2rad(0)),   RDP * np.sin(np.deg2rad(0)),   HAP])
P_2 = np.array([RDP * np.cos(np.deg2rad(120)), RDP * np.sin(np.deg2rad(120)), HAP])
P_3 = np.array([RDP * np.cos(np.deg2rad(240)), RDP * np.sin(np.deg2rad(240)), HAP])

# Initial Base Anchor points (matrices of [x, y, z])

B_1 = np.array([RDB * np.cos(np.deg2rad(0)),   RDB * np.sin(np.deg2rad(0)),   0])
B_2 = np.array([RDB * np.cos(np.deg2rad(120)), RDB * np.sin(np.deg2rad(120)), 0])
B_3 = np.array([RDB * np.cos(np.deg2rad(240)), RDB * np.sin(np.deg2rad(240)), 0])



fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot static base points
B_x, B_y, B_z = zip(*B_points)
ax.scatter(B_x, B_y, B_z, color='blue', label='Base Anchors')

# Prepare plot objects for platform points and actuator lines to update in animation
rotated_platform_scatter = ax.scatter([], [], [], color='red', label='Rotated Platform')
actuator_lines = [ax.plot([], [], [], 'r-')[0] for _ in range(3)]

# Set labels and view
ax.set_xlabel('X [cm]')
ax.set_ylabel('Y [cm]')
ax.set_zlabel('Z [cm]')
ax.set_title('Stewart Platform Animation')
ax.legend()
ax.grid(True)
ax.view_init(elev=25, azim=45)

# Limits to keep the plot stable
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_zlim(0, 15)

def rotation_matrix(roll, pitch, yaw):
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

def init():
    rotated_platform_scatter._offsets3d = ([], [], [])
    for line in actuator_lines:
        line.set_data([], [])
        line.set_3d_properties([])
    return actuator_lines + [rotated_platform_scatter]

#Definition of Analysis lists
Length_Actuator1 = [] 
Length_Actuator2 = []
Length_Actuator3 = []

def update(frame):
    # Pitch, Roll and Yaw variables (set to what it wanted)
    pitch = np.deg2rad(0 * np.sin(frame * 0.1))
    roll = np.deg2rad(0 * np.sin(frame * 0.1))
    yaw = np.deg2rad(0 * np.sin(frame * 0.1))
    Rot_Matrix = rotation_matrix(roll, pitch, yaw)

    #Translation Vector (set to what is wanted)
    TransV = [(0 * np.sin(frame * 0.1)), (0 * np.sin(frame * 0.1)), (0 * np.sin(frame * 0.1)) ]
    
    Q_points = [O + TransV + Rot_Matrix @ p for p in P_points]
    
    # Update rotated platform points scatter
    Q_x, Q_y, Q_z = zip(*Q_points)
    rotated_platform_scatter._offsets3d = (Q_x, Q_y, Q_z)
    
    # Update actuator lines
    for i, line in enumerate(actuator_lines):
        x_vals = [B_points[i][0], Q_points[i][0]]
        y_vals = [B_points[i][1], Q_points[i][1]]
        z_vals = [B_points[i][2], Q_points[i][2]]
        line.set_data(x_vals, y_vals)

        line.set_3d_properties(z_vals)

    # Calculating vector lengths and Logging values into lists
    for i in range(len(Q_points)):
        L_vec = Q_points[i] - B_points[i]
        L_len = np.linalg.norm(L_vec)
        Title = f"Actuator {i+1} length: {L_len:.3f} cm"
        if 'Actuator 1 length' in Title: 
            Length_Actuator1.append(Title[19:24])

        elif 'Actuator 2 length' in Title:
            Length_Actuator2.append(Title[19:24])

        elif 'Actuator 3 length' in Title:
            Length_Actuator3.append(Title[19:24])

        else:
            print('not valid value')
    
    return actuator_lines + [rotated_platform_scatter]

ani = Animation.FuncAnimation(fig, update, frames=200, init_func=init, blit=True, interval=50)
plt.show()


#--------------------------------------------------------------------------------------------------------------------------------

#Analysis of motion animation values 

#Transforming list of strings to floats
Length_Actuator1 = [float(Length) for Length in Length_Actuator1]
Length_Actuator2 = [float(Length) for Length in Length_Actuator2]
Length_Actuator3 = [float(Length) for Length in Length_Actuator3]

#Max length values
Max_Actuator1 = max(Length_Actuator1)
Max_Actuator2 = max(Length_Actuator2)
Max_Actuator3 = max(Length_Actuator3)

#Min length values
Min_Actuator1 = min(Length_Actuator1)
Min_Actuator2 = min(Length_Actuator2)
Min_Actuator3 = min(Length_Actuator3)

#Deltas
Delta_Actuator1 = Max_Actuator1 - Min_Actuator1
Delta_Actuator2 = Max_Actuator2 - Min_Actuator2
Delta_Actuator3 = Max_Actuator3 - Min_Actuator3
