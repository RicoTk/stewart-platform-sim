import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.animation as Animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

#constants of platform


#Angles related to each rotational motion
theta = np.deg2rad(40)  # pitch
phi   = np.deg2rad(20) # roll
psi   = np.deg2rad(0)  # yaw

# Base center pivot point
O = np.array([0, 0, 0]) 

#Radial Distance from Anchor points to origin of platform and base 
#RDP = Radial Distance Platform (in cm)
RDP = 4

#RDB = Radial Distance Base Points (in cm)
RDB = 6

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
# Platform Anchor points (aligned with base angles for symmetry)
P_1 = np.array([RDP * np.cos(np.deg2rad(0)),    RDP * np.sin(np.deg2rad(0)), HAP])
P_2 = np.array([RDP * np.cos(np.deg2rad(120)),  RDP * np.sin(np.deg2rad(120)),  HAP])
P_3 = np.array([RDP * np.cos(np.deg2rad(240)),  RDP * np.sin(np.deg2rad(240)),  HAP])
#P_4 = np.array([RDP * np.cos(np.deg2rad(150)), RDP * np.sin(np.deg2rad(150)), HAP])
#P_5 = np.array([RDP * np.cos(np.deg2rad(210)), RDP * np.sin(np.deg2rad(210)), HAP])
#P_6 = np.array([RDP * np.cos(np.deg2rad(270)), RDP * np.sin(np.deg2rad(270)), HAP])



# Base Anchor points (matrices of [x, y, z])
B_1 = np.array([RDB* np.cos(np.deg2rad(330)), RDB * np.sin(np.deg2rad(330)), 0])
B_2 = np.array([RDB* np.cos(np.deg2rad(30)),  RDB * np.sin(np.deg2rad(30)),  0])
B_3 = np.array([RDB* np.cos(np.deg2rad(90)),  RDB * np.sin(np.deg2rad(90)),  0])
B_4 = np.array([RDB* np.cos(np.deg2rad(150)), RDB * np.sin(np.deg2rad(150)), 0])
B_5 = np.array([RDB* np.cos(np.deg2rad(210)), RDB * np.sin(np.deg2rad(210)), 0])
B_6 = np.array([RDB* np.cos(np.deg2rad(270)), RDB * np.sin(np.deg2rad(240)), 0])

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

#Coordinates of Anchor Points with Respect to Base reference framework

#Point 1
Q_1 = TransV + O + Rot_Matrix @ P_1

#Point 2
Q_2 = TransV + O + Rot_Matrix @ P_2

#Point 3
Q_3 = TransV + O + Rot_Matrix @ P_3

#Point 1
#Q_4 = TransV + O + Rot_Matrix @ P_4

#Point 2
#Q_5 = TransV + O + Rot_Matrix @ P_5

#Point 3
#Q_6 = TransV + O + Rot_Matrix @ P_6

P_points = [P_1, P_2, P_3]     #P_4, P_5, P_6]
Q_points = [Q_1, Q_2, Q_3]    #Q_4, Q_5, Q_6]
B_points = [B_1, B_2, B_3, B_4, B_5, B_6]

#Computation of Actuator Lengths (For this project it will most likely represent links)
for j in range (3):
    for i in range(6):
        L_vec = Q_points[j] - B_points[i]
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
# Example mapping of base anchors to platform anchors
# Assume B1-B2 go to P1, B3-B4 to P2, B5-B6 to P3
connections = [(0, 0), (1, 0),  # B1, B2 → P1
               (2, 1), (3, 1),  # B3, B4 → P2
               (4, 2), (5, 2)]  # B5, B6 → P3

for b_idx, p_idx in connections:
    ax.plot([B_points[b_idx][0], Q_points[p_idx][0]],
            [B_points[b_idx][1], Q_points[p_idx][1]],
            [B_points[b_idx][2], Q_points[p_idx][2]],
            'r-')


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

#---------------------------------------------------------------------------------------------------------------------------------

# Constants
RDP = 5   # Radial Distance Platform [cm]
RDB = 10   # Radial Distance Base [cm]
HAP = 10   # Home Anchor Point Height [cm]
O = np.array([0, 0, 0])

# Define base and platform anchor points
B_points = [np.array([RDB * np.cos(np.deg2rad(angle)), RDB * np.sin(np.deg2rad(angle)), 0]) for angle in [330, 30, 90, 150, 210, 270]]
P_points = [np.array([RDP * np.cos(np.deg2rad(angle)), RDP * np.sin(np.deg2rad(angle)), HAP]) for angle in [0, 120, 240]]

# Define rotation matrix function
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

# Logging lists for each actuator
lengths_A1, lengths_A2, lengths_A3 = [], [], []
lengths_A4, lengths_A5, lengths_A6 = [], [], []

# Set up plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 15)
ax.set_xlabel("X [cm]")
ax.set_ylabel("Y [cm]")
ax.set_zlabel("Z [cm]")
ax.set_title("Stewart Platform with 6 Base Anchors and Platform Surface")
ax.grid(True)
ax.view_init(elev=30, azim=45)

# Plot base points
B_x, B_y, B_z = zip(*B_points)
ax.scatter(B_x, B_y, B_z, color='blue', label='Base Anchors')

# Initialize plot objects
platform_scatter = ax.scatter([], [], [], color='red', label='Platform Anchors')
platform_surface = Poly3DCollection([], alpha=0.4, facecolor='red')
ax.add_collection3d(platform_surface)
actuator_lines = [ax.plot([], [], [], 'k-')[0] for _ in range(6)]

# Animation init
def init():
    platform_scatter._offsets3d = ([], [], [])
    for line in actuator_lines:
        line.set_data([], [])
        line.set_3d_properties([])
    platform_surface.set_verts([])
    return actuator_lines + [platform_scatter, platform_surface]

# Animation update function
def update(frame):
    pitch = np.deg2rad(30 * np.sin(frame * 0.1))
    roll = np.deg2rad(0)
    yaw = np.deg2rad(0)
    trans = np.array([2 * np.sin(frame * 0.1), 0, 0])
    
    R = rotation_matrix(roll, pitch, yaw)
    Q_points = [O + trans + R @ p for p in P_points]
    
    # Update scatter
    Q_x, Q_y, Q_z = zip(*Q_points)
    platform_scatter._offsets3d = (Q_x, Q_y, Q_z)
    
    # Update surface
    platform_surface.set_verts([[Q_points[0], Q_points[1], Q_points[2]]])
    
    # Actuator connections: (Base idx, Platform idx)
    connections = [(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)]
    length_lists = [lengths_A1, lengths_A2, lengths_A3, lengths_A4, lengths_A5, lengths_A6]

    for i, (b_idx, p_idx) in enumerate(connections):
        b = B_points[b_idx]
        q = Q_points[p_idx]
        actuator_lines[i].set_data([b[0], q[0]], [b[1], q[1]])
        actuator_lines[i].set_3d_properties([b[2], q[2]])
        
        length = np.linalg.norm(q - b)
        length_lists[i].append(length)

    return actuator_lines + [platform_scatter, platform_surface]

ani = FuncAnimation(fig, update, frames=200, init_func=init, blit=False, interval=50)
plt.legend()
plt.tight_layout()
plt.show()

#--------------------------------------------------------------------------------------------------------------------------------

#Analysis of motion animation values 

#Max length values
Max_Actuator1 = max(lengths_A1) #from B0 to P0
Max_Actuator2 = max(lengths_A2) #from B1 to P0
Max_Actuator3 = max(lengths_A3) #from B2 to P1
Max_Actuator4 = max(lengths_A4) #from B3 to P1
Max_Actuator5 = max(lengths_A5) #from B4 to P2
Max_Actuator6 = max(lengths_A6) #from B5 to P2

#Min length values
Min_Actuator1 = min(lengths_A1) #from B0 to P0
Min_Actuator2 = min(lengths_A2) #from B1 to P0
Min_Actuator3 = min(lengths_A3) #from B2 to P1
Min_Actuator4 = min(lengths_A4) #from B3 to P1
Min_Actuator5 = min(lengths_A5) #from B4 to P2
Min_Actuator6 = min(lengths_A6) #from B5 to P2

#Deltas
Delta_Actuator1 = Max_Actuator1 - Min_Actuator1
Delta_Actuator2 = Max_Actuator2 - Min_Actuator2
Delta_Actuator3 = Max_Actuator3 - Min_Actuator3
Delta_Actuator4 = Max_Actuator4 - Min_Actuator4
Delta_Actuator5 = Max_Actuator5 - Min_Actuator5
Delta_Actuator6 = Max_Actuator6 - Min_Actuator6


#print(lengths_A1[:5])  # First 5 length values for actuator 1
#print(lengths_A4[-1])  # Last value for actuator 4
print(Max_Actuator1)
print(Min_Actuator1)


#---------------------------------------------------------------------------------------------------------------------------------
# Home Servo Joint points (matrices of [x, y, z])
A_1 = np.array([RDB* np.cos(np.deg2rad(330)), RDB * np.sin(np.deg2rad(330)), 0])
A_2 = np.array([RDB* np.cos(np.deg2rad(30)),  RDB * np.sin(np.deg2rad(30)),  0])
A_3 = np.array([RDB* np.cos(np.deg2rad(90)),  RDB * np.sin(np.deg2rad(90)),  0])
A_4 = np.array([RDB* np.cos(np.deg2rad(150)), RDB * np.sin(np.deg2rad(150)), 0])
A_5 = np.array([RDB* np.cos(np.deg2rad(210)), RDB * np.sin(np.deg2rad(210)), 0])
A_6 = np.array([RDB* np.cos(np.deg2rad(270)), RDB * np.sin(np.deg2rad(240)), 0])

A_points = [A_1, A_2, A_3, A_4, A_5, A_6]


#Calculations of Servo arm lengths based on changed parameters

#Calculating M and N constants to get phase shift 
N_1 = 2 * 1.5 * ( (np.cos(330) *(P_1[0] - B_1[0]))  +  (np.sin(330) * (P_1[1] - B_1[1])))   
print(N_1)

M_1 = 2 * 1.5 * HAP
print(M_1)

#Phase Shift
pS_1 = np.atan(N_1/M_1)
print(pS_1)

#Desired Distance between B_1 and P_1 at home position
l_0 = 11.180

#angle between lever arms in home position
alpha_0 = 20

#Calculating bigger lever arm based on a, M, N, and alpha_0 
s_0 = np.sqrt(-1 * (np.sin(np.deg2rad(alpha_0) + pS_1) * np.sqrt(M_1**2 + N_1 **2)) + (l_0 **2 ) + (2.5**2))
print(s_0)



