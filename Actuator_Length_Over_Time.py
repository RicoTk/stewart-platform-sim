import numpy as np
import matplotlib.pyplot as plt

# --- Platform Parameters ---
RDP = 4     # Radial distance from center to joints (cm)
HAP = 5     # Height from base (cm)
O = np.array([0, 0, 0])  # Base center

# --- Base and Platform Anchor Points ---
angles_deg = [0, 120, 240]
B_points = [np.array([RDP * np.cos(np.deg2rad(a)), RDP * np.sin(np.deg2rad(a)), 0]) for a in angles_deg]
P_points = [np.array([RDP * np.cos(np.deg2rad(a)), RDP * np.sin(np.deg2rad(a)), HAP]) for a in angles_deg]

# --- Rotation Matrix Function (ZYX) ---
def rotation_matrix(roll, pitch, yaw):
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0,              1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    return Rz @ Ry @ Rx

# --- Generate Sine Wave for Pitch (±40° = ±0.698 rad) ---
duration = 1          # seconds
sample_rate = 1000    # samples/sec
t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
motion_wave = np.deg2rad(40) * np.sin(2 * np.pi * 1 * t)  # 1 Hz motion oscillation

# --- Store Actuator Lengths ---
actuator_lengths = [[] for _ in range(3)]

# --- Simulate Motion ---
for motion in motion_wave:
    R = rotation_matrix(motion, motion, motion)
    Q_points = [O + R @ P for P in P_points]
    for i in range(3):
        L_vec = Q_points[i] - B_points[i]
        L_len = np.linalg.norm(L_vec)
        actuator_lengths[i].append(L_len)

# --- Plot Actuator Lengths Over Time ---
plt.figure(figsize=(10, 5))
for i, L in enumerate(actuator_lengths):
    plt.plot(t, L, label=f'Actuator {i+1}')
plt.title("Actuator Lengths During 1 Hz Yaw Motion (±40°)")
plt.xlabel("Time [s]")
plt.ylabel("Length [cm]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# --- Find Max and Min Lengths for Each Actuator ---
for i, L in enumerate(actuator_lengths):
    print(f"Actuator {i+1}: min = {min(L):.2f} cm, max = {max(L):.2f} cm, Δ = {max(L) - min(L):.2f} cm")
