import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

time_step = 0.01  # Control loop step

# --------------------- DH Transformation ---------------------
def dh_matrix(a, alpha, d, theta):
    """
    Classic (standard) DH transformation matrix.
    Angles in radians.
    """
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def forward_kinematics_dh(dh_table, limit):
    """
    Computes cumulative forward kinematics using DH parameters.
    dh_table: list of dicts with keys a, alpha, d, theta
    limit: up to which row to multiply
    """
    T_total = np.eye(4)
    for i, row in enumerate(dh_table):
        if i <= limit:
            T_total = T_total @ dh_matrix(row["a"], row["alpha"], row["d"], row["theta"])
    return T_total

# --------------------- Wrist Angles Extraction ---------------------
def wrist_angles_from_R(R, eps=1e-8):
    """
    Extract theta4, theta5, theta6 from a 3x3 rotation matrix.
    Handles singularities at theta5 ~ 0 or pi.
    """
    r02, r12, r22 = R[0, 2], R[1, 2], R[2, 2]
    r20, r21 = R[2, 0], R[2, 1]

    sin_theta5 = np.sqrt(max(0.0, r02**2 + r12**2))
    theta5 = np.arctan2(sin_theta5, r22)

    if sin_theta5 > eps:
        theta4 = np.arctan2(-r12, -r02)
        theta6 = np.arctan2(-r21, r20)
    else:
        theta4 = np.arctan2(R[1, 0], R[0, 0])
        theta6 = 0.0

    return wrap_continuous(theta4, 0), theta5, wrap_continuous(theta6, 0)

def wrist_angles_from_R_trajectory(R, theta_prev_4, theta_prev_6, eps=1e-8):
    """
    Extract wrist angles with continuous wrapping for trajectory planning.
    Prevents jumps in joint values during motion.
    """
    r02, r12, r22 = R[0, 2], R[1, 2], R[2, 2]
    r20, r21 = R[2, 0], R[2, 1]

    sin_theta5 = np.sqrt(max(0.0, r02**2 + r12**2))
    theta5 = np.arctan2(sin_theta5, r22)

    theta4_raw = np.arctan2(-r12, -r02)
    theta6_raw = np.arctan2(-r21, r20)

    theta4 = wrap_continuous(theta4_raw, theta_prev_4)
    theta6 = wrap_continuous(theta6_raw, theta_prev_6)

    return theta4, theta5, theta6

# --------------------- Forward Kinematics ---------------------
def forward_kinematics(angles, limit=100):
    """
    Compute full forward kinematics given joint angles.
    """
    dh = [
        {"theta": 0, "a": 0, "alpha": 0, "d": 0.19952},
        {"theta": angles[0], "a": -0.040, "alpha": np.pi/2, "d": 0.13048},
        {"theta": -np.pi/2 + angles[1], "a": -0.445, "alpha": np.pi, "d": 0},
        {"theta": angles[2], "a": -0.040, "alpha": -np.pi/2, "d": 0},
        {"theta": angles[3], "a": 0, "alpha": np.pi/2, "d": 0.440},
        {"theta": angles[4], "a": 0, "alpha": -np.pi/2, "d": 0},
        {"theta": angles[5], "a": 0, "alpha": 0, "d": 0.214},
    ]
    return forward_kinematics_dh(dh, limit)

# --------------------- Trajectory Planning ---------------------
def trapezoidal_profile(time_points, total_distance, max_velocity, acceleration):
    """
    Linear Segment with Parabolic Blend (LSPB) profile.
    Returns s(t) array and total time.
    """
    D, v_max, a_max = total_distance, max_velocity, acceleration
    t_a = v_max / a_max
    t_f = D / v_max + t_a

    if 2 * t_a > t_f:
        t_a = t_f / 2  # Triangular profile fallback

    s_t = np.zeros_like(time_points)
    for i, t in enumerate(time_points):
        if t <= t_a:
            s = (a_max/2) * t**2 / D
        elif t <= t_f - t_a:
            s = (v_max * t - (v_max**2)/(2*a_max)) / D
        elif t <= t_f:
            s = 1 - (a_max/2)*(t_f - t)**2 / D
        else:
            s = 1.0
        s_t[i] = s
    return s_t, t_f

# --------------------- Inverse Kinematics ---------------------
def angles2and3(T_final):
    """
    Solve for joints 2 and 3 using symbolic solution.
    """
    O0c = T_final[:3, 3] - 0.214 * T_final[:3, 2]
    t2, t3 = sp.symbols('t2 t3', real=True)
    eq1 = 0.04 + 0.445*sp.sin(t2) + 0.04*sp.sin(t2-t3) + 0.44*sp.cos(t2-t3) - float(np.sqrt(O0c[0]**2 + O0c[1]**2))
    eq2 = 0.33 + 0.445*sp.cos(t2) + 0.04*sp.cos(t2-t3) - (0.44*sp.sin(t2-t3) + O0c[2])
    sol = sp.nsolve([eq1, eq2], [t2, t3], [0.5, 0.5])
    return sol

def inverse_kinematics(T_target):
    """
    Standard IK for a pose with fixed orientation.
    """
    R_fixed = T_target[:3, :3]
    P_target = T_target[:3, 3]
    angles23 = angles2and3(T_target)

    Q_target = np.array([
        np.arctan2(-P_target[1], -P_target[0]),
        float(angles23[0]),
        float(angles23[1]),
        0, 0, 0
    ])
    R3 = forward_kinematics(Q_target, 3)[:3, :3]
    R36 = R3.T @ R_fixed
    Q_target[3], Q_target[4], Q_target[5] = wrist_angles_from_R(R36)
    return Q_target

def inverse_kinematics_trajectory(T_target, theta_pre4, theta_prev6):
    """
    IK version for trajectory planning with continuous wrist angle wrapping.
    """
    R_fixed = T_target[:3, :3]
    P_target = T_target[:3, 3]
    angles23 = angles2and3(T_target)

    Q_target = np.array([
        np.arctan2(-P_target[1], -P_target[0]),
        float(angles23[0]),
        float(angles23[1]),
        0, 0, 0
    ])
    R3 = forward_kinematics(Q_target, 3)[:3, :3]
    R36 = R3.T @ R_fixed
    Q_target[3], Q_target[4], Q_target[5] = wrist_angles_from_R_trajectory(R36, theta_pre4, theta_prev6)
    return Q_target

# --------------------- Trajectory Utilities ---------------------
def wrap_continuous(a_current, a_previous):
    """
    Wrap angle to be continuous relative to previous angle.
    """
    k = np.round((a_previous - a_current) / (2*np.pi))
    return a_current + 2*np.pi*k

def fix_trajectory_start(Q_start_new, Q_end_previous):
    """
    Ensure continuity of new trajectory segment with previous.
    """
    Q_fixed = Q_start_new.copy()
    Q_fixed[3] = wrap_continuous(Q_start_new[3], Q_end_previous[3])
    Q_fixed[5] = wrap_continuous(Q_start_new[5], Q_end_previous[5])
    return Q_fixed

# --------------------- Main Trajectory Function ---------------------
def main(Q_START=np.array([0, 0, 0, 0, 0, 0]), 
         Q_END=np.array([0, 0.005119, -0.2218, 0, 0.226919, 0]),need_plots=False):
    """
    Main function to generate joint trajectories with fixed orientation.
    """
    MAX_V_LINEAR = 0.1
    MAX_A_LINEAR = 0.05
    TIME_STEP = time_step

    T_start = forward_kinematics(Q_START)
    T_end = forward_kinematics(Q_END)

    P_start, P_end = T_start[:3, 3], T_end[:3, 3]
    R_fixed = T_start[:3, :3]

    total_distance = np.linalg.norm(P_end - P_start)
    time_points = np.arange(0, total_distance/MAX_V_LINEAR + MAX_V_LINEAR/MAX_A_LINEAR, TIME_STEP)
    s_t, t_final = trapezoidal_profile(time_points, total_distance, MAX_V_LINEAR, MAX_A_LINEAR)

    joint_trajectory = np.zeros((len(time_points), 6))
    current_angles = Q_START.copy()

    for i, s in enumerate(s_t):
        P_target = P_start + s * (P_end - P_start)
        T_target = np.eye(4)
        T_target[:3, :3] = R_fixed
        T_target[:3, 3] = P_target
        Q_target = inverse_kinematics_trajectory(T_target, current_angles[3], current_angles[5])
        joint_trajectory[i] = Q_target
        current_angles = Q_target

        if i % 100 == 0:
            print(f"Time: {time_points[i]:.2f}s | s: {s:.2f} | Pos: {P_target}")

    print(joint_trajectory)
    if need_plots:
        plots(joint_trajectory,time_points,s_t)
    return joint_trajectory

def plots(joint_trajectory,time_points,s_t):
    # --- 5. Visualization (Optional) ---

    # Plot joint trajectories
    plt.figure(figsize=(10, 6))
    plt.title("Joint Trajectories with Constant Orientation Planning")
    plt.plot(time_points, joint_trajectory)
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (radians)")
    plt.grid(True)
    plt.legend([f'Joint {i+1}' for i in range(6)])
    plt.tight_layout()
    plt.show()

    # Plot the s(t) trapezoidal velocity profile
    plt.figure(figsize=(10, 4))
    plt.plot(time_points, s_t, label='s(t) - Path Ratio')
    plt.title("Trapezoidal Velocity Profile (s vs t)")
    plt.xlabel("Time (s)")
    plt.ylabel("Path Ratio (s)")
    plt.grid(True)
    plt.show()

# --------------------- Check Imports ---------------------
def check_imports():
    print("Imported Correctly")
    return 0

# --------------------- Run Main ---------------------
if __name__ == '__main__':
    main()
