import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
#np.set_printoptions(precision=5, suppress=True) #so we can see better coment out at run
time_step=0.5
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
    Computes individual classic DH matrices and cumulative FK.
    dh_table: list of dicts with keys a, alpha, d, theta
    """
    T_total = np.eye(4)

    for i, row in enumerate(dh_table):
        T_i = dh_matrix(row["a"], row["alpha"], row["d"], row["theta"])
        if i <= limit:
            T_total = T_total @ T_i
            #print(f"H{i} gives the position of {i+1} coordinate system")
            #print(f"--- Final FK (Base to Link {i+1}) ---")
            #print(T_total)
            #print("\n\n")

    return T_total



def wrist_angles_from_R(R, eps=1e-8):
    """
    Extract theta4, theta5, theta6 from a 4x4 homogeneous matrix H
    for the wrist rotation parametrization used earlier.

    Assumes the rotation submatrix R corresponds to:
        R = [[..., ..., r02],
             [..., ..., r12],
             [r20, r21, r22]]

    Returns (theta4, theta5, theta6) in radians.

    The function auto-corrects if the 3x3 block appears transposed.
    """

    # Clear variable names (zero-based indices):
    # r02 = R[0,2], r12 = R[1,2], r22 = R[2,2]
    # r20 = R[2,0], r21 = R[2,1]
    r02 = R[0, 2]
    r12 = R[1, 2]
    r22 = R[2, 2]

    r20 = R[2, 0]
    r21 = R[2, 1]

    # theta5: use atan2(sin_theta, cos_theta) with sin_theta = sqrt(r02^2 + r12^2)
    sin_theta5 = np.sqrt(max(0.0, r02*r02 + r12*r12))
    theta5 = np.arctan2(sin_theta5, r22)   # in [0, pi]

    # Handle general vs singular cases
    if sin_theta5 > eps:
        # general case: theta5 not near 0 or pi
        theta4 = np.arctan2(-r12, -r02)   # atan2( -R[1,2], -R[0,2] ) -> gives theta4
        theta6 = np.arctan2(-r21, r20)    # atan2( -R[2,1], R[2,0] ) -> gives theta6
    else:
        # singular: theta5 ~ 0 or pi -> r02 and r12 are ~0
        # There is infinite (phi,psi) pairs; choose a convention:
        # set theta4 = atan2(R[1,0], R[0,0]) and theta6 = 0
        theta4 = np.arctan2(R[1, 0], R[0, 0])
        theta6 = 0.0

    # Optional: wrap angles to (-pi, pi]
    def wrap(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    return wrap(theta4), theta5, wrap(theta6)






# --- 2. Trapezoidal Velocity Profile (LSPB) Function ---

def trapezoidal_profile(time_points, total_distance, max_velocity, acceleration):
    """
    Generates the s(t) ratio for a Linear Segment with Parabolic Blend (LSPB).
    
    Returns: s_t (array of path ratios 0 to 1)
    """
    D = total_distance
    v_max = max_velocity
    a_max = acceleration
    
    # 1. Calculate critical times
    t_a = v_max / a_max  # Acceleration time
    t_f = D / v_max + t_a # Total time
    if 2 * t_a > t_f:
        print("Error: Max velocity too high or D too short. Using triangular profile.")
        t_a = t_f / 2.0 # Fallback to triangular profile
        
    s_t = np.zeros_like(time_points)
    
    for i, t in enumerate(time_points):
        if t <= t_a:
            # Acceleration phase (Parabolic)
            s = (a_max / 2.0) * t**2 / D
        elif t <= (t_f - t_a):
            # Constant velocity phase (Linear)
            s = (v_max * t - (v_max**2) / (2.0 * a_max)) / D
        elif t <= t_f:
            # Deceleration phase (Parabolic)
            s = 1.0 - (a_max / 2.0) * (t_f - t)**2 / D
        else:
            # End of motion
            s = 1.0
        s_t[i] = s
        
    return s_t, t_f

def forward_kinematics(angles,limit=100):
    dh = [
    {"theta": 0, "a": 0, "alpha": 0,        "d": 0.19952},#link 0==>coordinate 0 after this it becomes coordinate 1
    {"theta": 0+angles[0], "a": -0.040, "alpha": np.pi/2,  "d": 0.13048},#link 1
    {"theta": -np.pi/2+angles[1], "a": -0.445,   "alpha": np.pi,    "d": 0},#link 2
    {"theta": 0+angles[2], "a": -0.040,   "alpha": -np.pi/2, "d": 0},#link 3
    {"theta": 0+angles[3], "a": 0,        "alpha": np.pi/2,        "d": 0.440},#link 4
    {"theta": 0+angles[4], "a": 0,        "alpha": -np.pi/ 2, "d": 0},#link 5
    {"theta": 0+angles[5], "a": 0,        "alpha": 0, "d": 0.214},#link 6 EE Shows at angles23 fn
    ]      #>6 to get all
    T_fk = forward_kinematics_dh(dh,limit)
    return T_fk

# --- 3. Main Trajectory Planning and Execution ---
def main(Q_START=np.array([0, 0, 0, 0.0, 0.0, 0.0]),Q_END=np.array([0,0.005119,-0.2218,0,0.2269190,0])):
    # 0. Define System Parameters
    MAX_V_LINEAR = 0.1  # m/s (Maximum linear speed of the end-effector)
    MAX_A_LINEAR = 0.05  # m/s^2 (Acceleration used for the profile)
    TIME_STEP = time_step    # seconds (Control loop frequency: 100 Hz)

    # 1. Define Start and End Angles (must be feasible)

    # 2. Get Start/End Cartesian Poses and the FIXED Orientation
    T_start = forward_kinematics(Q_START)
    T_end = forward_kinematics(Q_END) # Calculate this to get the End position

    P_start = T_start[:3, 3]
    P_end = T_end[:3, 3]

    # *** CRUCIAL STEP: LOCK THE ORIENTATION ***
    R_fixed = T_start[:3, :3]
    # The orientation of the box remains this R_fixed matrix throughout the move.
    
    # 3. Calculate Path Distance and Time Profile
    total_distance = np.linalg.norm(P_end - P_start)
    t_a = MAX_V_LINEAR / MAX_A_LINEAR  # Acceleration time
    t_f = total_distance / MAX_V_LINEAR + t_a
    time_points = np.arange(0, t_f, TIME_STEP) # Start with long array, will trim later
    # Generate the smooth path ratio s(t)
    s_t, t_final = trapezoidal_profile(time_points, total_distance, MAX_V_LINEAR, MAX_A_LINEAR)
    # s_t = s_t[time_points <= t_final] # Trim to actual duration
    time_array = time_points
    # 4. Generate the Trajectory (The IK Loop)
    # Initialize storage
    joint_trajectory = np.zeros((len(time_array), 6))
    P_trajectory = np.zeros((len(time_array)+1, 3))
    current_angles = Q_START.copy()

    print(f"Total Trajectory Time: {t_final:.2f} s")
    print("-" * 30)

    for i, s in enumerate(s_t):
        # A. Linear Position Interpolation
        P_target = P_start + s * (P_end - P_start)
        P_trajectory[i] = P_target
        
        # B. Construct Target Pose (T_target) with FIXED Orientation
        T_target = np.identity(4)
        T_target[:3, :3] = R_fixed # Orientation is LOCKED to the start orientation
        T_target[:3, 3] = P_target
        
        # C. Run Inverse Kinematics (using Kinematic Decoupling logic inside IK)
        # The current_angles are passed to help the IK choose the correct elbow/wrist configuration.
        Q_target = inverse_kinematics_trajectory(T_target,current_angles[3],current_angles[5])
        #Q_target = inverse_kinematics(T_target)
        # D. Store and Update
        joint_trajectory[i] = Q_target
        current_angles = Q_target
        if i % 100 == 0:
            print(f"Time: {time_array[i]:.2f}s | s: {s:.2f} | Pos: {P_target}")
    #joint_trajectory[len(time_array)]=fix_trajectory_start(Q_END,joint_trajectory[-2])
    print(joint_trajectory)
    return joint_trajectory


    # --- 5. Visualization (Optional) ---
    # Plotting the joint angles shows the smooth, non-linear movement required
    # to compensate for the straight Cartesian path and fixed orientation.
    # plt.figure(figsize=(10, 6))
    # plt.title("Joint Trajectories with Constant Orientation Planning")
    # plt.plot(time_array, joint_trajectory)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Joint Angle (radians)")
    # plt.grid(True)
    # plt.legend([f'Joint {i+1}' for i in range(6)])
    # plt.tight_layout()
    # plt.show()

    # # You can also plot the s(t) profile to verify the trapezoid
    # plt.figure(figsize=(10, 4))
    # plt.plot(time_array, s_t, label='s(t) - Path Ratio')
    # plt.title("Trapezoidal Velocity Profile (s vs t)")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Path Ratio (s)")
    # plt.grid(True)
    # plt.show()

def angles2and3(T_final):
    #get the 0Oc location
    O0c=T_final[:3, 3]-(0.214)*T_final[:3,2]
    t2, t3 = sp.symbols('t2 t3', real=True)
    eq1 = 0.04 + 0.445*sp.sin(t2) + 0.04*sp.sin(t2 - t3) + 0.44*sp.cos(t2 - t3) - (float(np.sqrt(O0c[0]**2+O0c[1]**2)))
    eq2 = 0.33 + 0.445*sp.cos(t2) + 0.04*sp.cos(t2 - t3) - (0.44*sp.sin(t2 - t3) + O0c[2])
    sol = sp.nsolve([eq1, eq2], [t2, t3], [0.5, 0.5])   # initial guess
    #print(sol)
    return sol

def inverse_kinematics(T_target):
    R_fixed =T_target[0:3, 0:3] # Orientation is LOCKED to the start orientation
    P_target=T_target[:3, 3]
    angles23=angles2and3(T_target)
    Q_target=np.array([np.arctan2(-P_target[1],-P_target[0]),float(angles23[0]),float(angles23[1]),0,0,0])
    H3=forward_kinematics(Q_target,3)
    R3=H3[0:3, 0:3]
    R36=R3.T@R_fixed
    Q_target[3], Q_target[4], Q_target[5]=wrist_angles_from_R(R36)
    H6=forward_kinematics(Q_target)#so it will be printed
    return Q_target

def inverse_kinematics_trajectory(T_target,theta_pre4,theta_prev6):
    R_fixed =T_target[0:3, 0:3] # Orientation is LOCKED to the start orientation
    P_target=T_target[:3, 3]
    angles23=angles2and3(T_target)
    Q_target=np.array([np.arctan2(-P_target[1],-P_target[0]),float(angles23[0]),float(angles23[1]),0,0,0])
    H3=forward_kinematics(Q_target,3)
    R3=H3[0:3, 0:3]
    R36=R3.T@R_fixed
    Q_target[3], Q_target[4], Q_target[5]=wrist_angles_from_R_trajectory(R36,theta_pre4,theta_prev6)
    H6=forward_kinematics(Q_target)#so it will be printed
    return Q_target

def check_imports(): # Used to check whether the import works
    print("Imported Correctly")
    return 0

if __name__=='__main__':
    main()

def wrap_continuous(a_current, a_previous):
    # This must be robust and generic for any joint
    k = np.round((a_previous - a_current) / (2 * np.pi))
    return a_current + 2 * np.pi * k


def wrist_angles_from_R_trajectory(R, theta_prev_4, theta_prev_6, eps=1e-8):
    """
    Extract theta4, theta5, theta6 from the required wrist rotation matrix R (R_3^6).
    
    This version relies on continuous wrapping to stabilize the singular region,
    rather than introducing a separate 'else' branch that often causes new discontinuities.
    """
    r02 = R[0, 2]
    r12 = R[1, 2]
    r22 = R[2, 2]
    r20 = R[2, 0]
    r21 = R[2, 1]

    # --- 1. Solve for theta5 ---
    sin_theta5 = np.sqrt(max(0.0, r02*r02 + r12*r12))
    theta5 = np.arctan2(sin_theta5, r22) 

   # 2. General case formulas
    theta4_raw = np.arctan2(-r12, -r02)
    theta6_raw = np.arctan2(-r21, r20)
    
    # --- 3. Apply Continuous Wrapping to both joints ---
    
    # This is the line that fixes the jump in Joint 4
    theta4 = wrap_continuous(theta4_raw, theta_prev_4) 
    
    # This is the line that fixes the jump in Joint 6
    theta6 = wrap_continuous(theta6_raw, theta_prev_6)

    return theta4, theta5, theta6

def fix_trajectory_start(Q_start_new, Q_end_previous):
    """
    Ensures the starting angles of the new trajectory segment are continuous 
    with the ending angles of the previous segment.
    """
    # Create a copy to modify
    Q_fixed = Q_start_new.copy()

    # Apply continuity fix to Joint 4 (theta4) - Column index 3
    Q_fixed[3] = wrap_continuous(Q_start_new[3], Q_end_previous[3])
    
    # Apply continuity fix to Joint 6 (theta6) - Column index 5
    Q_fixed[5] = wrap_continuous(Q_start_new[5], Q_end_previous[5])
    
    return Q_fixed