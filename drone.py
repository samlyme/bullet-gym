# quad_point_control.py
import math
import time  # noqa: F401
import numpy as np
import pybullet as p
import pybullet_data

# -------- Model & physics (tune to your URDF) --------
G = 9.81
MASS = 0.5
ARM  = 0.15
KF   = 6e-6      # thrust coeff: T = KF * w^2
KM   = 2.45e-7   # yaw reaction coeff per w^2
KM_PER_T = KM / KF

# Rotor layout (+1=CCW, -1=CW)
ROTORS = [
    {"name": "rotor1_joint", "pos": ( +ARM,  0.0, 0.0), "dir": +1},  # +x
    {"name": "rotor2_joint", "pos": ( 0.0,  +ARM, 0.0), "dir": -1},  # +y
    {"name": "rotor3_joint", "pos": ( -ARM,  0.0, 0.0), "dir": +1},  # -x
    {"name": "rotor4_joint", "pos": ( 0.0,  -ARM, 0.0), "dir": -1},  # -y
]

# -------- Control gains (start conservative) --------
# Position → desired accelerations
Kp_xy, Kd_xy = 2.5, 1.8
Kp_z,  Kd_z  = 12.0, 7.0
# Attitude (track desired roll/pitch/yaw)
Kp_att, Kd_att = 8.0, 0.9
Kp_yaw, Kd_yaw = 4.0, 0.5

# -------- Helpers --------
def quat_to_euler(q): return p.getEulerFromQuaternion(q)

def world_to_body(vec_w, R33):
    R = np.array(R33).reshape(3,3)
    return R.T @ np.asarray(vec_w)

def clamp(x, lo, hi): return max(lo, min(hi, x))

def wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi

def invert_mixer(arm, km_per_t, dirs):
    d1, d2, d3, d4 = dirs
    M = np.array([
        [1.0,  1.0,  1.0,  1.0],                  # sum T = U1
        [0.0,  arm,  0.0, -arm],                  # tau_x = Ux
        [-arm, 0.0,  arm,  0.0],                  # tau_y = Uy
        [km_per_t*d1, km_per_t*d2, km_per_t*d3, km_per_t*d4],  # tau_z = Uz
    ])
    return np.linalg.inv(M)

def load_world():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -G)
    p.loadURDF("plane.urdf")
    drone = p.loadURDF("quadrotor.urdf", (0,0,0.8), p.getQuaternionFromEuler((0,0,0)))
    return drone

def joint_indices_by_name(body, names):
    name_to_idx = {}
    for j in range(p.getNumJoints(body)):
        nm = p.getJointInfo(body, j)[1].decode("utf-8")
        name_to_idx[nm] = j
    return [name_to_idx[n] for n in names]

def spin_joints(body, joint_indices, omegas):
    for j_idx, w in zip(joint_indices, omegas):
        p.setJointMotorControl2(bodyUniqueId=body, jointIndex=j_idx,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=w, force=1e3)

def apply_rotor_forces(body, rotors, thrusts):
    total_yaw = 0.0
    for rotor, T in zip(rotors, thrusts):
        p.applyExternalForce(body, -1, (0,0,float(T)), rotor["pos"], p.LINK_FRAME)
        total_yaw += rotor["dir"] * KM_PER_T * float(T)
    p.applyExternalTorque(body, -1, (0,0,float(total_yaw)), p.LINK_FRAME)

# -------- Target API --------
TARGET = {"pos": np.array([0.0, 0.0, 1.2]), "yaw": 0.0}  # default
def set_target(pos_xyz, yaw=0.0):
    TARGET["pos"] = np.array(pos_xyz, dtype=float)
    TARGET["yaw"] = float(yaw)

# -------- Main --------
if __name__ == "__main__":
    drone = load_world()
    JOINT_IDX = joint_indices_by_name(drone, [r["name"] for r in ROTORS])
    DIRS = [r["dir"] for r in ROTORS]
    Minv = invert_mixer(ARM, KM_PER_T, DIRS)

    dt = 1.0/240.0
    prev_e = {"roll":0.0,"pitch":0.0,"yaw":0.0,"z":0.0}

    # EXAMPLE: change the target during flight
    # (you can remove these and just call set_target(...) interactively)
    waypoints = [
        ((0.0, 0.0, 1.2), 0.0),
        ((0.5, 0.0, 1.2), 0.0),
        ((0.5, 0.5, 1.4), 0.0),
        ((0.0, 0.5, 1.0), math.radians(45)),
        ((0.0, 0.0, 1.2), 0.0),
    ]
    wp_idx, wp_ticks = 0, 0
    set_target(*waypoints[0])

    for step in range(120*240):  # ~120 s
        # periodically switch targets (every ~6 s)
        if step % (6*240) == 0 and step > 0:
            wp_idx = (wp_idx + 1) % len(waypoints)
            set_target(*waypoints[wp_idx])

        pos, orn = p.getBasePositionAndOrientation(drone)
        if step == 0:
            print(type(pos), type(orn))
        v_lin, w_world = p.getBaseVelocity(drone)
        roll, pitch, yaw = quat_to_euler(orn)
        R = p.getMatrixFromQuaternion(orn)
        w_body = world_to_body(w_world, R)

        # -------- Outer loop: position → desired accelerations --------
        exy = TARGET["pos"][:2] - np.array(pos[:2])
        evxy = -np.array(v_lin[:2])
        ax_des, ay_des = (Kp_xy*exy + Kd_xy*evxy)
        ez = TARGET["pos"][2] - pos[2]
        ez_dot = -v_lin[2]
        az_des = Kp_z*ez + Kd_z*ez_dot

        # Total thrust (feedforward g)
        U1 = MASS*(G + az_des)

        # Map desired horizontal accels to desired tilt (small-angle approx):
        # ax ≈ g * (-pitch), ay ≈ g * (roll)
        roll_des  =  ay_des / G
        pitch_des = -ax_des / G
        yaw_des   = TARGET["yaw"]

        # -------- Attitude PD to get body torques --------
        e_roll  = roll_des  - roll
        e_pitch = pitch_des - pitch
        e_yaw   = wrap_pi(yaw_des - yaw)

        Ux = Kp_att*e_roll  + Kd_att*(-w_body[0])  # roll torque
        Uy = Kp_att*e_pitch + Kd_att*(-w_body[1])  # pitch torque
        Uz = Kp_yaw *e_yaw  + Kd_yaw *(-w_body[2]) # yaw torque

        # -------- Mix to per-rotor thrusts --------
        U = np.array([U1, Ux, Uy, Uz], dtype=float)
        T = Minv @ U

        # Saturate thrusts
        Tmax = MASS*G  # per-rotor limit guess; adjust if needed
        T = np.clip(T, 0.0, Tmax)

        # Spin joints visually consistent with thrust
        omegas = [math.sqrt(t / KF) if t > 0.0 else 0.0 for t in T]
        spin_joints(drone, JOINT_IDX, omegas)

        # Apply forces/torques at hubs
        apply_rotor_forces(drone, ROTORS, T)

        p.stepSimulation()
        # time.sleep(dt)

    p.disconnect()
