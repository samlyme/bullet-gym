from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
import time
from tkinter.tix import Control
from typing import Type  # noqa: F401
import numpy as np
import pybullet as p
import pybullet_data


# ---------- Drone (plant) ----------
@dataclass
class DroneParams:
    mass: float = 0.5
    arm: float = 0.15
    KF: float = 6e-6  # thrust coeff (N/(rad/s)^2)
    KM: float = 2.45e-7  # reaction torque coeff (N·m/(rad/s)^2)
    dirs: tuple[int, int, int, int] = (+1, -1, +1, -1)  # CCW/CW signs

    @property
    def km_per_t(self) -> float:
        return self.KM / self.KF


@dataclass
class Drone:
    body: int
    rotors: tuple[int, int, int, int]  # joint indices
    hubs: tuple[tuple[float, float, float], ...]  # hub positions in base frame
    params: DroneParams

    def get_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.body)
        v_lin, w_world = p.getBaseVelocity(self.body)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        w_body = R.T @ np.asarray(w_world)
        return {
            "pos": np.array(pos),
            "vel": np.array(v_lin),
            "rpy": np.array([roll, pitch, yaw]),
            "R": R,
            "w_body": w_body,
        }

    def spin_visual(self, omegas):
        for j, w in zip(self.rotors, omegas):
            p.setJointMotorControl2(
                self.body,
                j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=float(w),
                force=1e3,
            )

    def apply_thrusts(self, thrusts):
        # apply per-rotor thrust at hub locations; add net yaw reaction torque
        total_yaw = 0.0
        for hub, d, T in zip(self.hubs, self.params.dirs, thrusts):
            p.applyExternalForce(self.body, -1, (0, 0, float(T)), hub, p.LINK_FRAME)
            total_yaw += d * self.params.km_per_t * float(T)
        p.applyExternalTorque(self.body, -1, (0, 0, total_yaw), p.LINK_FRAME)

class Controller(ABC):

    @abstractmethod
    def __init__(self, drone: Drone) -> None:
        pass

    @abstractmethod
    def update(self, state, target) -> tuple:
        pass

# ---------- Controller (strategy) ----------
class QuadPDController(Controller):
    def __init__(self, drone: Drone):
        self.d = drone
        # Position → desired accels
        self.Kp_xy, self.Kd_xy = 4, 3
        self.Kp_z, self.Kd_z = 4, 3
        # Attitude torques
        self.Kp_att, self.Kd_att = 0.4, 0.01 # cant set too low or pos control breaks
        
        # I have no clue what i am doing, so i will just disable yaw. 
        self.Kp_yaw, self.Kd_yaw = 0, 0

        # Precompute inverse mixer
        a = self.d.params.arm
        k = self.d.params.km_per_t
        d1, d2, d3, d4 = self.d.params.dirs
        M = np.array(
            [
                [1, 1, 1, 1],  # U1
                [0, a, 0, -a],  # Ux
                [-a, 0, a, 0],  # Uy
                [k * d1, k * d2, k * d3, k * d4],  # Uz
            ],
            dtype=float,
        )
        self.Minv = np.linalg.inv(M)

    def update(self, state, target):
        m = self.d.params.mass
        g = 9.81

        pos, vel = state["pos"], state["vel"]
        roll, pitch, yaw = state["rpy"]
        w_body = state["w_body"]

        # Outer loop: position → desired accelerations
        exy = target["pos"][:2] - pos[:2]
        evxy = -vel[:2]
        ax_des, ay_des = self.Kp_xy * exy + self.Kd_xy * evxy
        ez = target["pos"][2] - pos[2]
        ez_dot = -vel[2]
        az_des = self.Kp_z * ez + self.Kd_z * ez_dot

        # Total thrust (feedforward g)
        U1 = m * (g + az_des)

        # Small-angle map: horizontal accel → desired tilt
        roll_des = -ay_des / g
        pitch_des = ax_des / g
        yaw_des = target["yaw"]
        TILT_MAX = math.radians(20)
        roll_des = max(-TILT_MAX, min(roll_des, TILT_MAX))
        pitch_des = max(-TILT_MAX, min(pitch_des, TILT_MAX))

        # Attitude PD → body torques
        def wrap_pi(a):
            return (a + math.pi) % (2 * math.pi) - math.pi

        e_roll = roll_des - roll
        e_pitch = pitch_des - pitch
        e_yaw = wrap_pi(yaw_des - yaw)

        Ux = self.Kp_att * e_roll + self.Kd_att * (-w_body[0])
        Uy = self.Kp_att * e_pitch + self.Kd_att * (-w_body[1])
        Uz = self.Kp_yaw * e_yaw + self.Kd_yaw * (-w_body[2])

        # Mix → per-rotor thrusts
        U = np.array([U1, Ux, Uy, Uz], dtype=float)
        T = self.Minv @ U

        # Saturate and return also visual omegas
        T = np.clip(T, 0.0, m * g)  # simple per-rotor cap; tune as needed
        omegas = [math.sqrt(t / self.d.params.KF) if t > 0 else 0.0 for t in T]
        return T, omegas


# ---------- World (orchestrator) ----------
class World:
    def __init__(self, controller_type: Type[Controller], dt=1 / 240, connection_type: int = p.GUI):
        self.dt = dt
        p.connect(connection_type)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        # Build drone
        body = p.loadURDF(
            "quadrotor.urdf", (0, 0, 0.8), p.getQuaternionFromEuler((0, 0, 0))
        )
        # Discover rotor joints by name
        name_to_idx = {
            p.getJointInfo(body, j)[1].decode(): j for j in range(p.getNumJoints(body))
        }
        rotors = tuple(
            name_to_idx[n]
            for n in ("rotor1_joint", "rotor2_joint", "rotor3_joint", "rotor4_joint")
        )
        hubs = ((+0.15, 0, 0), (0, +0.15, 0), (-0.15, 0, 0), (0, -0.15, 0))
        params = DroneParams()
        self.drone = Drone(body, rotors, hubs, params)  # type: ignore
        self.controller = controller_type(self.drone)
        self.target = {"pos": (0, 0, 1), "yaw": 0}

    def set_target(self, target):
        self.target = {"pos": target[0], "yaw": target[1]}

    def step(self):
        s = self.drone.get_state()
        T, w = self.controller.update(s, self.target)
        # Visual spin *without* adding unwanted motor torques:
        p.setJointMotorControlArray(
            self.drone.body,
            self.drone.rotors,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=w,
            forces=[0] * 4,
        )
        self.drone.apply_thrusts(T)
        p.stepSimulation()
        # Debug
        # dist = np.linalg.norm(s["pos"] - self.target["pos"])
        # if dist > 1.0:   # meters; tune threshold
        #     print(f"[WARN] Drone drifted {dist:.2f} m from target {self.target['pos']}")
        #     print(f"[INFO] Controller output {T}")

    def reached(self) -> bool:
        s = self.drone.get_state()
        t = self.target
        if (
            np.linalg.norm(s["pos"] - np.asarray(t["pos"])) < 0.8
            and np.linalg.norm(s["vel"]) < 0.1
            # and abs(s["rpy"][2] - t['yaw']) < 0.005
        ):
            return True
        # print("current state", s)
        # print("current target", t)
        return False


# ---------- Example run ----------
if __name__ == "__main__":
    world = World(controller_type=QuadPDController, connection_type=p.GUI)
    n = 6
    pos_stationary = [(0, 0 ,1) for _ in range(n)]
    pos_flat_sqaure = [
        (0, 0, 1),
        (1, 0, 1),
        (1, 1, 1),
        (0, 1, 1),
        (1, 0, 1),
        (1, 1, 1),
    ]
    pos_vert_sqaure = [
        (0, 0, 1),
        (1, 0, 2),
        (1, 1, 1),
        (0, 1, 2),
        (1, 0, 1),
        (1, 1, 2),
    ]
    pos_vert = [(0, 0, 1 if i % 2 == 0 else 2) for i in range(n)]
    pos_far = [(0, 5 if i % 2 == 0 else -5, 1) for i in range(n)]
    yaw_stationary = [0 for _ in range(n)]
    yaw_small = [0 if i % 2 == 0 else 1 for i in range(n)]
    yaw_large = [0 if i % 2 == 0 else 2 for i in range(n)]
    waypoints = list(zip(pos_vert_sqaure, yaw_stationary))
        

    waypoint_idx = 0
    for k in range(240 * 240):
        # change target when waypoint is reached
        if world.reached():
            print("going to waypoint", waypoints[waypoint_idx % len(waypoints)])
            world.set_target(waypoints[waypoint_idx % len(waypoints)])
            waypoint_idx += 1

        world.step()

        # time.sleep(world.dt)
    p.disconnect()
