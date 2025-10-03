import time
import numpy as np
import pandas as pd
from dataclasses import dataclass

import pybullet as p
import pybullet_data

Num = np.float64
QuinticCoefficients = np.ndarray
Vec3 = np.ndarray

E3 = np.ndarray([0, 0, 1], dtype=Num)


# ----------------- Quintic utility -----------------
def quintic_coeffs(
    p0: Num, v0: Num, a0: Num, pT: Num, vT: Num, aT: Num, T: Num
) -> np.ndarray:
    """
    Solve for 6 coefficients of a quintic p(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
    subject to boundary conditions at t=0 and t=T.
    """
    A = np.array(
        [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T**2, T**3, T**4, T**5],
            [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
            [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
        ],
        dtype=Num,
    )
    b = np.array([p0, v0, a0, pT, vT, aT], dtype=Num)
    return np.linalg.solve(A, b)


def eval_quintic(c: QuinticCoefficients, t: Num) -> tuple[Num, Num, Num, Num, Num]:
    """Return p, v, a, j, s at time t for quintic coefficients c (length 6)."""

    a0, a1, a2, a3, a4, a5 = c
    tt = t
    p = a0 + a1 * tt + a2 * tt**2 + a3 * tt**3 + a4 * tt**4 + a5 * tt**5
    v = a1 + 2 * a2 * tt + 3 * a3 * tt**2 + 4 * a4 * tt**3 + 5 * a5 * tt**4
    a = 2 * a2 + 6 * a3 * tt + 12 * a4 * tt**2 + 20 * a5 * tt**3
    j = 6 * a3 + 24 * a4 * tt + 60 * a5 * tt**2
    s = 24 * a4 + 120 * a5 * tt
    return p, v, a, j, s


# Analytic derivative for b1 = v/||v||
def normalize(v: Vec3, eps=1e-12) -> tuple[Vec3, Num]:
    """
    This gets the unit vector of velocity defined by our quintic, along with its
    norm aka magnitude.

    This info is used to orient the direction of travel and speed.
    """
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros_like(v), Num(0.0)
    return v / n, n


def b1_db1(v: Vec3, a: Vec3, eps=1e-12) -> tuple[Vec3, Vec3]:
    """
    b1 = v/||v|| -> Heading
    db1 = (I - b1 b1^T) * a / ||v||   (time derivative of normalized vector)
    """
    b1, nv = normalize(v, eps)
    if nv < eps:
        return np.zeros(3), np.zeros(3)
    db1 = (np.eye(3) - np.outer(b1, b1)) @ (a / nv)  # Calc 3 magic
    return b1, db1


def d2b1_analytic(v: Vec3, a: Vec3, j: Vec3, eps=1e-12) -> Vec3:
    b1, s = normalize(v, eps)
    if s < eps:
        return np.zeros(3)
    P = np.eye(3) - np.outer(b1, b1)
    Pa = P @ a
    term1 = (P @ j) / s
    term2 = 2.0 * (v @ a) / (s**3) * Pa
    term3 = (Pa @ Pa) / (s**2) * b1
    return term1 - term2 - term3


# Helper functions
def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)


def vee(M):
    return (
        np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]], float) / 2
    )


@dataclass
class Params:
    J: np.ndarray  # TODO: make type for Jacobian
    mass: Num
    gravity: Num


@dataclass
class Gains:
    kx: Num
    kv: Num
    kr: Num
    komega: Num  # This naming convenction is odd. Maybe use k_b1 and k_db1?


@dataclass
class State:
    p: Vec3
    v: Vec3
    R: np.ndarray  # TODO: make type for rotation matrix, maybe even swap to Quaternion
    omega: np.ndarray


@dataclass
class Desired:
    p: Vec3
    v: Vec3
    a: Vec3
    j: Vec3
    s: Vec3

    # Heading
    b1: Vec3
    db1: Vec3
    d2b1: Vec3


@dataclass
class Traj3D:
    # c_ variables are individual quintic polynomials in each cardinal direction.
    cx: QuinticCoefficients
    cy: QuinticCoefficients
    cz: QuinticCoefficients
    T: Num

    def eval(self, t: Num) -> tuple[Vec3, Vec3, Vec3, Vec3, Vec3]:
        """Return position, velocity, acceleration, jerk, snap (each 3,) at time t (clamped to [0,T])."""
        tt = np.clip(t, 0.0, self.T)
        px, vx, ax, jx, sx = eval_quintic(self.cx, tt)
        py, vy, ay, jy, sy = eval_quintic(self.cy, tt)
        pz, vz, az, jz, sz = eval_quintic(self.cz, tt)
        p = np.array([px, py, pz], dtype=Num)
        v = np.array([vx, vy, vz], dtype=Num)
        a = np.array([ax, ay, az], dtype=Num)
        j = np.array([jx, jy, jz], dtype=Num)
        s = np.array([sx, sy, sz], dtype=Num)
        return p, v, a, j, s


@dataclass
class ControlSignal:
    f: Num  # Total thrust
    M: np.ndarray  # TODO: Implement type for thrust mixer matrix


# NOTE: we build the traj from our current p, v, a and a target p, v, a, at time T.
# Using that info, we can then get a specific desired state at a time T in between.
def desired_from_traj(
    traj: Traj3D, t: Num, default_b1: Vec3 = np.ndarray([1, 0, 0], dtype=Num)
):
    p, v, a, j, s = traj.eval(t)

    # TODO: Refactor this to just a "headings" function
    b1, db1 = b1_db1(v, a)
    d2b1 = d2b1_analytic(v, a, j)

    if np.allclose(b1, 0):
        b1 = default_b1
        db1 = np.zeros(3, dtype=Num)
        d2b1 = np.zeros(3, dtype=Num)

    return Desired(p, v, a, j, s, b1, db1, d2b1)


class LeeController:
    def __init__(self, params: Params, gains: Gains) -> None:
        self.params = params
        self.gains = gains

    def __call__(self, state: State, desired: Desired) -> ControlSignal:
        ex, ev = state.p - desired.p, state.v - desired.v
        A = (
            (-self.gains.kx) * ex
            + (-self.gains.kv) * ev
            - self.params.mass * self.params.gravity * E3
            + self.params.mass * desired.a
        )

        b3 = state.R @ E3
        f = -A.dot(b3)
        nA = np.linalg.norm(A)

        if nA < 1e-12:
            Rc = state.R.copy()
            dRc = np.zeros((3, 3), dtype=Num)
            d2Rc = np.zeros((3, 3), dtype=Num)
        else:
            b3c = -A / nA  # "Desired body z-axis" -> b3 command
            C = np.cross(b3c, desired.b1)
            nC = np.linalg.norm(C)
            if nC < 1e-12:
                Rc = state.R.copy()
                dRc = np.zeros((3, 3), dtype=Num)
                d2Rc = np.zeros((3, 3), dtype=Num)

            else:
                b2c = C / nC
                b1c = -np.cross(b3c, b2c)

                Rc = np.column_stack((b1c, b2c, b3c))  # Rotational command

                # "Feed-forward" for desired rotational commands with jerk, snap, and heading derivatives
                dA = self.params.mass * desired.j
                d2A = self.params.mass * desired.s

                db3c = -dA / nA + (A.dot(dA) / (nA**3)) * A  # TODO: this line is sus
                A1n2 = dA @ dA  # wtf is this
                d2b3c = (
                    -d2A / nA
                    + (2.0 / (nA**3)) * A.dot(dA) * dA
                    + ((A1n2 + A.dot(d2A)) / (nA**3)) * A
                    - (3.0 / (nA**5)) * (A.dot(dA) ** 2) * A
                )

                dC = np.cross(db3c, desired.b1) + np.cross(b3c, desired.db1)
                d2C = (
                    np.cross(d2b3c, desired.b1)
                    + np.cross(b3c, desired.d2b1)
                    + 2 * np.cross(db3c, desired.db1)
                )

                # finally the control rotations
                db2c = dC / nC - (C.dot(dC) / (nC**3)) * C
                db1c = np.cross(db2c, b3c) + np.cross(b2c, db3c)

                term = (np.linalg.norm(dC) ** 2 + C.dot(d2C)) / (nC**3)

                d2b2c = (
                    d2C / nC
                    - 2.0 * (C.dot(dC)) / (nC**3) * dC
                    - term * C
                    + 3.0 * (C.dot(dC) ** 2) / (nC**5) * C
                )

                d2b1c = (
                    np.cross(d2b2c, b3c)
                    + np.cross(b2c, d2b2c)
                    + 2.0 * np.cross(db2c, db3c)
                )

                dRc = np.column_stack((db1c, db2c, db3c))
                d2Rc = np.column_stack((d2b1c, d2b2c, d2b3c))

        # desired body rates and accels
        omegac = vee(Rc.T @ dRc)
        domegac = vee(Rc.T @ d2Rc - hat(omegac) @ hat(omegac))

        # Errors and moments ??
        eR = 2.0 * vee(Rc.T @ state.R - state.R.T @ Rc)
        eomega = state.omega - state.R.T @ Rc @ omegac

        Jw = self.params.J @ state.omega
        feed_forward = self.params.J @ (
            state.R.T @ Rc @ domegac - hat(state.omega) @ (state.R.T @ Rc @ omegac)
        )

        M = (
            (-self.gains.kr) * eR
            + (-self.gains.komega) * eomega
            + np.cross(state.omega, Jw)
            + feed_forward
        )

        return ControlSignal(f, M)


if __name__ == "__main__":
    G = 9.81
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -G)
    p.loadURDF("plane.urdf")

    drone = p.loadURDF(
        "crazyflie.urdf.xacro", (0, 0, 0.8), p.getQuaternionFromEuler((0, 0, 0))
    )

        
    while True:
        p.stepSimulation()
        # time.sleep(1.0/240.0)