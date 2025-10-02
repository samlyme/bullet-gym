# typings/pybullet/__init__.pyi
from typing import Any, Dict, Iterable, Sequence, Tuple  # noqa: F401

# Connection modes
GUI: int
DIRECT: int

# Control modes
VELOCITY_CONTROL: int
POSITION_CONTROL: int
TORQUE_CONTROL: int

# Keyboard
KEY_IS_DOWN: int
B3G_SPACE: int

LINK_FRAME: int

Vec3 = tuple[float, float, float]
Vec4 = tuple[float, float, float]
Vec9 = tuple[float, float, float,
             float, float, float,
             float, float, float]

Euler = Vec3
Quaternion = Vec4
RotationMatrix = Vec9

def connect(mode: int) -> int: ...
def disconnect() -> None: ...
def setAdditionalSearchPath(path: str) -> None: ...
def setGravity(x: float, y: float, z: float) -> None: ...

def loadURDF(
    fileName: str,
    basePosition: Vec3 | None = ...,
    baseOrientation: Quaternion | None = ...,
    useFixedBase: bool | int | None = ...,
    flags: int | None = ...,
) -> int: ...

def getQuaternionFromEuler(euler: Euler) -> Quaternion: ...

def resetBasePositionAndOrientation(
    bodyUniqueId: int,
    pos: Vec3,
    orn: Vec4,
) -> None: ...

def getNumJoints(bodyUniqueId: int) -> int: ...
def getJointInfo(bodyUniqueId: int, jointIndex: int) -> Tuple[Any, ...]: ...

def setJointMotorControl2(
    bodyUniqueId: int,
    jointIndex: int,
    controlMode: int,
    targetPosition: float | None = ...,
    targetVelocity: float | None = ...,
    force: float | None = ...,
    positionGain: float | None = ...,
    velocityGain: float | None = ...,
) -> None: ...

def setJointMotorControlArray(
    bodyUniqueId: int,
    jointIndices: Sequence[int],
    controlMode: int,
    targetPositions: Sequence[float] | None = ...,
    targetVelocities: Sequence[float] | None = ...,
    forces: Sequence[float] | None = ...,
    positionGains: Sequence[float] | None = ...,
    velocityGains: Sequence[float] | None = ...,
) -> None: ...

def stepSimulation() -> None: ...

def getBasePositionAndOrientation(
    bodyUniqueId: int,
) -> Tuple[Vec3, Quaternion]: ...

def getKeyboardEvents() -> Dict[int, int]: ...


def applyExternalForce(
    objectUniqueId: int, 
    linkIndex: int, 
    forceObj: Vec3,
    posObj: Vec3,
    flags: int,
    physicsClientId: int | None = None
) -> None: ...

def getDynamicsInfo(bodyUniqueId: int, linkIndex: int, physicsClientId: int | None = ...) -> Any: ...

def getBaseVelocity(bodyUniqueId: int, physicsClientId: int | None = ...) -> tuple[Vec3, Vec3]: ...

def getEulerFromQuaternion(quaternion: Vec4, physicsClientId: int | None = ...) -> Euler: ...

def applyExternalTorque(objectUniqueId: int, linkIndex: int, forceObj: Vec3, flags: int): ...

def getMatrixFromQuaternion(quaternion: Quaternion) -> RotationMatrix:...