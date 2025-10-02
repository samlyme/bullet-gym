# typings/pybullet/__init__.pyi
from typing import Any, Dict, Iterable, Sequence, Tuple

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

def connect(mode: int) -> int: ...
def disconnect() -> None: ...
def setAdditionalSearchPath(path: str) -> None: ...
def setGravity(x: float, y: float, z: float) -> None: ...

def loadURDF(
    fileName: str,
    basePosition: Sequence[float] | None = ...,
    baseOrientation: Sequence[float] | None = ...,
    useFixedBase: bool | int | None = ...,
    flags: int | None = ...,
) -> int: ...

def getQuaternionFromEuler(euler: Sequence[float]) -> Tuple[float, float, float, float]: ...

def resetBasePositionAndOrientation(
    bodyUniqueId: int,
    pos: Sequence[float],
    orn: Sequence[float],
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
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]: ...

def getKeyboardEvents() -> Dict[int, int]: ...


def applyExternalForce(
    objectUniqueId: int, 
    linkIndex: int, 
    forceObj: list[float],
    posObj: list[float],
    flags: int,
    physicsClientId: int | None = None
) -> None: ...

def getDynamicsInfo(bodyUniqueId: int, linkIndex: int, physicsClientId: int | None = None) -> Any: ...
