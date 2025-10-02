import pybullet as p
import time  # noqa: F401
import pybullet_data

# --- setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")
startPos = (0, 0, 1)
startOrientation = p.getQuaternionFromEuler((0, 0, 0))
robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# Wheel joints (from your printout)
RIGHT_WHEELS = [2, 3]
LEFT_WHEELS  = [6, 7]
ALL_WHEELS   = RIGHT_WHEELS + LEFT_WHEELS

# Optionally print to verify
for j in ALL_WHEELS:
    print(j, p.getJointInfo(robotId, j)[1])

# --- driving parameters ---
MAX_SPEED = 10.0  # rad/s
FORCE     = 100.0  # Nm (increase if wheels stall)
left_cmd, right_cmd = 0.0, 0.0

print("Controls: W=forward, S=back, A=left, D=right, SPACE=stop, Q=quit")

def set_wheel_speeds(left_speed: float, right_speed: float):
    # If you discover one side spins backward, flip its sign here.
    target_vels = []
    for j in RIGHT_WHEELS:
        target_vels.append(right_speed)
    for j in LEFT_WHEELS:
        target_vels.append(left_speed)
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=ALL_WHEELS,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=target_vels,
        forces=[FORCE]*len(ALL_WHEELS)
    )

# --- loop ---
while True:
    keys = p.getKeyboardEvents()

    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        left_cmd, right_cmd =  MAX_SPEED,  MAX_SPEED
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        left_cmd, right_cmd = -MAX_SPEED, -MAX_SPEED
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        left_cmd, right_cmd = -MAX_SPEED,  MAX_SPEED
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        left_cmd, right_cmd =  MAX_SPEED, -MAX_SPEED
    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_IS_DOWN:
        left_cmd, right_cmd = 0.0, 0.0
    if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        break

    set_wheel_speeds(left_cmd, right_cmd)

    p.stepSimulation()
    # Uncomment to slow down simulation.
    # time.sleep(1./240.)

p.disconnect()
