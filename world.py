import pybullet as p
import pybullet_data 


if __name__ == "__main__":
    G = 9.81
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -G)