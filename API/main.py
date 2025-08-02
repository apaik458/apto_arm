import numpy as np
import time

from apto_client import *

class AptoNode:
    def __init__(self):
        # Parameters
        self.kP = 32
        self.kI = 0
        self.kD = 32
        self.curr_lim = 500     # 500 * 0.0065A = 3.25A
        self.prev_pos = self.pos = self.curr_pos = np.zeros(7)
        self.motors = [1, 2, 3, 4, 5, 6, 7]

        # Connect to arm
        self.apto_client = AptoClient(self.motors, 'COM4', 1000000)
        self.apto_client.connect()

        # Set the default parameters
        self.apto_client.set_torque_enabled(self.motors, True)
        self.apto_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 21, 2) # Pgain stiffness     
        self.apto_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 23, 2) # Igain
        self.apto_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 22, 2) # Dgain damping
        self.apto_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 28, 2) # 500 normal limit or 350 for lite limit
        
        # Command a position
        self.apto_client.write_desired_pos(self.motors, self.curr_pos)

    # set a goal pose for the joints (radians)
    def set_pose(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.apto_client.write_desired_pos(self.motors, self.curr_pos)

    # read position of robot joints
    def read_pos(self):
        return self.apto_client.read_pos()

def main():
    apto_arm = AptoNode()
    while True:
        apto_arm.set_pose(np.zeros(7))
        print("Position: " + str(apto_arm.read_pos()))
        time.sleep(0.05)

if __name__ == "__main__":
    main()
