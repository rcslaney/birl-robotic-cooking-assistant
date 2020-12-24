import rtde_control
import time
import json
from gripper import Gripper
import numpy as np

print("Connecting?")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.6")
print("Connected?")

gripper = Gripper(2, 1)

# Parameters
velocity = 0.25
acceleration = 0.25
dt = 1.0/10  # 2ms
lookahead_time = 0.2
gain = 200
joint_q = [-1.54, -1.4, -2.28, -0.59, 1.60, 0.023]

sequence = json.load(open("records/20201217-171704/arm_trajectory.json"))

input("Play?")

#offset = [0.25493689, -0.56119273, -0.02703432, 0, 0, 0]
offset = [0, 0, 0, 0, 0, 0]
# Move to initial joint position with a regular moveJ
rtde_c.moveL(np.add(offset, sequence[0][0]).tolist())
print("moved?")
time.sleep(1)
print("Moved to inital position")

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(len(sequence)):
    start = time.time()
    rtde_c.servoL(np.add(offset, sequence[i][0]), velocity, acceleration, dt, lookahead_time, gain)
    gripper.set_state(sequence[i][2])
    end = time.time()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)

print("Control loop done")

rtde_c.servoStop()
rtde_c.stopScript()
