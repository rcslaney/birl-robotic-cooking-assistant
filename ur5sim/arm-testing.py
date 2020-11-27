import time
import threading
from robot_utils import UR5SimSync
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import matplotlib.pyplot as plt

robot = UR5SimSync(robot_ip="192.168.1.6")

def freedrive_loop():
    print("Started!")
    while True:
        global freedrive
        robot.freedrive()
        time.sleep(0.05)
        if not freedrive:
            print("Broke!")
            break


poses = []
def record_loop():
    global poses, robot

    while True:
        global record
        poses.append(robot.get_state()["actual_TCP_pose"])
        if not record:
            break


freedrive_thread = threading.Thread(target=freedrive_loop)
record_thread = threading.Thread(target=record_loop)

input("Press enter to start...")

freedrive = True
freedrive_thread.start()

record = True
record_thread.start()

input("Press enter to stop...")

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
x = [p[0] for p in poses]
y = [p[1] for p in poses]
z = [p[2] for p in poses]


print(len(x), len(y), len(z))
ax.plot(x, y, z, label='End effector trajectory')
ax.legend()

freedrive = False

plt.show()

print("Freedrive wait end")


time.sleep(1)

robot.disconnect()
