import json
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

poses = json.load(open("records/20201221-164924/arm_trajectory.json"))

fig = plt.figure()
ax = fig.gca()
x = [p[0][0] for p in poses]
y = [p[0][1] for p in poses]
z = [p[0][2] for p in poses]

print(poses[0])

gripper = [p[2] for p in poses]
t = np.linspace(0, 0.1 * len(gripper), len(gripper))

print(len(gripper))

ax.plot(t, x, label='x')
ax.plot(t, y, label='y')
ax.plot(t, z, label='z')
ax.legend()

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("End effector positon vs time")

segments = [4, 14, 17, 28, 32.5, 41]

for x in segments:
    plt.axvline(x=x, dashes=(5,2,1,2))

freedrive = False

plt.show()
