import matplotlib.pyplot as plt
import json
import numpy as np
from scipy import interpolate

armd = json.load(open("records/20201221-164924/arm_trajectory.json", "r"))
trajd = json.load(open("records/20201221-164924/objects.json", "r"))

x = [p[0][0] for p in armd]
y = [p[0][1] for p in armd]
z = [p[0][2] for p in armd]

plt.figure(100)

plt.plot(x)
plt.plot(y)
plt.plot(z)

plt.figure(200)

x = [p[1][0] + 0.25 if p[1] is not None else np.nan for p in trajd["14"]]
y = [p[1][1] if p[1] is not None else np.nan for p in trajd["14"]]
z = [p[1][2] if p[1] is not None else np.nan for p in trajd["14"]]

plt.plot(x)
plt.plot(y)
plt.plot(z)

plt.figure(300)

x = [p[1][0] if p[1] is not None else np.nan for p in trajd["36"]]
y = [p[1][1] if p[1] is not None else np.nan for p in trajd["36"]]
z = [p[1][2] if p[1] is not None else np.nan for p in trajd["36"]]
t = [p[0] for p in trajd["36"]]

plt.plot(t, x)
plt.plot(t, y)
plt.plot(y, z)

plt.figure(400)

s = [p[2] for p in armd]

plt.plot(s)

plt.figure(500)

t1 = [p[0] for p in trajd["14"]]
x1 = [p[1][0] if p[1] is not None else np.nan for p in trajd["14"]]
y1 = [p[1][1] if p[1] is not None else np.nan for p in trajd["14"]]
z1 = [p[1][2] if p[1] is not None else np.nan for p in trajd["14"]]


x2 = [p[0][0] for p in armd]
y2 = [p[0][1] for p in armd]
z2 = [p[0][2] for p in armd]
t2 = [p[3] for p in armd]

plt.plot(t1, x1, label="Whisk X")
plt.plot(t2, x2, label="Arm X")

plt.plot(t1, y1, label="Whisk Y")
plt.plot(t2, y2, label="Arm Y")

plt.plot(t1, z1, label="Whisk Z")
plt.plot(t2, z2, label="Arm Z")

plt.legend()

# whisk_x = interpolate.interp1d(t1, x1)
# whisk_y = interpolate.interp1d(t1, y1)
# whisk_z = interpolate.interp1d(t1, z1)
#
# arm_x = interpolate.interp1d(t2, x2)
# arm_y = interpolate.interp1d(t2, y2)
# arm_z = interpolate.interp1d(t2, z2)
#
# dist = []
#
# for t in t2:
#     dist.append(((whisk_x(t) - arm_x(t)) ** 2 + (whisk_y(t) - arm_y(t)) ** 2 + (whisk_z(t) - arm_z(t)) ** 2) ** 0.5)
#
# plt.figure(600)
# plt.plot(t2, dist)

plt.show()
