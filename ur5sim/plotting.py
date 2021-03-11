import json
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
from scipy.optimize import fsolve


def nan_helper(y):
    return np.isnan(y), lambda z: z.nonzero()[0]

recording_name = "20201221-172620-beansrec1"

arm_data = json.load(open("records/" + recording_name + "/arm_trajectory.json"))
object_data = json.load(open("records/" + recording_name + "/objects.json"))

fig, axs = plt.subplots(3, 3)

axs[0, 0].title.set_text("Arm trajectory")

x = [p[0][0] for p in arm_data]
y = [p[0][1] for p in arm_data]
z = [p[0][2] for p in arm_data]
t = [p[3] for p in arm_data]

axs[0, 0].plot(t, x)
axs[0, 0].plot(t, y)
axs[0, 0].plot(t, z)

axs[0, 1].title.set_text("Whisk trajectory")

whisk_loc = object_data["14"]

xw = [p[1][0] if p[1] is not None else np.NaN for p in whisk_loc]
yw = [p[1][1] if p[1] is not None else np.NaN for p in whisk_loc]
zw = [p[1][2] if p[1] is not None else np.NaN for p in whisk_loc]
tw = [p[0] for p in whisk_loc]

axs[0, 1].plot(tw, xw)
axs[0, 1].plot(tw, yw)
axs[0, 1].plot(tw, zw)

axs[1, 0].title.set_text("Arm-whisk distance")

f_x = interp1d(t, x)
f_y = interp1d(t, y)
f_z = interp1d(t, z)

f_xw = interp1d(tw, xw)
f_yw = interp1d(tw, yw)
f_zw = interp1d(tw, zw)

arm_whisk_dist = np.sqrt((f_x(tw) - f_xw(tw)) ** 2 + (f_y(tw) - f_yw(tw)) ** 2 + (f_z(tw) - f_zw(tw)) ** 2)

axs[1, 0].plot(tw, arm_whisk_dist)

axs[1, 1].title.set_text("Arm-pan distance")

pan_loc = object_data["36"]

xp = [p[1][0] if p[1] is not None else None for p in pan_loc]
yp = [p[1][1] if p[1] is not None else None for p in pan_loc]
zp = [p[1][2] if p[1] is not None else None for p in pan_loc]

f_xp = interp1d(tw, xp)
f_yp = interp1d(tw, yp)
f_zp = interp1d(tw, zp)

arm_pan_dist = np.sqrt((f_x(tw) - f_xp(tw)) ** 2 + (f_y(tw) - f_yp(tw)) ** 2 + (f_z(tw) - f_zp(tw)) ** 2)

axs[1, 1].plot(tw, arm_pan_dist)

axs[0, 2].title.set_text("Whisk X-Y")

# axs[0, 2].plot(xw, yw)

# Interpolation time

whisk_loc = np.stack([np.array(v[1]) if v[1] is not None else [np.NaN, np.NaN, np.NaN] for v in whisk_loc])
tw = np.array(tw)

whisk_loc_av = np.zeros_like(whisk_loc)

for i in range(len(whisk_loc)):
    if np.isnan(whisk_loc[i][0]):
        whisk_loc_av[i] = [np.NaN, np.NaN, np.NaN]
        continue

    w = 2
    n = 0
    for o in range(-w, w+1):
        if 0 <= i + o < len(whisk_loc) and not np.isnan(whisk_loc[i + o][0]):
            n += 1
            whisk_loc_av[i] += whisk_loc[i + o]

    whisk_loc_av[i] /= n

whisk_loc = whisk_loc_av

nans = [np.isnan(v[0]) for v in whisk_loc]

whisk_loc = np.delete(whisk_loc, nans, axis=0)
tw = np.delete(tw, nans, axis=0)

whisk_loc_f = interp1d(tw, whisk_loc, axis=0)

t = np.linspace(tw[0], tw[-1], 100)

arm_whisk_dist = np.sqrt((f_x(t) - whisk_loc_f(t)[:,0]) ** 2 + (f_y(t) - whisk_loc_f(t)[:,1]) ** 2 + (f_z(t) - whisk_loc_f(t)[:,2]) ** 2)


def arm_whisk_dist_f(t):
    return np.sqrt((f_x(t) - whisk_loc_f(t)[:,0]) ** 2 + (f_y(t) - whisk_loc_f(t)[:,1]) ** 2 + (f_z(t) - whisk_loc_f(t)[:,2]) ** 2)


axs[2, 0].plot(t, arm_whisk_dist)

axs[1, 2].plot(tw, whisk_loc)

axs[0, 2].plot(whisk_loc[:,0], whisk_loc[:,1])

ts = np.linspace(t[0], t[-1], 10000)
awd = arm_whisk_dist_f(ts)

triggers = np.zeros_like(awd) + 0.3

idx = np.argwhere(np.diff(np.sign(awd - triggers))).flatten()
axs[2,0].plot(ts[idx], awd[idx], 'ro')

axs[2, 0].plot(ts, triggers)

print("Solved!", fsolve(lambda t: arm_whisk_dist_f(t) - 0.35, t[0]))

plt.tight_layout()

print(np.argwhere(np.diff(np.sign(arm_pan_dist - 0.37))).flatten())
test = np.argwhere(np.diff(np.sign(arm_whisk_dist - 0.37))).flatten()
# for i in test:
#     print(tw[i])
#     axs[0, 0].plot(tw[i], 0, marker="o")

plt.show()


