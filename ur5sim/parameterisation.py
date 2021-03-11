import json
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
import scipy.signal
import seaborn as sns
from scipy.spatial.transform import Rotation as Rot

# Scaling XYZ
# Translation XYZ
# Offset
# Rotation XYZ


whisking_recordings = ["20201221-171929-whiskrec1", "20201221-172049-whiskrec2", "20201221-172205-whiskrec3", "20201221-172335-whiskrec4", "20201221-172446-whiskrec5", "20201221-173051-whiskrecnew1"]
beans_recordings = ["20201221-172620-beansrec1", "20201221-172739-beansrec2", "20201221-172852-beansrec3"]


def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w


def nan_helper(y):
    return np.isnan(y), lambda z: z.nonzero()[0]


def get_clean_func(object_loc_orig_data):
    object_loc_data = np.stack([np.array(v[1]) if v[1] is not None else [np.NaN, np.NaN, np.NaN] for v in object_loc_orig_data])
    tw = np.array([v[0] for v in object_loc_orig_data])

    #print(tw)

    object_loc_data_av = np.zeros_like(object_loc_data)

    for i in range(len(object_loc_data)):
        if np.isnan(object_loc_data[i][0]):
            object_loc_data_av[i] = [np.NaN, np.NaN, np.NaN]
            continue

        w = 2
        n = 0
        for o in range(-w, w + 1):
            if 0 <= i + o < len(object_loc_data) and not np.isnan(object_loc_data[i + o][0]):
                n += 1
                object_loc_data_av[i] += object_loc_data[i + o]

        object_loc_data_av[i] /= n

    object_loc_data = object_loc_data_av

    nans = [np.isnan(v[0]) for v in object_loc_data]

    object_loc_data = np.delete(object_loc_data, nans, axis=0)
    tw = np.delete(tw, nans, axis=0)

    object_loc_data_f = interp1d(tw, object_loc_data, axis=0, bounds_error=False, fill_value=(object_loc_data[0], object_loc_data[-1]))

    return object_loc_data_f


def load(recording_name):
    arm_data = json.load(open("records/" + recording_name + "/arm_trajectory.json"))
    arm_data_f = interp1d([v[3] for v in arm_data], [v[0][:3] for v in arm_data], axis=0, bounds_error=False)

    object_data = json.load(open("records/" + recording_name + "/objects.json"))

    recording_start = arm_data[0][3]
    recording_end = arm_data[-1][3]

    data = {}

    data["arm_data"] = arm_data

    data["t_start"] = recording_start
    data["t_end"] = recording_end

    data["arm_f"] = arm_data_f

    data["object_f"] = {}

    for k in object_data.keys():
        data["object_f"][k] = get_clean_func(object_data[k])

    return data


def space_transform_arm(data, offset_object=None, dist=0.01, sample_freq=10):
    samples = np.linspace(data["t_start"], data["t_end"], round(sample_freq * (data["t_end"] - data["t_start"])))

    if offset_object is None:
        f = data["arm_f"]
    else:
        def f(x):
            return data["arm_f"](x) - data["object_f"][offset_object](x)

    t = [samples[0]]
    points = [f(samples[0])]

    # print(data["object_f"][offset_object](samples[0]))
    # print(points[0])

    for ts in samples:
        if np.linalg.norm(f(ts) - points[-1]) > dist:
            points.append(f(ts))
            t.append(ts)
            # print("Append")

    return np.array(t), np.array(points)


def compare_trajectories(a, b):
    if len(a) < len(b):
        tmp = a
        a = b
        b = tmp

    offset = -len(b)

    result = np.zeros((len(a) + len(b), len(b)))

    while offset < len(a):
        for i in range(len(b)):
            if 0 <= offset + i < len(a):
                result[offset + len(b)][i] = np.linalg.norm(b[i] - a[offset + i])

        offset += 1

    return result


def cost(a, b, a_offset=0):
    total_error = 0
    compares = 0
    for i in range(len(a)):
        if 0 <= i + a_offset < len(a) and 0 <= i < len(b):
            total_error += np.linalg.norm(a[i + a_offset] - b[i])
            compares += 1

    return total_error/compares


master = load(beans_recordings[0])
secondary = load(beans_recordings[1])

st_m_t, st_m = space_transform_arm(master)
st_s_t, st_s = space_transform_arm(secondary)

ma, mb = 41, -27
st_m_t = st_m_t[ma:mb]
st_m = st_m[ma:mb]

st_m -= np.average(st_m, axis=0)

sa, sb = 33, -27
st_s_t = st_s_t[sa:sb]
st_s = st_s[sa:sb]

print(np.average(st_s, axis=0))

st_s -= np.average(st_s, axis=0)

fig, ax = plt.subplots(3, 2)

tm = np.linspace(st_m_t[0], st_m_t[-1], 1000)
ax[0, 0].plot(tm, master["arm_f"](tm))
ax[0, 0].title.set_text("Master trajectory")

ts = np.linspace(st_s_t[0], st_s_t[-1], 1000)
ax[0, 1].plot(ts, secondary["arm_f"](ts))
ax[0, 1].title.set_text("Secondary trajectory")

ax[1, 0].plot(st_m_t, st_m)
ax[1, 1].plot(st_s_t, st_s)

ax[2, 0].plot(st_m[:, 0], st_m[:, 1])
ax[2, 1].plot(st_s[:, 0], st_s[:, 1])

fig2, ax2 = plt.subplots(2, 1)

ax2[0].plot(st_m[:, 0], st_m[:, 1])
ax2[0].plot(st_s[:, 0], st_s[:, 1])

ax2[0].title.set_text("Unscaled vs scaled trajectory")

print("Initial cost:", cost(st_m, st_s))

offset = np.array([i for i in range(-20, 20)])

costs_o = np.zeros(len(offset))

for on, ov in enumerate(offset):
    costs_o[on] = cost(st_m, st_s, a_offset=ov)

min_coords = np.argmin(costs_o)
min_offset = offset[min_coords]

# NEED TO REMOVE THIS!!!!!
min_offset = 0

plt.figure()

plt.plot(offset, costs_o)
plt.gca().set_xlabel("'Spatial' trajectory offset")
plt.gca().set_ylabel("Error")
plt.gca().title.set_text("Offset optimisation")

print("Min offset:", min_offset, "error", costs_o[min_coords])

n = 20
x = np.linspace(0.7, 1.3, n)
y = np.linspace(0.7, 1.3, n)
z = np.linspace(0.7, 1.3, n)

costs = np.zeros((n, n, n))

for xn, xv in enumerate(x):
    for yn, yv in enumerate(y):
        for zn, zv in enumerate(z):
            costs[xn, yn, zn] = cost(st_m, st_s * [xv, yv, zv], a_offset=min_offset)

min_coords = np.unravel_index(np.argmin(costs, axis=None), costs.shape)
scaling = (x[min_coords[0]], y[min_coords[1]], z[min_coords[2]])
print("Min in scaling", scaling, "error", costs[min_coords])

ax2[1].plot(st_m[:, 0], st_m[:, 1])
ax2[1].plot((st_s * scaling)[:, 0], (st_s * scaling)[:, 1])

plt.figure()

plt.imshow(costs[:, :, 0], extent=[x[0], x[-1], y[-1], y[0]])
#plt.gca().set_aspect(100)
plt.gca().set_xlabel("Scaling in X")
plt.gca().set_ylabel("Scaling in Y")
plt.gca().title.set_text("Scaling optimisation")

plt.colorbar(orientation='vertical')

n = 30

xdeg = np.linspace(-180, 180, n)
ydeg = np.linspace(-180, 180, n)
zdeg = np.linspace(-180, 180, n)

costs2 = np.zeros((n, n, n))

for xdegn, xdegv in enumerate(xdeg):
    for ydegn, ydegv in enumerate(ydeg):
        for zdegn, zdegv in enumerate(zdeg):
            tmp = Rot.from_euler("x", xdegv, degrees=True).apply(st_s)
            tmp = Rot.from_euler("y", ydegv, degrees=True).apply(tmp)
            tmp = Rot.from_euler("z", zdegv, degrees=True).apply(tmp)
            costs2[xdegn, ydegn, zdegn] = cost(st_m, tmp, a_offset=min_offset)

min_coords = np.unravel_index(np.argmin(costs2, axis=None), costs2.shape)
rotation = (xdeg[min_coords[0]], ydeg[min_coords[1]], zdeg[min_coords[2]])
print("Min in rotation", rotation, "error", costs2[min_coords])

plt.figure()

plt.imshow(costs2, extent=[xdeg[0], xdeg[-1], ydeg[0], ydeg[-1]])
#plt.scatter(min_coords[1], min_coords[0], s=50, c='red', marker='x')
#plt.gca().set_aspect(0.2)
plt.gca().set_xlabel("Rotation about x axis")
plt.gca().set_ylabel("Rotation about y axis")
plt.gca().title.set_text("Rotation optimisation")

plt.colorbar(orientation='vertical')

n = 20

x = np.linspace(-0.05, 0.05, n)
y = np.linspace(-0.05, 0.05, n)
z = np.linspace(-0.05, 0.05, n)

costs3 = np.zeros((n, n, n))

for xn, xv in enumerate(x):
    for yn, yv in enumerate(y):
        for zn, zv in enumerate(z):
            costs3[xn, yn, zn] = cost(st_m, st_s - [xv, yv, zv])

min_coords = np.unravel_index(np.argmin(costs3, axis=None), costs3.shape)
print("Min in translation", x[min_coords[0]], y[min_coords[1]], z[min_coords[2]], "error", costs3[min_coords], "vs", cost(st_m, st_s))

plt.figure()

plt.imshow(costs3[:,0,:])
plt.gca().set_xlabel("Translation in x")
plt.gca().set_ylabel("Translation in y")
plt.gca().title.set_text("Translation optimisation")

plt.show()
