import json
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
import scipy.signal
import seaborn as sns

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


def space_transform_arm(data, offset_object=None):
    samples = np.linspace(data["t_start"], data["t_end"], round(10 * (data["t_end"] - data["t_start"])))

    if offset_object is None:
        f = data["arm_f"]
    else:
        def f(x):
            return data["arm_f"](x) - data["object_f"][offset_object](x)

    t = [samples[0]]
    points = [f(samples[0])]

    print(data["object_f"][offset_object](samples[0]))
    print(points[0])

    for ts in samples:
        if np.linalg.norm(f(ts) - points[-1]) > 0.01:
            points.append(f(ts))
            t.append(ts)

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


fig, axs = plt.subplots(3, 3)

data1 = load(beans_recordings[0])
data2 = load(beans_recordings[1])

t1 = np.linspace(data1["t_start"], data1["t_end"], 1000)
t2 = np.linspace(data2["t_start"], data2["t_end"], 1000)

ts1, st_data1 = space_transform_arm(data1, offset_object="36")
ts2, st_data2 = space_transform_arm(data2, offset_object="36")

time_width = data1["t_end"] - data1["t_start"]

offsets = np.linspace(-time_width, +time_width, 100)

axs[0, 0].plot(st_data1[:, 0], st_data1[:, 1])
axs[1, 1].plot(st_data2[:, 0], st_data2[:, 1])
print(len(st_data1), len(st_data2))

axs[0, 1].plot(data1["arm_f"](t1)[:, 0], data1["arm_f"](t1)[:, 1])

axs[1, 0].plot(t2, data2["object_f"]["36"](t2))

res = compare_trajectories(st_data1, st_data2)
plt.figure()
ax = sns.heatmap(res, linewidth=0)

plt.figure()
res2 = np.abs(moving_average(np.gradient(moving_average(np.sum(res, axis=0), 20)), 30))
peaks, properties = scipy.signal.find_peaks(res2, prominence=0.4)
print(peaks)
print(properties)
print(data1["arm_f"](t1))

offset = 25


def arm_obj_dist(t, data, obj_id):
    return np.linalg.norm(data["object_f"][obj_id](t) - data["arm_f"](t), axis=1)


if False and len(ts2) > len(ts1):
    print("AXS[2,0] is series 1")
    print(ts1[peaks[0]], ts1[peaks[1]])
    axs[2, 0].plot(t1, data1["arm_f"](t1))
    axs[2, 0].axvline(ts1[peaks[0] + offset], dashes=(5, 2, 1, 2))
    axs[2, 0].axvline(ts1[peaks[1] + offset], dashes=(5, 2, 1, 2))
    axs[2, 1].plot(t1, arm_obj_dist(t1, data1, "14"))
    axs[2, 2].plot(t1, arm_obj_dist(t1, data1, "36"))

    idx = np.argwhere(np.diff(np.sign(arm_obj_dist(t1, data1, "14") - 0.35))).flatten()
    print("These:", t1[idx])
    axs[2, 0].axvline(t1[idx][0], dashes=(5, 2, 1, 2), color="red")
    axs[2, 0].axvline(t1[idx][1], dashes=(5, 2, 1, 2), color="red")

    idx = np.argwhere(np.diff(np.sign(arm_obj_dist(t1, data1, "36") - 0.35))).flatten()
    print("These:", t1[idx])
    axs[2, 0].axvline(t1[idx][0], dashes=(5, 2, 1, 2), color="red")
    axs[2, 0].axvline(t1[idx][1], dashes=(5, 2, 1, 2), color="red")

    gripper = [p[2] for p in data1["arm_data"]]
    idx = np.argwhere(np.diff(gripper)).flatten()
    times = [data1["arm_data"][idx[i]][3] for i in range(len(idx))]

    for time in times:
        axs[2, 0].axvline(time, dashes=(5, 2, 1, 2), color="green")
else:
    print("AXS[2,0] is series 2")
    print(ts2[peaks[0]], ts2[peaks[1]])
    axs[2, 0].plot(t2, data2["arm_f"](t2))
    axs[2, 0].axvline(ts2[peaks[0] + offset], dashes=(5, 2, 1, 2))
    axs[2, 0].axvline(ts2[peaks[1] + offset], dashes=(5, 2, 1, 2))
    axs[2, 1].plot(t2, arm_obj_dist(t2, data2, "14"))
    axs[2, 2].plot(t2, arm_obj_dist(t2, data2, "36"))

    idx = np.argwhere(np.diff(np.sign(arm_obj_dist(t2, data2, "14") - 0.35))).flatten()
    print("These:", t2[idx])
    axs[2, 0].axvline(t2[idx][0], dashes=(5, 2, 1, 2), color="red")
    axs[2, 0].axvline(t2[idx][1], dashes=(5, 2, 1, 2), color="red")

    idx = np.argwhere(np.diff(np.sign(arm_obj_dist(t2, data2, "36") - 0.35))).flatten()
    print("These:", t2[idx])
    axs[2, 0].axvline(t2[idx][0], dashes=(5, 2, 1, 2), color="red")
    axs[2, 0].axvline(t2[idx][1], dashes=(5, 2, 1, 2), color="red")

    gripper = [p[2] for p in data2["arm_data"]]
    idx = np.argwhere(np.diff(gripper)).flatten()
    times = [data2["arm_data"][idx[i]][3] for i in range(len(idx))]

    for time in times:
        axs[2, 0].axvline(time, dashes=(5, 2, 1, 2), color="green")


def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    print(ix)


cid = fig.canvas.mpl_connect('button_press_event', onclick)

print(times)

plt.plot(res2)

plt.show()

