import time
import traceback
import numpy as np
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import os
from dt_apriltags import Detector, Detection
from robot_utils import UR5SimSync
from robot_utils import p
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json


class CustomDetection:
    def __init__(self, detection):
        self.age = 0
        self.detection = detection

    def __getattr__(self, item):
        return getattr(self.detection, item)

    def tick(self):
        self.age += 1

    def is_new(self):
        return self.age == 0


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def snapshot():
    global master_tag, translations, transforms
    #robot.get_state()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    json.dump(trajectories, open("trajectory_recording_" + str(round(time.time())) + ".json", "w"))

    # print(whisk_trajectory)
    #
    # xs = [p[0][0] for p in whisk_trajectory]
    # ys = [p[1][0] for p in whisk_trajectory]
    # zs = [p[2][0] for p in whisk_trajectory]
    #
    # ax.plot(xs, ys, zs)
    # set_axes_equal(ax)
    # plt.show()

    for i in translations.keys():
        for j in translations[i].keys():
            if translations[i] is None or translations[i][j] is None:
                continue
            print(i, ":", j)
            print(np.average(transforms[i][j], axis=0))
            print(np.average(translations[i][j], axis=0))
            print(np.linalg.norm(np.var(translations[i][j], axis=0)))


def get_cameras():
    stream = os.popen("v4l2-ctl --list-devices")
    op = stream.read().split("\n")
    cameras = {}
    desc = ""
    i = 0
    while i < len(op):
        if len(op[i]) > 1:
            if op[i][1] != "/":
                desc = op[i]
            else:
                cameras[desc + "(" + op[i][1:] + ")"] = op[i][1:]
        i += 1
    return cameras


def set_camera_parameter(key, value, device):
    stream = os.popen("v4l2-ctl -d {} --set-ctrl {}={}".format(device, key, value))
    op = stream.read()
    if len(op) > 1:
        raise Exception(op)


def update_detection_data(curr_detections, detections):
    """ Takes current detections and adds them to global 'detections' adding an age parameter """

    for d in detections.values():
        d.tick()

    for d in curr_detections:
        detections[d.tag_id] = CustomDetection(d)

    return detections


def draw_outlines(detections, frame, max_age=0):
    for d in filter(lambda t: t.age <= max_age, detections.values()):
        frame = draw_poly(d.corners, frame)

    return frame


def draw_poly(corners, frame):
    """ Draws polygon on CV2 image frame"""
    for i in range(0, 4):
        frame = cv2.line(frame, (round(corners[i][0]), round(corners[i][1])), (round(corners[(i + 1) % 4][0]), round(corners[(i + 1) % 4][1])), (255, 0, 0), 8)
    return frame


def draw_debug_text(detections, frame, max_age=0):
    for y, d in enumerate(filter(lambda t: t.age <= max_age, detections.values())):
        cv2.putText(frame, str(d.tag_id), (10, 30 + y * 27), cv2.FONT_HERSHEY_COMPLEX, 0.75, (0, 0, 255))

    return frame


def extract_all_transforms(detections):
    global master_tags
    for master_tag_id in master_tags:
        if master_tag_id not in detections:
            continue

        master_tag = detections[master_tag_id]
        for secondary_tag in detections.values():
            if master_tag.tag_id != secondary_tag.tag_id and secondary_tag.tag_id not in master_tags:
                extract_transform(master_tag, secondary_tag)


def extract_transform(master_d, secondary_d):
    """ Updates the transform between a master and secondary tag in the scene """
    if master_d.tag_id not in transforms:
        transforms[master_d.tag_id] = {}
        translations[master_d.tag_id] = {}

    if secondary_d.tag_id not in transforms[master_d.tag_id]:
        transforms[master_d.tag_id][secondary_d.tag_id] = np.zeros((30, 3, 3))
        translations[master_d.tag_id][secondary_d.tag_id] = np.zeros((30, 3, 1))

    transforms[master_d.tag_id][secondary_d.tag_id] = np.roll(transforms[master_d.tag_id][secondary_d.tag_id], 1, axis=0)
    transforms[master_d.tag_id][secondary_d.tag_id][0] = (np.linalg.inv(master_d.pose_R) @ secondary_d.pose_R)

    translations[master_d.tag_id][secondary_d.tag_id] = np.roll(translations[master_d.tag_id][secondary_d.tag_id], 1, axis=0)
    translations[master_d.tag_id][secondary_d.tag_id][0] = (np.linalg.inv(master_d.pose_R) @ (master_d.pose_t - secondary_d.pose_t))


def extract_world_coordinates(reference_d, object_d):
    """ Takes a reference tag and an object tag and returns its translation wrt. the world frame"""
    point = object_d.pose_t - object_d.pose_R @ np.array([[0], [0], [-0.02]]) - reference_d.pose_t
    point = np.linalg.inv(reference_d.pose_R) @ point
    point[2][0] *= -1
    point[1][0] *= -1
    return point


def show_frame():
    """ Main loop """
    global cap, detections, transforms, translations

     #p.removeAllUserDebugItems()

    try:
        _, frame = cap.read()

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = update_detection_data(at_detector.detect(gray_frame, estimate_tag_pose=True, camera_params=brio_params, tag_size=0.032, tag_size_overrides=tag_size_overrides), detections)

        if learning_mode.get():
            extract_all_transforms(detections)

        frame = draw_outlines(detections, frame)

        frame = draw_debug_text(detections, frame)

        tmp_t = {}
        for i in master_tags:
            tmp_t[i] = []

        for d in detections.values():
            t = extract_world_coordinates(detections[0], d)
            if d.tag_id in master_tags:
                tmp_t[d.tag_id].append(t)
            else:
                for k, v in secondary_tags.items():
                    if d.tag_id in v:
                        tmp_t[k].append(t)
                        break

        for k, v in tmp_t.items():
            if len(v) > 0:
                trajectories[k].append([time.time(), list(np.average(v, axis=0).flatten())])
            else:
                trajectories[k].append([time.time(), None])

        #print(tmp_t)


        # for d in detections.values():
        #     #if d.tag_id == 4:
        #     #    continue
        #
        #     if d.tag_id != 0:
        #         point_a = d.pose_t - d.pose_R @ np.array([[0], [0], [-0.02]]) - detections[0].pose_t
        #         try:
        #             point_b = d.pose_t - d.pose_R @ np.array([[0], [0], [-0.02]]) - (d.pose_R @ np.linalg.inv(np.average(transforms[master_tag][d.tag_id], axis=0))) @ np.array([[0], [0], [-0.3]]) - detections[0].pose_t
        #
        #             point_a = np.linalg.inv(detections[0].pose_R) @ point_a
        #             point_b = np.linalg.inv(detections[0].pose_R) @ point_b
        #
        #             point_a[2][0] *= -1
        #             point_b[2][0] *= -1
        #
        #             point_a[1][0] *= -1
        #             point_b[1][0] *= -1
        #
        #             # p.addUserDebugLine(point_a.flatten(), point_b.flatten(), [1, 0, 0], lineWidth=3)
        #         except Exception as e:
        #             print("error", e)

        # whisk_translation = np.array([[0.0], [0.0], [0.0]])
        # for d in whisk_detections:
        #     whisk_translation += extract_world_coordinates(detections[0], d)
        #
        # whisk_translation /= len(whisk_detections)
        #
        # whisk_translation[0][0] += 0.27
        #
        # print(whisk_translation)

        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)

        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    except Exception as e:
        print(traceback.format_exc())
    lmain.after(33, show_frame)


def change_camera(e):
    global camera, cameras, cap
    no = "".join([s for s in cameras[camera.get()] if s.isdigit()])
    cap = cv2.VideoCapture(int(no), cv2.CAP_V4L)

    print_properties()


def print_properties():
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(width, height, fps)


def set_good_parameters():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 60)

    print_properties()


#robot = UR5SimSync(robot_ip="192.168.1.6")
#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

trajectories = {14: [], 36: []}

# Set up GUI
window = tk.Tk()  # Makes main window
window.wm_title("Tracking")
window.config()

# Graphics window
imageFrame = tk.Frame(window, width=600, height=500)
imageFrame.grid(row=0, column=0, padx=10, pady=2)

# Capture video frames
lmain = tk.Label(imageFrame)
lmain.grid(row=0, column=0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L)

# Set up detector
at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

detections = {}

transforms = {}
translations = {}

c270_params = [1402, 1402, 642, 470]  # Probably at odd resolution
brio_params = [499.0239, 499.1960, 310.1258, 232.4641]  # At 640x480

tag_size_overrides = {
    0: 0.092
}

master_tags = [14, 36]

secondary_tags = {
    14: [10, 11, 12, 13],
    36: []
}

# Slider window (slider controls stage position)
sliderFrame = tk.Frame(window, width=900, height=100)
sliderFrame.grid(row=1, column=0, padx=10, pady=2)

btn = tk.Button(sliderFrame, text="Snapshot", command=snapshot)
btn.pack(fill="both", expand=True, padx=10, pady=10)

btn2 = tk.Button(sliderFrame, text="Set camera params", command=set_good_parameters)
btn2.pack(fill="both", expand=True, padx=10, pady=10)

cameras = get_cameras()

cam_label = tk.Label(sliderFrame, text="Camera device")
cam_label.pack()
camera = tk.StringVar()
camera.set(list(cameras.keys())[0])
w = tk.OptionMenu(sliderFrame, camera, *tuple(cameras.keys()), command=change_camera)
w.pack()

powerline_options = {
    "No correction": 0,
    "50Hz correction": 1,
    "60Hz correciton": 2
}

plc_label = tk.Label(sliderFrame, text="Powerline correction")
plc_label.pack()
powerline_compensation = tk.StringVar()
plc = tk.OptionMenu(sliderFrame, powerline_compensation, *tuple(powerline_options.keys()), command=lambda e: set_camera_parameter("power_line_frequency", powerline_options[powerline_compensation.get()], cameras[camera.get()]))
plc.pack(expand=True, fill="x")

manual_exposure = tk.IntVar()
cb = tk.Checkbutton(sliderFrame, text="Manual exposure", variable=manual_exposure, command=lambda: set_camera_parameter("exposure_auto", 1 if bool(manual_exposure.get()) else 3, cameras[camera.get()]))
cb.pack()

exposure = tk.IntVar()
s = tk.Scale(sliderFrame, from_=0, to=1000, tickinterval=100, orient=tk.HORIZONTAL, label="Exposure", command=lambda e: set_camera_parameter("exposure_absolute", s.get(), cameras[camera.get()]))
s.pack(expand=True, fill="x")

learning_mode = tk.IntVar()
lm_cb = tk.Checkbutton(sliderFrame, text="Learning mode", variable=learning_mode)
lm_cb.pack()

show_frame()
window.mainloop()
