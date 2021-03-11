import time
import traceback
import numpy as np
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import os
from dt_apriltags import Detector, Detection
from robot_utils_urx import UR5SimSync
from robot_utils_urx import p
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import threading
from tkinter import simpledialog, filedialog
import tkinter.ttk
from gripper import Gripper


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

    # printc(whisk_trajectory)
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
            printc(i, ":", j)
            printc(np.average(transforms[i][j], axis=0))
            printc(np.average(translations[i][j], axis=0))
            printc(np.linalg.norm(np.var(translations[i][j], axis=0)))


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


def move_sphere():
    global sphereId
    while True:
        for i in range(0, 10):
            print(i)
            p.resetBasePositionAndOrientation(sphereId, [0, 0, i/10], [0, 0, 0, 1])
            time.sleep(0.1)


def create_sphere():
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                        radius=0.05,
                                        rgbaColor=[1, 0, 0, 1],
                                        visualFramePosition=[0, 0, 0])
    sphereId = p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0, 0, 1])

    return sphereId


def create_box(half_extents, position):
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=half_extents,
                                        rgbaColor=[0.7, 0.7, 0.7, 1],
                                        visualFramePosition=[0, 0, 0])

    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=half_extents,
                                        collisionFramePosition=[0, 0, 0])

    boxId = p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseVisualShapeIndex=visualShapeId,
                      baseCollisionShapeIndex=collisionShapeId,
                      basePosition=position)

    return boxId


def create_object(filename, position):
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                        fileName=filename,
                                        rgbaColor=[0.7, 0.7, 0.7, 1],
                                        visualFramePosition=[0, 0, 0],
                                        meshScale=[1/1000, 1/1000, 1/1000])

    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                        fileName=filename,
                                        halfExtents=filename,
                                        collisionFramePosition=[0, 0, 0],
                                        meshScale=[1/1000, 1/1000, 1/1000])

    objectId = p.createMultiBody(baseMass=0.2,
                      baseInertialFramePosition=[0, 0, 0],
                      baseVisualShapeIndex=visualShapeId,
                      baseCollisionShapeIndex=collisionShapeId,
                      basePosition=position)

    return objectId


def update_camera_list():
    cameras = get_cameras()

    device_menu.delete(0, device_menu.index("end"))

    for k, v in cameras.items():
        device_menu.add_radiobutton(label=k, value=v, variable=camera, command=change_camera)


def set_camera_parameter(key, value, device):
    device = "".join([s for s in device if s.isdigit()])
    stream = os.popen("v4l2-ctl -d {} --set-ctrl {}={}".format(device, key, value))
    op = stream.read()
    if len(op) > 1:
        raise Exception(op)


def change_camera():
    global capturing, camera, vid_cap_thread
    capturing = False
    #vid_cap_thread.join()
    time.sleep(0.1)
    path = camera.get()
    no = "".join([b for b in path if b.isdigit()])

    vid_cap_thread = threading.Thread(target=vid_cap_thread_func, args=(int(no),))
    vid_cap_thread.start()


def print_properties(cap):
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    buffer_size = cap.get(cv2.CAP_PROP_BUFFERSIZE)
    printc(width, height, fps, buffer_size)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def set_good_parameters():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 60)

    print_properties()


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
    point[0][0] += 0.28
    return point


def do_recording(e):
    global out, recording, record_button, recording_name, trajectories

    if not recording:
        for k in trajectories.keys():
            trajectories[k].clear()
        record_start_time = time.strftime("%Y%m%d-%H%M%S")
        recording_name = record_start_time + "-" + tkinter.simpledialog.askstring("Input name for file", "Name of recording:")
        os.mkdir("records/" + recording_name)
        record_button["text"] = "Stop recording (space)"
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter("records/" + recording_name + "/output.avi", fourcc, 30.0, (640, 480))
        recording = True
        threading.Thread(target=robo_record).start()
    else:
        recording = False
        time.sleep(0.1)
        out.release()
        json.dump(trajectories, open("records/" + recording_name + "/objects.json", "w"))
        record_button["text"] = "Start recording (space)"


def robo_record(offset=(0, 0, 0, 0, 0, 0)):
    global recording, recording_name

    print("Starting robo recording")

    gripper.open()

    sequence = []
    n = 0
    sequence.append([np.subtract(robot.getl(), offset).tolist(), 0, 0, time.time()])
    toc = time.time()
    tic = toc
    time.sleep(0.1)
    while recording and n < 3000:
        timestep = time.time() - tic
        sequence.append([np.subtract(robot.getl(), offset).tolist(), timestep, gripper.state, time.time()])
        tic = time.time()
        n += 1
        time.sleep(0.1)

    sequence.append([np.subtract(robot.getl(), offset).tolist(), time.time() - toc, 0, time.time()])
    print("recorded ", time.time() - toc, "secs")

    open("records/" + recording_name + "/arm_trajectory.json", "w").write(json.dumps(sequence))

    print("Wrote robo recording")


def robo_play():
    d = filedialog.askdirectory(title="Select recording directory", initialdir="records")

    if d == "":
        return

    sequence = json.load(open(d + "/arm_trajectory.json"))
    print(len(sequence))
    print("average timestep: ", sequence[-1][1] / (len(sequence) - 2))
    robot.movel(sequence[0][0])
    print("Made it past here!")
    toc = time.time()
    for i in range(1, len(sequence) - 1):
        print(i)
        robot.servoj_cart(sequence[i][0])
        time.sleep(0.1)

    robot.stopj()
    tic = time.time()

    print("recorded ", sequence[-1][1], "secs")
    print("executed in ", tic - toc, "secs")
    print("recorded end_pos: ", sequence[-1][0])
    print("actual end_pos:", robot.getl())


def draw_spheres(detections, trajectories):
    global last_known_location
    for k, v in trajectories.items():
        if k not in spheres:
            spheres[k] = create_sphere()
            print("Created sphere!")

        if k in last_known_location.keys():
            p.resetBasePositionAndOrientation(spheres[k], last_known_location[k], [0, 0, 0, 1])


def show_frame():
    """ Main loop """
    global cap, detections, transforms, translations, frame, last_error, last_known_location

    try:
        # time.sleep(0.2)
        tframe = frame

        gray_frame = cv2.cvtColor(tframe, cv2.COLOR_BGR2GRAY)

        detections = update_detection_data(at_detector.detect(gray_frame, estimate_tag_pose=True, camera_params=brio_params, tag_size=0.032, tag_size_overrides=tag_size_overrides), detections)

        if learning_mode.get():
            extract_all_transforms(detections)

        tframe = draw_outlines(detections, tframe)

        tframe = draw_debug_text(detections, tframe)

        if p.isConnected():
            draw_spheres(detections, trajectories)
            # p.removeAllUserDebugItems()

        if 0 in detections:
            tmp_t = {}
            for i in master_tags:
                tmp_t[i] = []

            for d in filter(lambda t: t.age <= 0, detections.values()):
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
                    last_known_location[k] = list(np.average(v, axis=0).flatten())
                    if recording:
                        trajectories[k].append([time.time(), last_known_location[k]])
                else:
                    if recording:
                        trajectories[k].append([time.time(), None])

        cv2image = cv2.cvtColor(tframe, cv2.COLOR_BGR2RGBA)

        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    except Exception as e:
        error = traceback.format_exc()
        if error != last_error:
            printc(error)
            last_error = error
        else:
            pass
            #printc(".", end="")
    lmain.after(10, show_frame)


def vid_cap_thread_func(cap_dev):
    global frame, recording, last_frame_time, capturing, cap, new_frame
    last_cap_error = ""
    printc("New recording thread for camera", cap_dev)
    capturing = True
    cap = cv2.VideoCapture(cap_dev, cv2.CAP_V4L)
    print_properties(cap)
    while capturing:
        try:
            ret, frame = cap.read()
            last_frame_time = cap.get(cv2.CAP_PROP_POS_MSEC)
            new_frame = True
            if recording:
                out.write(frame)
        except Exception as e:
            if e != last_cap_error:
                printc("Error in camera thread " + str(cap_dev) + ":", e)
            else:
                pass
    cap.release()
    del cap
    printc("Thread for camera", cap_dev, "ended")


def change_robot_ip():
    global ip
    dialog_input = tk.simpledialog.askstring(title="Robot configuration", prompt="Enter robot IP:", initialvalue=ip)

    if dialog_input is not None:
        ip = dialog_input


class HoldButton(tk.Button):
    def __init__(self, master, ondown=None, onup=None, key=None, tooltip=None, debounce=False, **kwargs):
        super().__init__(master, **kwargs)
        self.bind('<Button-1>', ondown)
        self.bind('<ButtonRelease-1>', onup)
        self.down = False
        self.last_down = 0
        self.debounce = debounce

        if key is not None:
            print("Binding", key)
            master.nametowidget(self.winfo_toplevel()).bind("<KeyPress-{}>".format(key), self.sim_down)
            master.nametowidget(self.winfo_toplevel()).bind("<KeyRelease-{}>".format(key), self.sim_up)
            print()

        if tooltip is not None:
            CreateToolTip(self, tooltip)

    def sim_down(self, e):
        self.last_down = time.time()
        if not self.down:
            self.down = True
            self.event_generate("<Button-1>")

    def sim_up(self, e):
        if self.debounce:
            self.after(200, self.check_last_down)
        else:
            self.down = False
            self.event_generate("<ButtonRelease-1>")

    def check_last_down(self):
        if self.down and time.time() - self.last_down > 0.1:
            self.down = False
            self.event_generate("<ButtonRelease-1>")


class CreateToolTip(object):
    """
    create a tooltip for a given widget
    """
    def __init__(self, widget, text='widget info'):
        self.waittime = 500     #miliseconds
        self.wraplength = 180   #pixels
        self.widget = widget
        self.text = text
        self.widget.bind("<Enter>", self.enter)
        self.widget.bind("<Leave>", self.leave)
        self.widget.bind("<ButtonPress>", self.leave)
        self.id = None
        self.tw = None

    def enter(self, event=None):
        self.schedule()

    def leave(self, event=None):
        self.unschedule()
        self.hidetip()

    def schedule(self):
        self.unschedule()
        self.id = self.widget.after(self.waittime, self.showtip)

    def unschedule(self):
        id = self.id
        self.id = None
        if id:
            self.widget.after_cancel(id)

    def showtip(self, event=None):
        x = y = 0
        x, y, cx, cy = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 25
        y += self.widget.winfo_rooty() + 20
        # creates a toplevel window
        self.tw = tk.Toplevel(self.widget)
        # Leaves only the label and removes the app window
        self.tw.wm_overrideredirect(True)
        self.tw.wm_geometry("+%d+%d" % (x, y))
        label = tk.Label(self.tw, text=self.text, justify='left',
                       background="#ffffff", relief='solid', borderwidth=1,
                       wraplength = self.wraplength)
        label.pack(ipadx=1)

    def hidetip(self):
        tw = self.tw
        self.tw= None
        if tw:
            tw.destroy()


def robot_control_dialog():
    global record_button
    newWindow = tk.Toplevel(window)
    newWindow.title("Robot control")

    newFrame = tk.Frame(newWindow)
    newFrame.grid(row=0, column=0, padx=20, pady=20)

    z_minus = HoldButton(newFrame, key="q", text="Z-", ondown=lambda e: set_robot_speed(2, -1), onup=lambda e: set_robot_speed(2, 0), tooltip="Z- (Q)")
    y_plus = HoldButton(newFrame, key="w", text="Y+", ondown=lambda e: set_robot_speed(1, 1), onup=lambda e: set_robot_speed(1, 0), tooltip="Y+ (W)")
    z_plus = HoldButton(newFrame, key="e", text="Z+", ondown=lambda e: set_robot_speed(2, 1), onup=lambda e: set_robot_speed(2, 0), tooltip="Z+ (E)")
    x_minus = HoldButton(newFrame, key="a", text="X-", ondown=lambda e: set_robot_speed(0, -1), onup=lambda e: set_robot_speed(0, 0), tooltip="X- (A)")
    y_minus = HoldButton(newFrame, key="s", text="Y-", ondown=lambda e: set_robot_speed(1, -1), onup=lambda e: set_robot_speed(1, 0), tooltip="Y- (S)")
    x_plus = HoldButton(newFrame, key="d", text="X+", ondown=lambda e: set_robot_speed(0, 1), onup=lambda e: set_robot_speed(0, 0), tooltip="X+ (D)")

    z_minus.grid(column=0, row=0, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    y_plus.grid(column=1, row=0, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    z_plus.grid(column=2, row=0, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    x_minus.grid(column=0, row=1, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    y_minus.grid(column=1, row=1, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    x_plus.grid(column=2, row=1, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)

    HoldButton(newFrame, text="Toggle freedrive (f)", key="f", ondown=lambda e: robot.toggle_freedrive()).grid(row=2, column=0, columnspan=3, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    HoldButton(newFrame, text="Toggle gripper (g)", key="g", ondown=lambda e: gripper.toggle()).grid(row=3, column=0, columnspan=3, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    record_button = HoldButton(newFrame, text="Start recording (space)", key="space", ondown=do_recording)
    record_button.grid(row=4, column=0, columnspan=3, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)
    HoldButton(newFrame, text="Play recording (p)", key="p", ondown=lambda e: robo_play()).grid(row=5, column=0, columnspan=3, sticky="nwes", ipadx=5, ipady=5, padx=5, pady=5)


def reconnect_robot():
    robot.reconnect()
    state_update_thread = threading.Thread(target=update_robot_state)
    state_update_thread.start()

robo_vel = [0, 0, 0, 0, 0, 0]
def set_robot_speed(direction, sign):
    global robo_vel

    move_speed = 0.1
    robo_vel[direction] = sign * move_speed
    robot.speedl(robo_vel, 1, 20)


def change_exposure():
    global exposure
    newWindow = tk.Toplevel(window)
    newWindow.title("Exposure adjustment")

    newFrame = tk.Frame(newWindow)
    newFrame.grid(row=0, column=0, padx=20, pady=20)

    tk.Label(newFrame, text="Slide to adjust camera exposure").pack()
    tk.Scale(newFrame, variable=exposure, from_=0, to=1000, tickinterval=100, length=500, orient=tk.HORIZONTAL, label="Exposure", command=lambda e: set_camera_parameter("exposure_absolute", exposure.get(), camera.get())).pack()

    printc("Changed")


def connect_robot():
    global robot, p

    robot = UR5SimSync(ip, use_rt=True)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create world
    create_box([0.3, 0.3, 0.5], [0, 0.15, 0.5])
    create_box([1.2, 0.3, 0.5], [0, -0.55, 0.5])
    create_box([1.2, 0.025, 0.7], [0, -0.85, 0.7])
    create_object("whisk.obj", [0, -0.55, 1.1])

    state_update_thread = threading.Thread(target=update_robot_state)
    state_update_thread.start()


def update_robot_state():
    global robot
    while True:
        robot.get_state()
        time.sleep(0.1)


def printc(*txt, end="\n"):
    #print(*txt)
    try:
        readonly.insert(tk.END, " ".join([str(t) for t in txt]) + end)
        readonly.yview(tk.END)
    except:
        pass


def readonly_event(event):
    if event.state & 4 == 4 and event.keysym == 'c':
        return
    else:
        return "break"


def run_command(e):
    printc(repr(eval(eval_str.get())))
    eval_str.set("")


robot = None
ip = "192.168.65.129"
capturing = True
last_error = ""
recording = False
frame = None
last_frame_time = None
new_frame = False

spheres = {}

gripper = Gripper(2, 1, virtual=True)
gripper.activate()

vid_cap_thread = threading.Thread(target=vid_cap_thread_func, args=(0,))
vid_cap_thread.start()

trajectories = {14: [], 36: []}
last_known_location = {}

# Set up GUI
window = tk.Tk()  # Makes main window
window.wm_title("Tracking")
window.config()

# Graphics window
imageFrame = tk.Frame(window, width=600, height=500)
imageFrame.grid(row=0, column=0, padx=10, pady=10, columnspan=2)

# Capture video frames
lmain = tk.Label(imageFrame)
lmain.grid(row=0, column=0)

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
sliderFrame.grid(row=1, column=0, padx=10, pady=10)

camera = tk.StringVar(value="/dev/video0")

cameras = get_cameras()

exposure = tk.IntVar()

readonly = tk.Text(sliderFrame, padx=20, pady=20, height=10)
readonly.configure(font="TkFixedFont")
readonly.bind("<Key>", readonly_event)
ys = tk.ttk.Scrollbar(sliderFrame, orient='vertical', command=readonly.yview)
readonly["yscrollcommand"] = ys.set
readonly.grid(column=0, row=0, sticky="nwes")
ys.grid(column=1, row=0, sticky="ns")
#readonly.pack()

tk.Label(sliderFrame, text="Evaluate:").grid(column=0, row=1, pady=10, sticky="w")
eval_str = tk.StringVar()
eval_prompt = tk.Entry(sliderFrame, textvariable=eval_str)
eval_prompt.bind("<Return>", run_command)
eval_prompt.grid(column=0, row=2, sticky="nwes")

menubar = tk.Menu(window)

filemenu = tk.Menu(menubar, tearoff=0)
menubar.add_cascade(label="File", menu=filemenu)

camera_menu = tk.Menu(menubar, tearoff=0)
menubar.add_cascade(label="Camera", menu=camera_menu)

robot_menu = tk.Menu(menubar, tearoff=0)
menubar.add_cascade(label="Robot", menu=robot_menu)

option_menu = tk.Menu(menubar, tearoff=0)
menubar.add_cascade(label="Options", menu=option_menu)

filemenu.add_separator()

filemenu.add_command(label="Exit", command=window.quit)

device_menu = tk.Menu(camera_menu, tearoff=0, postcommand=update_camera_list)

camera_menu.add_cascade(label="Camera device", menu=device_menu)

powerline_correction_menu = tk.Menu(camera_menu, tearoff=0)
camera_menu.add_cascade(label="Powerline correction", menu=powerline_correction_menu)

powerline_compensation = tk.StringVar()

powerline_correction_menu.add_radiobutton(label="None", value=0, variable=powerline_compensation, command=lambda: set_camera_parameter("power_line_frequency", 0, camera.get()))
powerline_correction_menu.add_radiobutton(label="50Hz", value=1, variable=powerline_compensation, command=lambda: set_camera_parameter("power_line_frequency", 1, camera.get()))
powerline_correction_menu.add_radiobutton(label="60Hz", value=2, variable=powerline_compensation, command=lambda: set_camera_parameter("power_line_frequency", 2, camera.get()))

manual_exposure = tk.IntVar()
camera_menu.add_checkbutton(label="Manual exposure", variable=manual_exposure, command=lambda: set_camera_parameter("exposure_auto", 1 if bool(manual_exposure.get()) else 3, camera.get()))
camera_menu.add_command(label="Adjust exposure", command=change_exposure)

robot_menu.add_command(label="Initialise", command=connect_robot)
robot_menu.add_command(label="Reonnect", command=reconnect_robot)
robot_menu.add_command(label="Control...", command=robot_control_dialog)

robot_menu.add_separator()

robot_menu.add_command(label="Configure...", command=change_robot_ip)

learning_mode = tk.IntVar(value=True)

option_menu.add_checkbutton(label="Learning mode", variable=learning_mode)

window.config(menu=menubar)

old_print = print

#print = printc

show_frame()

window.mainloop()
