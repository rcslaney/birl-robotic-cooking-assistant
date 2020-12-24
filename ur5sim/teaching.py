from robot_utils import UR5
import _thread
import time
import json
from gripper import Gripper
import numpy as np

gripper = Gripper(2, 1)

robot = UR5(robot_ip="192.168.1.6")

freedrive = False
thread_flag = False


def wait_for_enter():
    print("Starting wait?")
    _thread.start_new_thread(wait_for_enter_thread, ())
    return


def wait_for_enter_thread():
    global gripper, thread_flag
    print("Started wait for enter...")
    thread_flag = True
    while True:
        c = input()
        if c == "t":
            gripper.toggle()
        else:
            break
    thread_flag = False
    return


def freedrive_start():
    global freedrive
    freedrive = True
    _thread.start_new_thread(freedrive_loop, ())
    return


def freedrive_stop():
    global freedrive
    freedrive = False


def freedrive_loop():
    print("Started freedrive!")
    while True:
        global freedrive
        robot.freedrive()
        time.sleep(0.05)
        if not freedrive:
            print("Ended freedrive!")
            break


def record(name, offset=(0, 0, 0, 0, 0, 0)):
    gripper.open()
    input("Press enter to start and stop recording")

    sequence = []
    n = 0
    sequence.append([np.subtract(robot.get_state()["actual_TCP_pose"], offset).tolist(), 0, 0])
    time.sleep(2)
    freedrive_start()
    wait_for_enter()
    toc = time.time()
    tic = toc
    time.sleep(0.1)
    while thread_flag and n < 3000:
        timestep = time.time() - tic
        sequence.append([np.subtract(robot.get_state()["actual_TCP_pose"], offset).tolist(), timestep, gripper.state])
        tic = time.time()
        n += 1
        time.sleep(0.1)

    freedrive_stop()
    sequence.append([np.subtract(robot.get_state()["actual_TCP_pose"], offset).tolist(), time.time() - toc, 0])
    print("recorded ", time.time() - toc, "secs")

    name += ".json"
    open(name, "w").write(json.dumps(sequence))


def play(filename):
    sequence = json.load(open(filename))
    print(len(sequence))
    print("average timestep: ", sequence[-1][1] / (len(sequence) - 2))
    robot.lin_go_to(sequence[0][0])
    toc = time.time()
    for i in range(1, len(sequence) - 1):
        robot.servoj_cart(sequence[i][0])

    robot.joint_stop()
    tic = time.time()

    print("recorded ", sequence[-1][1], "secs")
    print("executed in ", tic - toc, "secs")
    print("recorded end_pos: ", sequence[-1][0])
    print("actual end_pos:", robot.get_state()["actual_TCP_pose"])


#record("arm_trajectory_" + str(round(time.time())))
#input("Ready to play?")
play("records/20201217-171704/arm_trajectory.json")

# [-0.307212, -0.388019, 0.290724, -1.22996, -1.0794, 1.00604] Kierans
# [-0.3072148632635549, -0.38798823416280576, 0.2907236202218176, -1.2298626399827235, -1.0794722257721994, 1.0058524346469067]
