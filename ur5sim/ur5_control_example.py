# UR5 example run with RTDE and URSim + custom PyBullet
# Author: Luca Scimeca

from rtde.UR5_protocol import UR5
from robot_utils import UR5SimSync
import time
import numpy as np
import matplotlib.pyplot as plt


# these you can read in the join position on the teachpad once the robot reaches the position
example_joint_start = np.deg2rad([85.12, -78.13, 121.96, 226.76, -87.81, -3.34])


if __name__ == "__main__":

    try:
        # initialize robot class. This class abstracts away all the complexity of using RTDE.
        # the IP needs to match the robot ip, set in the robot configurations through the touchpad
        # read its code to see how you can control the robot.
        # The class UR5SimSync can be swapped for the class UR5, and a real robot can be controlled instead
        # (with the same code)
        robot = UR5SimSync(robot_ip="192.168.56.101")

        print("Going to start position...")
        # go-to joint position with set acceleration and velocity
        robot.joint_go_to(example_joint_start, acc=.5, vel=1)

        # keep checking the robot state during the execution of the motion, and stops robot when the position is reached.
        # if this is not done the robot will stop automatically once it reaches the position. However, if a new command
        # is sent, the new command overrides the previous command.
        while not robot.reached_point(point=example_joint_start, mode='joint'): pass
        print("Start position reached!")

        # pull the robot state through the data
        state = robot.get_state()

        if state is None:
            raise EnvironmentError("the robot state cannot be read!")

        start_time = time.time()
        counter = 0
        xs = []
        ys = []

        # for 10 seconds
        while time.time() - start_time < 20:
            current_time = time.time()
            # get pose of end effector, format [x, y, z, rx, ry, rz]
            current_pose = robot.get_state('actual_TCP_pose')
            # save the robot x-y position every time
            xs += [current_pose[0]]
            ys += [current_pose[1]]

            # lets make the robot do a circle about 5.5cm (0.055m) in radius, and in about 4 seconds (2. x2)
            # also derivative of sin/cos funct. in x and y to achieve circle of course.
            x = -0.2*2.*np.sin(2.*current_time)
            y = -0.2*2.*np.cos(2.*current_time)

            # lets control the robot with speed profiles, so the motion is smooth and continuous.
            # the speeds will be in [x, y, z, rx, ry, rz], and remember distances in the robots are in meters!
            speed_vector = [x, y, 0, 0, 0, 0]
            # set robot speeds on all axis
            robot.set_speed(axis='all', vel=speed_vector, acc=2., t=10)

            # good practice to do this each iteration
            robot.kick_watchdog()

            # count iterations
            counter += 1

        print("Average control frame rate during the experiment was {}frames per sec".format(counter/10))

        # return to the start position
        robot.joint_go_to(example_joint_start, acc=.5, vel=1)
        while not robot.reached_point(point=example_joint_start, mode='joint'): pass

        while True: robot.get_state("actual_TCP_pose")

        # plot where the robot has been during those 10 seconds
        fig = plt.figure()
        plt.plot(xs, ys)
        plt.show()



    finally:
        robot.disconnect()
