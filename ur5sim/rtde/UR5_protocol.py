import socket
import time
import argparse
import sys
import logging
import numpy as np

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


def encode(string):
    return string.encode()


class UR5:
    host = None
    data_port_number = None
    control_port_number = None
    rtde = None          # object to retrieve data from robot - port 30004
    ur5_control = None   # object to control robot            - port 30003
    conf = None
    watchdog = None

    def __init__(self, robot_ip="192.168.1.4", control_port_number=30003, data_port_number=30004):
        self.host = robot_ip
        self.control_port_number = control_port_number
        self.data_port_number = data_port_number
        self._enstablish_connection()

    def _enstablish_connection(self):
        # enstablish control connection
        self.ur5_control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ur5_control.settimeout(10)
        self.ur5_control.connect((self.host, self.control_port_number))
        time.sleep(1.00)
        self.ur5_control.send(b"set_digital_out(2, True)" + b"\n")
        time.sleep(.1)
        self.ur5_control.send(b"set_digital_out(7, True)" + b"\n")
        time.sleep(1.00)
        print("CONTROL CONNECTION TO UR5 ESTABLISHED...")

        # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # s.settimeout(10)
        # s.connect((self.host, self.data_port_number))
        # time.sleep(1.00)
        # print("DATA CONNECTION TO UR5 ESTABLISHED...")

        # enstablish Dashboard Server connection
        #"unlock protective stop" 29999

        self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dashboard.settimeout(10)
        self.dashboard.connect((self.host, 29999))
        time.sleep(1.00)
        print("UR5 DASHBOARD SERVER CONNECTION ESTABLISHED...")
        self.dashboard.send(encode("unlock protective stop\n"))
        print("protective stop unlocked")

        # parameters
        parser = argparse.ArgumentParser()
        parser.add_argument('--host', default='localhost', help='name of host to connect to (localhost)')
        parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
        parser.add_argument('--samples', type=int, default=0, help='number of samples to record')
        parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
        parser.add_argument('--config', default='rtde/UR5_config.xml',
                            help='data configuration file to use (record_configuration.xml)')
        parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
        args = parser.parse_args()

        if args.verbose:
            logging.basicConfig(level=logging.INFO)

        self.conf = rtde_config.ConfigFile(args.config)
        state_names, state_types = self.conf.get_recipe('out')
        watchdog_names, watchdog_types = self.conf.get_recipe('watchdog')

        # enstablish data connection
        self.rtde = rtde.RTDE(self.host, self.data_port_number)
        self.rtde.connect()
        # get controller version
        self.rtde.get_controller_version()
        # setup recipes
        if not self.rtde.send_output_setup(state_names, state_types, frequency=args.frequency):
            logging.error('Unable to configure output')
            sys.exit()
        self.watchdog = self.rtde.send_input_setup(watchdog_names, watchdog_types)
        print("DATA CONNECTION TO UR5 ESTABLISHED...")

        # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
        self.watchdog.input_int_register_0 = 0

        # start data synchronization
        if not self.rtde.send_start():
            sys.exit()

    def get_state(self, param=None):
        state = self.rtde.receive()
        if state is None:
            print("WARNING: Connection to robot lost... attepting to reconnect...")
            time.sleep(2)
            self._enstablish_connection()
            print("Ready, resuming experiments...")
            time.sleep(2)
            return None
        else:
            if param:
                return state.__dict__[param]
            return state.__dict__

    # NOTE: x,y,z in meters and rx, ry, rz in degrees
    def cart_go_to(self, pose, acc=1.4, vel=1.05, t=0, bland_radius=0):
        x, y, z, rx, ry, rz = pose
        self.ur5_control.send(encode("movej(p[" +
                              str(x) + ", " +
                              str(y) + ", " +
                              str(z) + ", " +
                              str(rx) + ", " +
                              str(ry) + ", " +
                              str(rz) + "], " +
                              "a=" + str(acc) +
                              ", " + "v=" + str(vel) +
                              (", " + "t=" + str(t) if t != 0 else '') +
                              (", " + "r=" + str(bland_radius) if bland_radius != 0 else '') + ")" + "\n"))
        return True

    def set_tcp(self, pose):
        self.ur5_control.send(encode("set_tcp([" +
                                     str(pose[0]) + ", " +
                                     str(pose[1]) + ", " +
                                     str(pose[2]) + ", " +
                                     str(pose[3]) + ", " +
                                     str(pose[4]) + ", " +
                                     str(pose[5]) + "]\n"))
        return True

    # NOTE: axis 0=x 1=y z=2
    def set_speed(self, axis=2, acc=1.4, vel=1.05, t=0, tool_acceleration=0):
        if axis == 'all' and (isinstance(vel, list) or isinstance(vel, np.ndarray)):
            speed_to_reach = vel
        elif (axis == 'all' or isinstance(axis, list)) and not (isinstance(vel, list) or isinstance(vel, np.ndarray)):
            speed_to_reach = np.zeros(6)
            speed_to_reach = np.array(speed_to_reach)
            speed_to_reach[axis] = vel
        else:
            speed_to_reach = np.zeros(6)
            speed_to_reach[axis] = vel
        self.ur5_control.send(encode("speedl([" +
                                     str(speed_to_reach[0]) + ", " +
                                     str(speed_to_reach[1]) + ", " +
                                     str(speed_to_reach[2]) + ", " +
                                     str(speed_to_reach[3]) + ", " +
                                     str(speed_to_reach[4]) + ", " +
                                     str(speed_to_reach[5]) + "], " +
                                     "a=" + str(acc) +
                                     (", " + "t=" + str(t) if t != 0 else '') +
                                     (", " + "aRot=" + str(tool_acceleration) if tool_acceleration != 0 else '') + ")" +
                                     "\n"))
        return True

    def joint_go_to(self, pose, acc=1.4, vel=.8, t=0, bland_radius=0):
        x, y, z, rx, ry, rz = pose
        self.ur5_control.send(encode("movej([" +
                              str(x) + ", " +
                              str(y) + ", " +
                              str(z) + ", " +
                              str(rx) + ", " +
                              str(ry) + ", " +
                              str(rz) + "], " +
                              "a=" + str(acc) +
                              ", " + "v=" + str(vel) +
                              (", " + "t=" + str(t) if t != 0 else '') +
                              (", " + "r=" + str(bland_radius) if bland_radius != 0 else '') + ")" + "\n"))
        return True

    def servoj(self, pose, acc=1.4, vel=.8, t=0, bland_radius=0):
        j0, j1, j2, j3, j4, j5 = pose
        self.ur5_control.send(encode("servoj([" +
                              str(j0) + ", " +
                              str(j1) + ", " +
                              str(j2) + ", " +
                              str(j3) + ", " +
                              str(j4) + ", " +
                              str(j5) + "], " +
                              "a=" + str(acc) +
                              ", " + "v=" + str(vel) +
                              (", " + "t=" + str(t) if t != 0 else '') +
                              (", " + "r=" + str(bland_radius) if bland_radius != 0 else '') + ")" + "\n"))
        return True

    def servoc(self, pose, acc=1.4, vel=.8, t=0, bland_radius=0):
        j0, j1, j2, j3, j4, j5 = pose
        self.ur5_control.send(encode("servoc([" +
                              str(j0) + ", " +
                              str(j1) + ", " +
                              str(j2) + ", " +
                              str(j3) + ", " +
                              str(j4) + ", " +
                              str(j5) + "], " +
                              "a=" + str(acc) +
                              ", " + "v=" + str(vel) +
                              (", " + "t=" + str(t) if t != 0 else '') +
                              (", " + "r=" + str(bland_radius) if bland_radius != 0 else '') + ")" + "\n"))
        return True

    def joint_stop(self, acc=3):
        self.ur5_control.send(encode("stopj({})\n".format(acc)))

    # move linear in tool space
    def lin_go_to(self, pose, acc=1.2, vel=0.25, t=0, bland_radius=0):
        x, y, z, rx, ry, rz = pose
        self.ur5_control.send(encode("movel([" +
                              str(x) + ", " +
                              str(y) + ", " +
                              str(z) + ", " +
                              str(rx) + ", " +
                              str(ry) + ", " +
                              str(rz) + "], " +
                              "a=" + str(acc) +
                              ", " + "v=" + str(vel) +
                              (", " + "t=" + str(t) if time != 0 else '') +
                              (", " + "r=" + str(bland_radius) if time != 0 else '') + ")" + "\n"))
        return True

    # Blend circular (in tool-space) and move linear (in tool-space) to
    # position. Accelerates to and moves with constant tool speed v.
    def process_go_to(self, pose, acc=1.2, vel=0.25, blend_radius=0):
        x, y, z, rx, ry, rz = pose
        self.ur5_control.send(encode("movej(p[" +
                              str(x) + ", " +
                              str(y) + ", " +
                              str(z) + ", " +
                              str(rx) + ", " +
                              str(ry) + ", " +
                              str(rz) + "], "+
                              "a=" + str(acc) + ", " +
                              "v=" + str(vel) + ", " +
                              "r=" + str(blend_radius) + ")" + "\n"))
        return True

    # Servo to position (circular in tool-space). Accelerates to and moves
    # with constant tool speed v.
    def circular_go_to(self, pose, acc=1, vel=0.2, blend_radius=0):
        x, y, z, rx, ry, rz = pose
        self.ur5_control.send(encode("movec([" +
                              str(x) + ", " +
                              str(y) + ", " +
                              str(z) + ", " +
                              str(rx) + ", " +
                              str(ry) + ", " +
                              str(rz) + "], "+
                              "a=" + str(acc) + ", " +
                              "v=" + str(vel) + ", " +
                              "r=" + str(blend_radius) + ")" + "\n"))
        return True

    def reached_point(self, point=None, mode='pose', radius=0.0005, state=None):
        # Get current position
        if mode == 'pose':
            if state is None:
                pose = np.array(self.get_state('actual_TCP_pose'))[:3]
            else:
                pose = np.array(state['actual_TCP_pose'])[:3]
        elif mode == 'joint':
            if state is None:
                pose = np.array(self.get_state('actual_q'))[:3]
            else:
                pose = np.array(state['actual_q'])[:3]
            radius = radius*10
        else:
            raise NotImplementedError('mode {} not implemented!'.format(mode))

        # Get position to reach
        if point is None:
            if state is None:
                target = np.array(self.get_state('target_TCP_pose'))[:3]
            else:
                target = np.array(state['target_TCP_pose'])[:3]
        else:
            target = np.array(point)[:3]

        # Check distance
        if np.sqrt(np.sum((pose-target)**2)) < radius:
            return True
        return False

    def is_moving(self, thresh=.0004):
        speeds = self.get_state('actual_TCP_speed')
        if speeds is not None:
            return np.any(np.abs(speeds) > thresh)
        else:
            return None

    def pause(self):
        self.rtde.send_pause()

    def start(self):
        self.rtde.send_start()

    def kick_watchdog(self):
        self.rtde.send(self.watchdog)

    def set_host(self, robot_ip):
        self.host = robot_ip

    def set_data_port(self, port_number):
        self.data_port_number = port_number

    def set_control_port(self, port_number):
        self.control_port_number = port_number

    def get_control_port(self):
        return self.control_port_number

    def get_host(self):
        return self.host

    def get_data_port(self):
        return self.data_port_number

    def get_rtde(self):
        return self.rtde

    def get_watchdog(self):
        return self.watchdog

    def disconnect(self):
        print("disconnecting...")
        time.sleep(2)
        self.ur5_control.send(b'set_digital_out(2, False)' + b'\n')
        time.sleep(.1)
        self.ur5_control.send(b'set_digital_out(7, False)' + b'\n')
        self.rtde.send_pause()
        self.rtde.disconnect()
        self.ur5_control.close()

    def custom_socket_send(self, prog):
        msg = "No message from robot"
        try:
            # Send formatted CMD
            self.ur5_control.send(str.encode(prog))
            # Wait for reply
            if prog[-3]=='0':
                msg = bytes.decode(self.ur5_control.recv(1024))
                if msg == "No message from robot" or msg == '':
                    print(".......................Robot disconnected :O.......................")
                    input("press enter to continue")

        except socket.error as socketerror:
            print(".......................Some kind of error :(.......................")
            input("press enter to continue")
        return msg

#     def myMoveJ(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
#         """
#         joint move in linear space
#         """
#         prog = format_prog(1, pose=pose, acc=acc, vel=vel, t=min_time, r=radius, w=wait)
#         return self.custom_socket_send(prog)
#
#
# def format_prog(CMD, pose=[0,0,0,0,0,0], acc=0.1, vel=0.1 , t=0, r=0, w=True):
#     wait = 0
#     if w is False:
#         wait = 1
#     return "({},{},{},{},{},{},{},{},{},{},{},{})\n".format(CMD, *pose, acc, vel, t, r, wait)
