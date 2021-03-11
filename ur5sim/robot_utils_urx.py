import os
import time
import pdb
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
import time
import argparse
import sys
import logging
import numpy as np
from urx import Robot


class UR5SimSync(Robot):
    def __init__(self, ip, robot_urdf_path="./urdf/ur5.urdf", server_mode=p.GUI, robot_start_pos=(0, 0, 1), verbose=True, *args, **kwargs):
        self.state = None
        self.freedrive = False

        # --- connecting to GUI ---
        if verbose:
            print("-- Starting GUI --")
        self.serverMode = server_mode
        self.robotUrdfPath = robot_urdf_path
        self.physicsClient = p.connect(self.serverMode)  # connect to engine servers
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # add search path for loadURDF

        if verbose:
            print("-- Defining world --")
        # --- define world ---
        p.setGravity(0, 0, -9.81)
        self.world_plane = p.loadURDF("plane.urdf")

        # --- load robot in GUI ---
        self.__load_virtual_robot(robot_start_pos)

        self.ursim_ip = ip
        print("-- Synchronizing simulation to URSim --")

        p.addUserDebugLine([0, 1, 1], [0, 2, 1], [1, 0, 0], 10)
        p.addUserDebugLine([1, 0, 1], [2, 0, 1], [0, 1, 0], 10)

        try:
            super().__init__(ip, *args, **kwargs)
        except Exception as e:
            print("Connection error:", e)

    def __load_virtual_robot(self, robot_start_pos):
        self.sim_controlJoints = ["shoulder_pan_joint",
                              "shoulder_lift_joint",
                              "elbow_joint",
                              "wrist_1_joint",
                              "wrist_2_joint",
                              "wrist_3_joint"]
        self.sim_robotStartPos = robot_start_pos
        self.sim_startQuaternion = p.getQuaternionFromEuler([0, 0, 3.1416])
        self.sim_pybullet_robot = p.loadURDF(self.robotUrdfPath, self.sim_robotStartPos, self.sim_startQuaternion,
                                             flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=1)

        self.sim_joint_typeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.sim_numJoints = p.getNumJoints(self.sim_pybullet_robot)
        self.sim_jointInfo = namedtuple("jointInfo",
                                        ["id",
                                     "name",
                                     "type",
                                     "lowerLimit",
                                     "upperLimit",
                                     "maxForce",
                                     "maxVelocity",
                                     "controllable"])
        self.sim_joints = AttrDict()
        for i in range(self.sim_numJoints):
            info = p.getJointInfo(self.sim_pybullet_robot, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            joint_type = self.sim_joint_typeList[info[2]]
            joint_lower_limit = info[8]
            joint_upper_limit = info[9]
            joint_max_force = info[10]
            joint_max_velocity = info[11]
            controllable = True if joint_name in self.sim_controlJoints else False
            info = self.sim_jointInfo(joint_id, joint_name, joint_type, joint_lower_limit,
                                      joint_upper_limit, joint_max_force, joint_max_velocity, controllable)
            if info.type == "REVOLUTE":  # set revolute joint to static
                p.setJointMotorControl2(self.sim_pybullet_robot, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.sim_joints[info.name] = info

    def get_state(self):  # also updates simulation
        self.state = super().getj()

        try:
            for i in range(6):
                p.resetJointState(self.sim_pybullet_robot, i, self.state[i])

            p.stepSimulation()
            return self.state

        except Exception as e:
            p.disconnect()
            raise(e)

    def set_freedrive(self, val, timeout=60):
        self.freedrive = val
        super().set_freedrive(val, timeout)
    
    def toggle_freedrive(self):
        self.freedrive = not self.freedrive
        self.set_freedrive(self.freedrive)

    def reconnect(self, ip=None):
        if ip is None:
            super().__init__(self.ursim_ip)
        else:
            super().__init__(ip)

    def servoj_cart(self, pose, acc=0.25, vel=0.25, t=.1, lookhead_time=0.2, gain=200):
        x, y, z, rx, ry, rz = pose
        self.rtmon.send_program(
            "servoj(get_inverse_kin(p[{}, {}, {}, {}, {}, {}]), a={}, v={}, t={}, lookahead_time={}, gain={})\n".format(x, y, z, rx, ry, rz, acc, vel, t, lookhead_time, gain)
        )
