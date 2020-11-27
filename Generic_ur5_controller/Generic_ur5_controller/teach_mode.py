import numpy as np
import time
import serial
import socket
import math
import json
import _thread
from math import pi

thread_flag = False
timer_flag = 0

def wait_for_enter():
    global thread_flag
    thread_flag = True
    input()
    thread_flag = False
    return

class teach_mode():
    def __init__(self,robot):
        self.name = "test.json"
        self.robot = robot

    def record(self,name=""):
        input("press enter to start and stop recording")
        global urnie_flag
        _thread.start_new_thread(wait_for_enter,())
        sequence = []
        n = 0
        sequence.append([self.robot.getl(),0])
        time.sleep(2)
        self.robot.socket_send(self.robot.format_prog(30))
        toc = time.time()
        tic = toc
        time.sleep(0.1)
        while(thread_flag == True and n < 3000):
            timestep = time.time()-tic
            sequence.append([self.robot.getl(),timestep])
            tic = time.time()
            n+=1
            time.sleep(0.1)

        self.robot.socket_send(self.robot.format_prog(31))
        sequence.append([self.robot.getl(),time.time()-toc])
        print("recorded ",time.time()-toc,"secs")

        if(name==""):
            self.name = input('Sequence captured\r\nenter name: ')
        else:
            self.name = name
        self.name+=".json"
        open(self.name, "w").write(json.dumps(sequence))

    def play(self, name=""):
        if name == "":
            name = self.name
        sequence = json.load(open(name))
        print(len(sequence))
        print("average timestep: ",sequence[-1][1]/(len(sequence)-2))
        self.robot.movel(sequence[0][0])
        toc = time.time()
        for i in range(1,len(sequence)-1):
            self.robot.servoj(sequence[i][0],control_time=sequence[i][1],lookahead_time=0.008,gain=300)

        self.robot.stopl(0.5)
        tic = time.time()
        self.robot.socket_ping()
        print("recorded ",sequence[-1][1],"secs")
        print("executed in ",tic-toc,"secs")
        print("recorded end_pos: ",sequence[-1][0])
        print("actual end_pos:",self.robot.getl())

    def seq_print(self,name = ""):
        if name == "":
            name = self.name
        sequence = json.load(open(name))
        for i in range(0,len(sequence)):
            print(sequence[i])
        print(len(sequence))

