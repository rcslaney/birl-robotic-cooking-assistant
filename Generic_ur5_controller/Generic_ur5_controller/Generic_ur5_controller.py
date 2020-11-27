import time
import serial
from math import pi
import numpy
import socket

import waypoints as wp
import kg_robot as kgr


def main():
    print("------------Configuring Burt-------------\r\n")
    burt = kgr.kg_robot(port=30010, db_host="192.168.1.6")
    #burt = kgr.kg_robot(port=30010,ee_port="COM32",db_host="192.168.1.51")
    print("----------------Hi Burt!-----------------\r\n\r\n")

    try:
        while 1:
            ipt = input("cmd: ")
            if ipt == 'close':
                break
            elif ipt == 'home':
                burt.home()
            elif ipt == 'rec':
                burt.teach_mode.record()
            elif ipt == 'play':
                burt.teach_mode.play(name="testRecording.json")
            elif ipt == 'vel':
                burt.speedl([0,0,-0.01,0,0,0],acc=0.1,blocking_time=5)
            elif ipt == "stop":
                burt.speedl([0, 0, 0, 0, 0, 0], acc=0.1, blocking_time=5)
            elif ipt == 'hi':
                burt.set_digital_out(0, 1)
            elif ipt == 'lo':
                burt.set_digital_out(0, 0)
            elif ipt == "z+":
                burt.translatel_rel([0, 0, 0.01])
            elif ipt == "z-":
                burt.translatel_rel([0, 0, -0.01])
            elif ipt == "y+":
                burt.translatel_rel([0, 0.05, 0])
            elif ipt == "y-":
                burt.translatel_rel([0, -0.05, 0])
            elif ipt == "x+":
                burt.translatel_rel([0.05, 0, 0])
            elif ipt == "x-":
                burt.translatel_rel([-0.05, 0, 0])
            elif ipt == "test":
                burt.movel()
            elif ipt == "myhome":
                burt.movel([0.125, 0.3, 0.4, 3, 0.9, 0])
            # else:
            #     var = int(input("var: "))
            #     burt.serial_send(ipt,var,True)

            print(burt.getl())

        
    finally:
        print("Goodbye")
        burt.close()
if __name__ == '__main__': main()
