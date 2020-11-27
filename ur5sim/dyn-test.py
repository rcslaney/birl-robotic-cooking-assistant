from ax12 import AX12controller
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from multiprocessing import Process, Lock
from gripper import Gripper

ax12_lock = Lock()

def load_plotting():
    global plt

    loads = []

    fig = plt.figure()

    def animate(i):
        global ax12, plt

        ax12_lock.acquire()
        loads.append(ax12.read_load(1))
        ax12_lock.release()
        fig.clear()
        plt.plot(loads)

    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()


ttyPort = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M4HIL-if00-port0"

gripper = Gripper(2, 1)

print("Test:", gripper.is_connected())

gripper.activate()

toggle = False

while True:
    command = input("Press enter to toggle or enter a command:")
    toggle = not toggle

    if command == "load":
        test = Process(target=load_plotting)
        test.start()
    else:
        ax12_lock.acquire()
        gripper.toggle()
        ax12_lock.release()