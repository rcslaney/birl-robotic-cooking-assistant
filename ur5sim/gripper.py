from ax12 import AX12controller

class Gripper:
    def __init__(self, id1, id2, ttyPort="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M4HIL-if00-port0"):
        self.id1 = id1
        self.id2 = id2

        self.controller = AX12controller(ttyPort)
        self.controller.init_communication()

        self.id1_open_position = 0
        self.id2_open_position = 300

        self.close_degrees = 70

        self.state = 0

    def activate(self):
        self.controller.enable_torque(self.id1)
        self.controller.enable_torque(self.id2)

    def is_connected(self):
        return self.controller.ping(self.id1) and self.controller.ping(self.id2)

    def close(self):
        self.controller.write_position(self.id1, self.id1_open_position + self.close_degrees)
        self.controller.write_position(self.id2, self.id2_open_position - self.close_degrees)
        self.state = 1

    def open(self):
        self.controller.write_position(self.id1, self.id1_open_position)
        self.controller.write_position(self.id2, self.id2_open_position)
        self.state = 0

    def toggle(self):
        if self.state:
            self.open()
        else:
            self.close()

    def set_state(self, state):
        if state:
            self.close()
        else:
            self.open()