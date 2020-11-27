import os
from dynamixel_sdk import *  # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty

    fd = sys.stdin.fileno()


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            pass
        return ch

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE = 1000000  # Dynamixel default baudrate : 57600

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

# Control table address
ADDR_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION = 30
ADDR_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40


class AX12controller:
    def __init__(self, device_name):
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def init_communication(self):
        # Open port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        # Set port baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        return None

    def close_port(self):
        self.port_handler.closePort()

        return None

    def ping(self, ax_id):
        dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, ax_id)
        if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            #print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            return False
        else:
            #print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (ax_id, dxl_model_number))
            return True

    def enable_torque(self, ax_id):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, ax_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            return False
        else:
            print("Dynamixel has been successfully connected")

        return True

    def write_position(self, ax_id, position):
        position = round(1024 * position / 300)

        if position > 1023:
            position = 1023
        elif position < 0:
            position = 0

        # Write goal position
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, ax_id, ADDR_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("write success")
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:

            print(dxl_error)
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

        return position

    def write_speed(self, ax_id, speed):
        # Write goal position
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, ax_id, ADDR_SPEED, speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("write success")
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:

            print(dxl_error)
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        return speed

    def read_position(self, ax_id):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, ax_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

        print("[ID:%03d]  PresPos:%03d" % (ax_id, dxl_present_position))
        return dxl_present_position

    def read_load(self, ax_id):
        return self.load(ax_id, ADDR_PRESENT_LOAD)

    def load(self, ax_id, addr):
        dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, ax_id, addr)
        if dxl_comm_result != COMM_SUCCESS:
            raise("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            raise("%s" % self.packet_handler.getRxPacketError(dxl_error))

        return dxl_data


