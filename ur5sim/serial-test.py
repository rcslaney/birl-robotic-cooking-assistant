import serial

ser = serial.Serial("/dev/ttyACM0")

toggle = False

while True:
    inp = input("Press senter to change state...")
    toggle = not toggle
    if toggle:
        ser.write(b"80,115,")  # Close
    else:
        ser.write(b"180,20,")  # Open
