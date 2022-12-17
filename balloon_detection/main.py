import utils.control as ctrl
import pickle
import socket
import struct

HOST = 'localhost'
PORT = 3001
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created")

s.bind((HOST,PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

conn, addr = s.accept()


# Set the PID gain
# print("Set pitch parameters:")
# kp = float(input("Kp of pitch: "))
# ki = float(input("Ki of pitch: "))
# kd = float(input("Kd of pitch: "))
pitch_PID = ctrl.PID(0, 0, 0)
# print("Set yaw parameters:")
# kp = float(input("Kp of yaw: "))
# ki = float(input("Ki of yaw: "))
# kd = float(input("Kd of yaw: "))
yaw_PID = ctrl.PID(0, 0, 0)

# Initialize the error status
pitch_PID.initialize()
yaw_PID.initialize()

xCenter = 320
yCenter = 240

while(True):
    # update the current position
    byte_data = conn.recv(4)
    xc = int.from_bytes(byte_data[:2], "little")
    yc = int.from_bytes(byte_data[2:4], "little")
    print([xc,yc])
    # calculate the error and PID control
    err = [xCenter-xc, yCenter-yc]
    s = "The error is: " + '(' + str(err[0]) + ',' + str(err[1]) + ')'
    print(s)
    pitchCommand = pitch_PID.update(err[0])
    yawCommand = yaw_PID.update(err[1]) 
