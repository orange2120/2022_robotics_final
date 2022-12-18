import sys
import pickle
import struct
import socket

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',3001))

def Socket_Message_Publish(frame):
    data = pickle.dumps(frame)
    message_size = struct.pack("L", len(data)) 
    print(type(message_size+data))
    clientsocket.sendall(message_size+data)

