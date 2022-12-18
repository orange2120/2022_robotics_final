import sys
import pickle
import struct
import socket

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',4000))

def Socket_Message_Publish(message):
    
    print(message)
    clientsocket.send(message)


if __name__ == '__main__':
    print("client running")