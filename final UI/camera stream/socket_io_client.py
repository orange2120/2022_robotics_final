import socketio
import numpy as np
from socket_client_move import  Socket_Message_Publish 

client = socketio.Client()

rc_val = [984,999,999,988,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000]


@client.event
def connect():
    print("I'm connected!")
@client.event
def connect_error(data):
    #print("The connection failed!")
    print(data)
@client.event
def disconnect():
    #print("I'm disconnected!")
    client.disconnect()

@client.on('fire_topic/ui_to_agv')
def on_message(data):
    Socket_Message_Publish(data)

@client.on('move_topic/ui_to_agv')
def on_message(data):
    #print('I received a message!')
    rc_val[1] = data
    message = np.array(rc_val,dtype='<u2').tobytes()
    Socket_Message_Publish(message)
    

@client.on('turn_topic/ui_to_agv')
def on_message(data):
    rc_val[0] = data
    message = np.array(rc_val,dtype='<u2').tobytes()
    Socket_Message_Publish(message)
    

@client.on('fire_topic/ui_to_agv')
def on_message(data):
    rc_val[9] = data
    message = np.array(rc_val,dtype='<u2').tobytes()
    Socket_Message_Publish(message)
    
    

@client.on('orient_topic/ui_to_agv')
def on_message(data):
    rc_val[6] = data
    message = np.array(rc_val,dtype='<u2').tobytes()
    Socket_Message_Publish(message)
    
    

@client.on('shift_topic/ui_to_agv')
def on_message(data):
    rc_val[3] = data
    message = np.array(rc_val,dtype='<u2').tobytes()
    Socket_Message_Publish(message)
    

   
client.connect('http://localhost:3001/socket.io')

if __name__ == '__main__':
    print("client running")