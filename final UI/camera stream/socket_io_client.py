import socketio
import json
import pickle
import base64

client = socketio.Client()



@client.event
def connect():
    print("I'm connected!")
@client.event
def connect_error(data):
    print("The connection failed!")
    print(data)
@client.event
def disconnect():
    print("I'm disconnected!")
    client.disconnect()

@client.on('fire_topic/ui_to_agv')
def on_message(data):
    print('I received a message!')
    print(data)

@client.on('mode_test')
def on_message(data):
    print('I received a message!')
    print(data)

@client.on('move_topic/ui_to_agv')
def on_message(data):
    print('I received a message!')
    print(data)

@client.on('orient_topic/ui_to_agv')
def on_message(data):
    print('I received a message!')
    print(data)

@client.on('mode_topic/ui_to_agv')
def on_message(data):
    print('I received a message!')
    print(data)

def SocketIO_Message_Publish(frame):
    # use dump() to write array into file
    imdata = pickle.dumps(frame)
    json_frame= json.dumps({"image": base64.b64encode(imdata).decode('ascii')})
    print(json_frame)
    client.emit("ui_to_robot/destination",json_frame)
   
client.connect('http://localhost:3001/socket.io')

if __name__ == '__main__':
    print("client running")