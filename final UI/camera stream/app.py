from flask import Flask,render_template,Response
import cv2
# import socket_io_client

app=Flask(__name__)
class VideoCamera(object):
    def __init__(self):
        #由opencv來獲取預設為0 裝置影像
        self.video = cv2.VideoCapture(0)

    def __del__(self):
        self.video.release()        

    def get_frame(self):
        ret, frame = self.video.read()

        ret, jpeg = cv2.imencode('.jpg', frame)

        return jpeg.tobytes()

video_stream = VideoCamera()

def generate_frames(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        
    


@app.route('/members')
def index():
    return render_template('index.html')

@app.route('/video')
def video():
    return Response(generate_frames(video_stream),mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/flaskapi")
def FlasktoReact():
    return {
        "bullets":100,
        "battery":100,
        "velocity":300
    }


if __name__=="__main__":
    app.run(host='localhost', port=5000,debug=True)
    
    