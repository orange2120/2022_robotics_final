import cv2
import numpy as np
# from socket_client import Socket_Message_Publish


def Camera():
    cap = cv2.VideoCapture(700)
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        ret,frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Socket_Message_Publish(frame)
        cv2.imshow('My Image', frame)
        
        c = cv2.waitKey(1)
        if c == 27:
            break
        
    cap.release(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    Camera()
    
    
    