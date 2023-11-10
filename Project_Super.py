#Importing OpenCV Library for basic image processing functions
import cv2
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
import face_recognition
#face_utils for basic operations of conversion
from imutils import face_utils
from threading import Thread
import argparse
import imutils
import time
import dlib
import cv2
import os
from flask import Flask,render_template,Response
import jsonify
import json, os, signal


app=Flask(__name__)


alarm_status = False
alarm_status2 = False
alarm_status3=False
saying = False
def alarm(msg):
    global alarm_status
    global alarm_status2
    global alarm_status3
    """global face_dist"""
    global saying
    while alarm_status:
        s = 'espeak "'+msg+'"'
        os.system(s)

    if alarm_status2:
        saying = True
        s = 'espeak "' + msg + '"'
        os.system(s)
        saying = False
    if alarm_status3:
        saying = True
        s= 'espeak "' + msg + '"'
        os.system(s)

def compute2(pta, ptb):
    dist =np.linalg.norm(pta - ptb)
    return dist
#Initializing the camera and taking the instance

    #cv2.destroyAllWindows();


#Initializing the face detector and landmark detector


def compute(ptA,ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a,b,c,d,e,f):
    up = compute(b,d) + compute(c,e)
    down = compute(a,f)
    ratio = up/(2.0*down)

    #Checking if it is blinked
    if(ratio>0.25):
         return 2
    elif(ratio>0.19 and ratio<=0.25):
        return 1
    else:
        return 0


camera=cv2.VideoCapture(0)

def generate_frames():
    while True:
        #read the camera frame
        
        success,frame=camera.read()
        if not success:
            break
        else:
            
            detector = dlib.get_frontal_face_detector()
            predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

            #status marking for current state
            sleep = 0
            drowsy = 0
            active = 0
            face1 = 0
            status=""
            color=(0,0,0)

            global alarm_status
            global alarm_status2
            global alarm_status3
            global saying



            while True:
                
                face_frame, frame = camera.read()
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = detector(gray)
                if(len(faces)==0):
                                face1+=1
                                if(face1>6 and alarm_status == False):
                                    alarm_status = True
                                    t = Thread(target=alarm, args=('Face Not Found',))
                                    t.deamon = True
                                    t.start()
                #detected face in faces array
                for face in faces:
                    x1 = face.left()
                    y1 = face.top()
                    x2 = face.right()
                    y2 = face.bottom()
                    
                    face_frame = frame.copy()
                    cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    landmarks = predictor(gray, face)
                    landmarks = face_utils.shape_to_np(landmarks)

                    #The numbers are actually the landmarks which will show eye
                    left_blink = blinked(landmarks[36],landmarks[37], 
                        landmarks[38], landmarks[41], landmarks[40], landmarks[39])
                    right_blink = blinked(landmarks[42],landmarks[43], 
                        landmarks[44], landmarks[47], landmarks[46], landmarks[45])
                    
                    #Now judge what to do for the eye blinks
                    if(left_blink==0 or right_blink==0):
                        sleep+=1
                        drowsy=0
                        active=0
                        if(sleep>6):
                            status="SLEEPING !!!"
                            color = (255,0,0)
                            if(alarm_status == False):
                                alarm_status = True
                                t = Thread(target=alarm, args=('wake up',))
                                t.deamon = True
                                t.start()
                    #elif(left_blink!=0 or right_blink!=0):
                        #alarm_status=False
                        #status="active"
                    
                    elif(left_blink==1 or right_blink==1):
                        alarm_status=False
                        sleep=0
                        active=0
                        drowsy+=1
                        if(drowsy>6):
                            status="Drowsy!!!"
                            color = (0,0,255)
                            if alarm_status2 == False and saying == False:
                                alarm_status2 = True
                                t = Thread(target=alarm, args=('take some fresh air',))
                                t.deamon = True
                                t.start()
                    #elif(left_blink!=1 or right_blink!=1):
                    # alarm_status2=False
                        #status="active"
                    


                    else:
                        alarm_status=False
                        alarm_status2=False
                        drowsy=0
                        sleep=0
                        active+=1
                        if(active>6):
                            status="Active :)"
                            color = (0,255,0)
                        
                    cv2.putText(frame, status, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color,3)

                    for n in range(0, 68):
                        (x,y) = landmarks[n]
                        cv2.circle(face_frame, (x, y), 1, (255, 255, 255), -1)

                """else:
                    t = Thread(target=alarm, args=('Face not found',))
                    t.deamon = True
                    t.start()
                   """ 

                cv2.imshow("Frame", frame)
                cv2.imshow("Result of detector", face_frame)
                
                success,frameNew=camera.read()
                if not success:
                    break
                else:
                    ret,buffer=cv2.imencode('.jpg',frameNew)
                    frame=buffer.tobytes()
                #return frame---> it will take only one and two frame to return then close
                yield(b'--frame\r\n'
                            b'Content-Type:image/jpeg\r\n\r\n'+ frame+ b'\r\n')
            
                key = cv2.waitKey(1)
                
                if key == 27:
                    break


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video')
def video():
    return Response(generate_frames(),mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/run", methods=["GET", "POST"])
def run_code():
    exit(0)
@app.route('/stopServer', methods=['GET'])
def stopServer():
    os.kill(os.getpid(), signal.SIGINT)
    return jsonify({ "success": True, "message": "Server is shutting down..." })
if __name__=='__main__':
    app.run(debug=True)



#from flask import Flask,render_template
#app=Flask(__name__,template_folder='template')
#@app.route("/") #this is endpoint and it cant be redefined
#def home():
    ##return "Hello World"
#    return render_template('index.html')
#if __name__=='__main__':
#    app.run(debug=True)

