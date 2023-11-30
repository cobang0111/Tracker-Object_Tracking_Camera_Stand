import platform
import argparse
import time
import sys
import os
from threading import Thread

import datetime
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# 환경 변수
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

CONFIDENCE_THRESHOLD = 0.6
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

model = YOLO('yolov8n.pt')
# Info remain until 50 frame
tracker = DeepSort(max_age=50)

# gstreamer
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate_1=120,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate_1,
            flip_method,
            display_width,
            display_height,
        )
    )


# GIPO pin setup
'''
servo1_gpio = 17
servo2_gpio = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1_gpio, GPIO.OUT)
GPIO.setup(servo2_gpio, GPIO.OUT)

# reset servo
servo1 = GPIO.PWM(servo1_gpio,50) # PWM frequency setup
servo2 = GPIO.PWM(servo2_gpio,50) # PWM frequency setup



# Motor Position Initializing
try:
	servo1.start(6.85)
		# 180deg = 11.7
		# 90deg = 6.85
		# 0deg = 2.2
		
	servo2.start(6.9)
		# 180deg = 11.9
		# 90deg = 6.9
		# 0deg = 1.9

except:
    pass

'''

#lr_dir = 6.9
#ud_dir = 6.85



# ******************** FUNCTION *********************

# 물체를 좌우 타겟 위치에 놓기 위해 로봇 동작을 단 한 번 조절하는 함수
def obj_x_centering(obj_x_center):
    global W_View_size
    global lr_dir

    if obj_x_center < ((W_View_size)/2 * 0.9) :
        print("object_x_center = {:.1f} -> Need to turn left!".format(obj_x_center))
        lr_dir += 0.3
        #servo2.ChangeDutyCycle(lr_dir) 

    elif obj_x_center > ((W_View_size)/2 * 1.1) :
        print("object_x_center = {:.1f} -> Need to turn right!".format(obj_x_center))
        lr_dir -= 0.3
        #servo2.ChangeDutyCycle(lr_dir)

    else :
        print("robot already satisfy the standard!")


#물체를 화면 y축 가운데 놓기 위해 로봇 헤드 각도를 한 번 조절하는 함수
def obj_y_centering(obj_y_center):
    global H_View_size
    global ud_dir
    # Head position control
    if obj_y_center > (H_View_size)/3 * 1.15 :
        print("object_y_center = {:.1f} -> Need to head down!".format(obj_y_center))
        ud_dir -= 0.3
        #servo1.ChangeDutyCycle(ud_dir)

    elif obj_y_center < (H_View_size)/3 * 0.85:
        ud_dir += 0.3
        #servo1.ChangeDutyCycle(ud_dir)
        print("object_y_center = {:.1f} -> Need to head up!".format(obj_y_center))

    else:
        print("Head already satisfy the standard!")


# Open Information

print(cv2.cuda.getCudaEnabledDeviceCount())
os_version = platform.platform()
print (" ---> OS " + os_version)
python_version = ".".join(map(str, sys.version_info[:3]))
print (" ---> Python " + python_version)
opencv_version = cv2.__version__
print (" ---> OpenCV  " + opencv_version)
   
#-------------------------------------

# video setting
W_View_size = 640 #800
H_View_size =480 #500
#View_select = 0 # Initial = Fast mode
#init = 1

# Open Video channel

print(gstreamer_pipeline(flip_method=0))
camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    #start = datetime.datetime.now()
    ret, frame = camera.read()
    if not ret:
        print('Camera Error')
        break

    detection = model.predict(source=[frame], save=False)[0]
    results = []
    
    for data in detection.boxes.data.tolist(): # data : [xmin, ymin, xmax, ymax, confidence_score, class_id]
        confidence = float(data[4])
        if confidence < CONFIDENCE_THRESHOLD:
            continue

        xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
        label = int(data[5])
        #cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        #cv2.putText(frame, class_list[label]+' '+str(round(confidence, 3)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
        
        if label == 0:
            results.append([[xmin, ymin, xmax-xmin, ymax-ymin], confidence, label])
    
    #tracker means DeepSort
    tracks = tracker.update_tracks(results, frame=frame)
    
    for track in tracks:
        if not track.is_confirmed():
            continue
        
        track_id = track.track_id
        ltrb = track.to_ltrb()

        xmin, ymin, xmax, ymax = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])

        # Draw green boundary
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
        # Draw id rectangle
        cv2.rectangle(frame, (xmin, ymin - 20), (xmin + 20, ymin), GREEN, -1)
        # Input id in rectangle
        cv2.putText(frame, str(track_id), (xmin + 5, ymin - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
