import platform
import argparse
import time
import sys
sys.path.append('/usr/lib/python3.8/site-packages/cv2/python-3.8')

import os
from threading import Thread

import datetime
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

import Jetson.GPIO as GPIO
import time


# 환경 변수
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

CONFIDENCE_THRESHOLD = 0.6
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

servo1_pin = 33
servo2_pin = 32

GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

servo1 = GPIO.PWM(servo1_pin, 50) #50Hz
servo1.start(duty_cycle_x)

servo2 = GPIO.PWM(servo2_pin, 50) #50Hz
servo2.start(duty_cycle_y)


duty_cycle_x = 6.9
duty_cycle_y = 7.2

frame_count = 0

x_center = 320
y_center = 240
x=320
y=240
dx = 0
dy = 0




model = YOLO('yolov8n.pt')
# Info remain until 8 frame
tracker = DeepSort(max_age=8)

def motor1_control(dx, dy):

    global duty_cycle_x, duty_cycle_y
    dt = 10
    print("dx = {:.2f}".format(dx), "| dy = {:.2f}".format(dy))

    for i in range(dt):
        if abs(dx) > 64:
           duty_cycle_x -= 0.00005*dx
           
        if abs(dy) > 48:
           duty_cycle_y += 0.00001*dy
                
        if duty_cycle_x >= 11.9:
            duty_cycle_x = 11.9

        elif duty_cycle_x < 1.9:
            duty_cycle_x = 1.9

        elif duty_cycle_y >= 12.2:
            duty_cycle_y = 12.2

        elif duty_cycle_y < 2.2:
            duty_cycle_y = 2.2


        servo1.ChangeDutyCycle(duty_cycle_x)
        time.sleep(1/200)
        servo2.ChangeDutyCycle(duty_cycle_y)
        time.sleep(1/200)

    print("duty_cycle_x = {:.2f}".format(duty_cycle_x), "| duty_cycle_y = {:.2f}".format(duty_cycle_y))




# gstreamer
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate_1=60,
    framerate_2=9,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1, format=(string)BGRx ! "
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
            framerate_2,
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

    min_track_id = 10000
    min_xmin = min_ymin = min_xmax = min_ymax = 0

    for track in tracks:
        if not track.is_confirmed():
            continue
        
        track_id = track.track_id
        ltrb = track.to_ltrb()

        xmin, ymin, xmax, ymax = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])

        if track_id < min_track_id:
            min_track_id = track_id
            min_xmin = xmin
            min_ymin = ymin
            min_xmax = xmax
            min_ymax = ymax


        dx = min_xmin - x_center
        dy = min_ymin - y_center

        # Draw green boundary
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
        # Draw id rectangle
        cv2.rectangle(frame, (xmin, ymin - 20), (xmin + 20, ymin), GREEN, -1)
        # Input id in rectangle
        cv2.putText(frame, str(track_id), (xmin + 5, ymin - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)    


    cv2.imshow('frame', frame)

    motor1_control(dx, dy)

    if cv2.waitKey(1) == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
servo1.ChangeDutyCycle(6.9)
servo2.ChangeDutyCycle(7.2)

servo1.stop()
servo2.stop()
GPIO.cleanup()
#yolov7_wrapper.destroy()