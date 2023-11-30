import platform
import argparse
import cv2
import time
import sys
import os
from threading import Thread

#import RPi.GPIO as GPIO

from ultralytics import YOLO

model = YOLO("yolov8n.pt")


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

lr_dir = 6.9
ud_dir = 6.85



# ******************** sg CUSTOMIZE FUNCTION *********************

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

    
# ***************   Main function   ****************
if __name__ == '__main__':

    #-------------------------------------

    os_version = platform.platform()
    print (" ---> OS " + os_version)
    python_version = ".".join(map(str, sys.version_info[:3]))
    print (" ---> Python " + python_version)
    opencv_version = cv2.__version__
    print (" ---> OpenCV  " + opencv_version)
   
    #-------------------------------------

    # video setting
    W_View_size = 1536 #800
    H_View_size = 864 #500
    View_select = 0 # Initial = Fast mode
    init = 1

    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size) )
    print ("")
    print ("-------------------------------------")
    
    #-------------------------------------

    # Get video port information        
    
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())

    if not args.get("video", False):
        camera = cv2.VideoCapture(0)
    else:
        camera = cv2.VideoCapture(args["video"])

    time.sleep(0.5)


    # -------- Main Loop Start --------
    while camera.isOpened():
        
        # grab the current frame
        grabbed, frame = camera.read()

        if grabbed:
            results = model(frame)
            boxes = results[0].boxes
            if init:
                cv2.imshow("yolov8n Results", results[0].plot())
                init = 0
            elif View_select == 0: # Fast operation 
                pass
            else:
                cv2.imshow("yolov8n Results", results[0].plot())

            x_standard = 0
            y_standard = 0
            total_n = 0

            # print all detected object
            for box in boxes :
                # confidence
                obj_conf = box.conf.cpu().detach().numpy().tolist()
                #print("conf = {:.2f}".format(*obj_conf)) 
                # object class
                obj_class = int(*box.cls.cpu().detach().numpy().tolist())
                #print("class = {:d}".format(obj_class) )

                if obj_class == 0 and obj_conf[0] >= 0.7:
                     # x, y coordination list
                    coor_list = box.xyxy.cpu().detach().numpy().tolist()[0] 
                    x1 = coor_list[0] #x1 (Upper Left)
                    y1 = coor_list[1] #y1 (Upper Left)
                    x2 = coor_list[2] #x2 (Lower Right)
                    y2 = coor_list[3] #y2 (Lower Right)

                    area = (x2-x1) * (y2-y1)

                    x_standard += (x1+x2)/2 * (area/10)
                    y_standard += (y1+y2)/3 * (area/10)

                    total_n += area/10

                if x_standard or y_standard:
                    x = x_standard/total_n
                    y = y_standard/total_n
                    print("x_standard = {:.1f}, y_standard = {:.1f}".format(x, y))

                    obj_x_centering(x)
                    obj_y_centering(y)    
                    
                
        key = 0xFF & cv2.waitKey(1) # waiting keypress
        
        if key == 27:  # ESC  Key -> Exit
            break

        elif key == ord(' '):  # spacebar Key -> mode change
            # Change mode : Fast operation -> show cam
            if View_select == 0: 
                View_select = 1
            # Change mode : show cam-> Fast operation
            else:
                View_select = 0

    # cleanup the camera and close any open windows
    receiving_exit = 0
    time.sleep(0.5)
    camera.release()
    cv2.destroyAllWindows()

    #servo1.stop()
    #servo2.stop()
    #GPIO.cleanup()




