import platform
import argparse
import cv2
import serial
import time
import sys
from threading import Thread

from ultralytics import YOLO

model = YOLO("yolov8n.pt")

# Serial Functions
#----------------------------------------------- 

# Send serial

def TX_data(ser, one_byte):  # one_byte= 0~255
    ser.write(serial.to_bytes([one_byte]))  #python3

#-----------------------------------------------

# Recieve serial

def RX_data(ser):
    global Temp_count
    try:
        if ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            return RX
        else:
            return 0
    except:
        Temp_count = Temp_count  + 1
        print("Serial Not Open " + str(Temp_count))
        return 0
        
#-----------------------------------------------

def RX_Receiving(ser):
    global receiving_exit,threading_Time

    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle


    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print ("RX=" + str(RX))

#-----------------------------------------------             

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
    W_View_size = 800
    H_View_size = 500
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

    #---------------------------

    # Serial

    # Serial Port setting
    BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
    serial_use = 0
    
    # Connect to serial
    if serial_use != 0:  # python3
        BPS =  4800  # 4800, 9600, 14400, 19200, 28800, 57600, 115200
        #---------local Serial Port : ttyS0 --------
        #---------USB Serial Port : ttyAMA0 --------
        serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        serial_port.flush() # serial cls
        time.sleep(0.5)
    
        serial_t = Thread(target=RX_Receiving, args=(serial_port,))
        serial_t.daemon = True
        serial_t.start()
        
    # Example of serial sending
    #TX_data(serial_port, 250)


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

            # print all detected object
            for box in boxes :
                # x, y coordination list
                coor_list = box.xyxy.cpu().detach().numpy().tolist()[0] 
                x1 = coor_list[0] #x1 (Upper Left)
                y1 = coor_list[1] #y1 (Upper Left)
                x2 = coor_list[2] #x2 (Lower Right)
                y2 = coor_list[3] #y2 (Lower Right)
                print("x1 = {:.1f}, y1 = {:.1f}, x2 = {:.1f}, y2 = {:.1f}".format(x1,y1,x2,y2))
                # confidence
                print("conf = {:.2f}".format(*box.conf.cpu().detach().numpy().tolist())) 
                # object class
                print("class = {:d}".format(int(*box.cls.cpu().detach().numpy().tolist()))) 


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







